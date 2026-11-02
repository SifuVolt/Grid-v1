
"""Grid ground driving mechanic.

Combines:
- Party Cannon-style turn radius and speed-loss modelling.
- Bumblebee-style steering, speed control and powerslide conditions.
- Persistent target locking to prevent unnecessary steering changes.
- Conservative reverse driving.
- Explicit output for the Grid action manager.

This module does not directly interact with RLBot.
It returns controls for the caller to apply.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import acos, atan2, cos, hypot, pi, sin

from physics.math3d import Vec3, clamp
from physics.turning import (
    calculate_turn,
    powerslide_decision,
    speed_after_turn,
    speed_from_turn_radius,
    turn_radius,
)


class DriveStatus(Enum):
    IDLE = "idle"
    APPROACHING = "approaching"
    ARRIVED = "arrived"
    INTERRUPTED = "interrupted"
    INVALID = "invalid"


@dataclass
class DriveOutput:
    throttle: float = 0.0
    steer: float = 0.0
    boost: bool = False
    handbrake: bool = False
    reverse: bool = False
    finished: bool = False
    distance_remaining: float = 0.0
    target_speed: float = 0.0
    reason: str = ""

    @staticmethod
    def neutral(reason: str = "") -> "DriveOutput":
        return DriveOutput(reason=reason)


@dataclass
class DriveConfig:
    arrival_radius: float = 100.0
    target_lock_time: float = 0.18
    max_speed: float = 2300.0
    min_speed: float = 400.0
    boost_threshold: float = 80.0
    boost_angle: float = 0.30
    allow_boost: bool = True
    allow_handbrake: bool = True
    allow_reverse: bool = True
    stop_on_arrival: bool = True
    target_speed: float | None = None


def _flat_distance(a: Vec3, b: Vec3) -> float:
    return hypot(b.x - a.x, b.y - a.y)


def _flat(vector: Vec3) -> Vec3:
    return Vec3(vector.x, vector.y, 0.0)


def _unit(vector: Vec3) -> Vec3:
    magnitude = hypot(vector.x, vector.y)
    if magnitude < 0.001:
        return Vec3(0.0, 0.0, 0.0)
    return Vec3(vector.x / magnitude, vector.y / magnitude, 0.0)


def _angle_to_target(forward: Vec3, direction: Vec3) -> tuple[float, float]:
    forward = _unit(forward)
    direction = _unit(direction)

    if forward.magnitude < 0.001 or direction.magnitude < 0.001:
        return 0.0, 0.0

    dot_value = clamp(forward.dot(direction), -1.0, 1.0)
    angle = acos(dot_value)

    cross = forward.x * direction.y - forward.y * direction.x
    signed_angle = angle if cross >= 0.0 else -angle

    return angle, signed_angle


def _command_value(value: float) -> float:
    return clamp(value, -1.0, 1.0)


def _normalise_axis(value: float, deadzone: float = 0.02) -> float:
    if abs(value) < deadzone:
        return 0.0
    return _command_value(value)


def _get_forward(car: VehicleState) -> Vec3:
    forward = getattr(car, "forward", None)

    if callable(forward):
        forward = forward()

    if forward is not None:
        return _flat(forward)

    yaw = getattr(car, "yaw", 0.0)
    return Vec3(cos(yaw), sin(yaw), 0.0)


def _get_velocity(car: VehicleState) -> Vec3:
    velocity = getattr(car, "velocity", None)

    if velocity is None:
        return Vec3(0.0, 0.0, 0.0)

    return velocity


def _get_speed(car: VehicleState) -> float:
    speed = getattr(car, "flat_speed", None)

    if speed is not None:
        return abs(float(speed))

    velocity = _get_velocity(car)
    return hypot(velocity.x, velocity.y)


def _get_forward_speed(car: VehicleState, forward: Vec3) -> float:
    return _get_velocity(car).dot(forward)


def _get_grounded(car: VehicleState) -> bool:
    return bool(
        getattr(car, "on_ground",
        getattr(car, "grounded", False))
    )


def _get_height(car: VehicleState) -> float:
    return float(getattr(car.location, "z", 0.0))


class Drive:
    """Persistent ground-driving action."""

    def __init__(
        self,
        config: DriveConfig | None = None,
        target: Vec3 | None = None,
    ):
        self.config = config or DriveConfig()

        self.target = target
        self.target_lock_remaining = 0.0

        self.status = DriveStatus.IDLE
        self.last_output = DriveOutput.neutral()

        self.elapsed = 0.0
        self.distance_remaining = 0.0

        self.backwards = False
        self._reverse_lock = 0.0
        self._arrival_frames = 0

    def start(
        self,
        car: VehicleState,
        target: Vec3,
    ) -> DriveOutput:
        self.reset()
        self.target = target
        self.status = DriveStatus.APPROACHING
        self.target_lock_remaining = self.config.target_lock_time

        return self.update(car, target, force_target=True)

    def set_target(
        self,
        target: Vec3,
        force: bool = False,
    ) -> None:
        if self.target is None or force:
            self.target = target
            self.target_lock_remaining = self.config.target_lock_time
            return

        if self.target_lock_remaining <= 0.0:
            self.target = target
            self.target_lock_remaining = self.config.target_lock_time

    def interrupt(self) -> None:
        self.status = DriveStatus.INTERRUPTED
        self.last_output = DriveOutput.neutral("drive interrupted")

    def reset(self) -> None:
        self.target = None
        self.target_lock_remaining = 0.0
        self.status = DriveStatus.IDLE
        self.last_output = DriveOutput.neutral()
        self.elapsed = 0.0
        self.distance_remaining = 0.0
        self.backwards = False
        self._reverse_lock = 0.0
        self._arrival_frames = 0

    def is_finished(self) -> bool:
        return self.status in (
            DriveStatus.ARRIVED,
            DriveStatus.INTERRUPTED,
            DriveStatus.INVALID,
        )

    def update(
        self,
        car: VehicleState,
        target: Vec3 | None = None,
        dt: float = 1.0 / 120.0,
        *,
        allow_boost: bool | None = None,
        max_speed: float | None = None,
        target_speed: float | None = None,
        force_target: bool = False,
    ) -> DriveOutput:

        dt = clamp(dt, 0.0, 0.1)
        self.elapsed += dt
        self.target_lock_remaining = max(
            0.0, self.target_lock_remaining - dt
        )
        self._reverse_lock = max(0.0, self._reverse_lock - dt)

        if target is not None:
            self.set_target(target, force_target)

        if self.target is None:
            self.status = DriveStatus.INVALID
            self.last_output = DriveOutput.neutral(
                "no driving target"
            )
            return self.last_output

        if getattr(car, "demolished", False):
            self.status = DriveStatus.INVALID
            self.last_output = DriveOutput.neutral(
                "car is demolished"
            )
            return self.last_output

        output = self._calculate_output(
            car,
            dt,
            allow_boost=allow_boost,
            max_speed=max_speed,
            target_speed=target_speed,
        )

        self.last_output = output
        return output

    def _calculate_output(
        self,
        car: VehicleState,
        dt: float,
        *,
        allow_boost: bool | None,
        max_speed: float | None,
        target_speed: float | None,
    ) -> DriveOutput:

        target = self.target
        location = car.location
        forward = _get_forward(car)
        velocity = _get_velocity(car)

        delta = _flat(target - location)
        distance = delta.magnitude
        self.distance_remaining = distance

        if distance <= self.config.arrival_radius:
            self._arrival_frames += 1

            if self._arrival_frames >= 2:
                self.status = DriveStatus.ARRIVED

            return DriveOutput(
                throttle=0.0 if self.config.stop_on_arrival else 1.0,
                finished=self.status == DriveStatus.ARRIVED,
                distance_remaining=distance,
                reason="destination reached",
            )

        self._arrival_frames = 0
        self.status = DriveStatus.APPROACHING

        direction = _unit(delta)
        angle, signed_angle = _angle_to_target(
            forward, direction
        )

        speed = _get_speed(car)
        forward_speed = _get_forward_speed(car, forward)
        grounded = _get_grounded(car)
        height = _get_height(car)

        # Reverse is deliberately conservative. Party Cannon also
        # requires a substantial ETA advantage before selecting it.
        if self.config.allow_reverse and self._reverse_lock <= 0.0:
            should_reverse = (
                angle > 2.10
                and forward_speed < 500.0
                and speed < 700.0
                and grounded
            )

            if should_reverse != self.backwards:
                self.backwards = should_reverse
                self._reverse_lock = 0.35

        if not self.config.allow_reverse:
            self.backwards = False

        steering_direction = direction
        if self.backwards:
            steering_direction = direction * -1.0

        steering_angle, signed_steering_angle = _angle_to_target(
            forward, steering_direction
        )

        # Party Cannon turn model: estimate how much speed will be
        # lost while rotating toward the target.
        turn = calculate_turn(
            forward,
            velocity,
            direction,
        )

        speed_cap = max_speed or self.config.max_speed
        speed_cap = clamp(speed_cap, self.config.min_speed, 2300.0)

        requested_speed = (
            target_speed
            if target_speed is not None
            else self.config.target_speed
        )

        if requested_speed is None:
            requested_speed = speed_cap

        requested_speed = clamp(
            requested_speed,
            0.0,
            speed_cap,
        )

        # Estimate a practical speed for the required turn.
        # Short, sharp turns require a smaller radius and lower speed.
        sine = abs(sin(angle * 0.5))

        if sine > 0.05:
            required_radius = distance / (2.0 * sine)
        else:
            required_radius = turn_radius(speed)

        required_radius = clamp(
            required_radius,
            145.0,
            1136.0,
        )

        corner_speed = speed_from_turn_radius(required_radius)

        if angle > 0.35:
            corner_speed = min(
                corner_speed,
                speed_after_turn(
                    speed,
                    angle,
                    0.65,
                ),
            )

        desired_speed = min(
            requested_speed,
            max(self.config.min_speed, corner_speed),
        )

        # Avoid slowing down unnecessarily for a distant,
        # shallow adjustment.
        if angle < 0.25:
            desired_speed = requested_speed

        if self.backwards:
            desired_speed = min(desired_speed, 1000.0)

        # Steering proportional to angular error.
        steer = _command_value(
            signed_steering_angle * 2.5
        )

        # Bumblebee-inspired powerslide conditions, with the
        # additional restriction that the car must be grounded.
        slide = powerslide_decision(
            forward=forward,
            velocity=velocity,
            target_direction=direction,
            grounded=grounded,
            height=height,
            distance_to_target=distance,
            steer=steer,
        )

        handbrake = (
            self.config.allow_handbrake
            and not self.backwards
            and slide.allowed
        )

        # Speed controller.
        speed_error = desired_speed - speed

        if self.backwards:
            reverse_speed = max(0.0, -forward_speed)
            reverse_error = desired_speed - reverse_speed

            if reverse_error > 80.0:
                throttle = -1.0
            elif reverse_error < -350.0:
                throttle = 1.0
            else:
                throttle = 0.0

        elif speed_error > 70.0:
            throttle = 1.0
        elif speed_error < -400.0:
            throttle = -1.0
        elif speed_error < -100.0:
            throttle = 0.0
        else:
            throttle = 0.15 if speed_error > 0.0 else 0.0

        # When a sharp turn is required, brake rather than
        # continuing to accelerate into the corner.
        if not self.backwards and angle > 1.1:
            if speed > desired_speed + 100.0:
                throttle = -1.0

        can_boost = (
            self.config.allow_boost
            if allow_boost is None
            else allow_boost
        )

        boost = (
            can_boost
            and not self.backwards
            and not handbrake
            and getattr(car, "boost", 0.0) >= self.config.boost_threshold
            and speed < desired_speed - 80.0
            and angle < self.config.boost_angle
            and speed < min(speed_cap, 2250.0)
            and throttle > 0.0
        )

        if self.backwards:
            boost = False
            handbrake = False

        output = DriveOutput(
            throttle=_normalise_axis(throttle),
            steer=_normalise_axis(steer),
            boost=bool(boost),
            handbrake=bool(handbrake),
            reverse=self.backwards,
            finished=False,
            distance_remaining=distance,
            target_speed=desired_speed,
            reason=(
                "reverse approach" if self.backwards
                else slide.reason if handbrake
                else "turning" if angle > 0.5
                else "driving toward target"
            ),
        )

        return output


def drive_to(
    car: VehicleState,
    target: Vec3,
    *,
    target_speed: float = 2300.0,
    allow_boost: bool = True,
    dt: float = 1.0 / 120.0,
) -> DriveOutput:
    """One-shot driving helper."""

    drive = Drive(
        DriveConfig(
            target_speed=target_speed,
            allow_boost=allow_boost,
        ),
        target,
    )

    return drive.update(car, dt=dt)


def create_drive(
    target: Vec3,
    *,
    target_speed: float = 2300.0,
    allow_boost: bool = True,
) -> Drive:
    """Create a persistent Drive action."""

    return Drive(
        DriveConfig(
            target_speed=target_speed,
            allow_boost=allow_boost,
        ),
        target,
    )
