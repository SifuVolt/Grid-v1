
"""Grid ground steering controller.

Combines target-relative steering with PD damping.

Inspired by:
- RLBotPack BitBot target-relative steering.
- RLBotPack Molten PD steering.
- Party Cannon's turn-aware driving model.

This module is independent of RLBot packet types.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, cos, hypot, pi, sin

from physics.math3d import Vec3, clamp


@dataclass
class SteeringResult:
    steer: float
    angle: float
    angular_rate: float
    reverse: bool
    aligned: bool
    reason: str


@dataclass
class SteeringConfig:
    proportional_gain: float = 3.8
    derivative_gain: float = 0.75
    max_steer: float = 1.0
    deadzone: float = 0.015
    alignment_tolerance: float = 0.035
    reverse_angle: float = 2.35
    reverse_speed_limit: float = 650.0
    reverse_forward_speed_limit: float = 500.0
    smoothing: float = 0.0


def _flat(vector: Vec3) -> Vec3:
    return Vec3(vector.x, vector.y, 0.0)


def _normalise(vector: Vec3) -> Vec3:
    vector = _flat(vector)
    magnitude = hypot(vector.x, vector.y)

    if magnitude < 0.001:
        return Vec3(0.0, 0.0, 0.0)

    return Vec3(
        vector.x / magnitude,
        vector.y / magnitude,
        0.0,
    )


def _signed_angle(forward: Vec3, target: Vec3) -> float:
    forward = _normalise(forward)
    target = _normalise(target)

    if forward.magnitude < 0.001 or target.magnitude < 0.001:
        return 0.0

    cross = forward.x * target.y - forward.y * target.x
    dot = clamp(forward.dot(target), -1.0, 1.0)

    return atan2(cross, dot)


def _get_forward(car) -> Vec3:
    forward = getattr(car, "forward", None)

    if callable(forward):
        forward = forward()

    if forward is not None:
        return _normalise(forward)

    yaw = float(getattr(car, "yaw", 0.0))
    return Vec3(cos(yaw), sin(yaw), 0.0)


def _get_velocity(car) -> Vec3:
    velocity = getattr(car, "velocity", None)

    if velocity is None:
        return Vec3(0.0, 0.0, 0.0)

    return velocity


def _get_yaw_rate(car) -> float:
    """Read yaw angular velocity when available."""

    angular_velocity = getattr(car, "angular_velocity", None)

    if angular_velocity is None:
        return 0.0

    # Rocket League angular velocity is a world-space vector.
    # Project it onto world Z for ground steering.
    return float(getattr(angular_velocity, "z", 0.0))


def _cap(value: float, maximum: float) -> float:
    return clamp(value, -maximum, maximum)


class SteeringController:
    """Stateful PD steering controller."""

    def __init__(
        self,
        config: SteeringConfig | None = None,
    ):
        self.config = config or SteeringConfig()
        self.previous_steer = 0.0
        self.last_angle = 0.0
        self.last_result = SteeringResult(
            steer=0.0,
            angle=0.0,
            angular_rate=0.0,
            reverse=False,
            aligned=False,
            reason="not initialised",
        )

    def reset(self) -> None:
        self.previous_steer = 0.0
        self.last_angle = 0.0
        self.last_result = SteeringResult(
            steer=0.0,
            angle=0.0,
            angular_rate=0.0,
            reverse=False,
            aligned=False,
            reason="reset",
        )

    def update(
        self,
        car,
        target: Vec3,
        dt: float = 1.0 / 120.0,
        *,
        reverse: bool = False,
        allow_reverse: bool = True,
        target_direction: Vec3 | None = None,
    ) -> SteeringResult:

        dt = clamp(dt, 0.001, 0.1)

        forward = _get_forward(car)
        velocity = _get_velocity(car)
        speed = hypot(velocity.x, velocity.y)
        forward_speed = velocity.dot(forward)

        if target_direction is None:
            target_direction = target - car.location

        target_direction = _normalise(target_direction)

        if target_direction.magnitude < 0.001:
            result = SteeringResult(
                steer=0.0,
                angle=0.0,
                angular_rate=_get_yaw_rate(car),
                reverse=reverse,
                aligned=True,
                reason="target is at current position",
            )
            self.last_result = result
            self.previous_steer = 0.0
            return result

        angle = _signed_angle(forward, target_direction)

        # Reverse only when the target is substantially behind us,
        # the car is moving slowly, and the forward velocity is low.
        if allow_reverse and not reverse:
            if (
                abs(angle) > self.config.reverse_angle
                and speed < self.config.reverse_speed_limit
                and forward_speed < self.config.reverse_forward_speed_limit
            ):
                reverse = True

        if reverse:
            # When reversing, the car's rear must face the target.
            angle = _signed_angle(
                forward * -1.0,
                target_direction,
            )

        angular_rate = _get_yaw_rate(car)

        # PD controller: proportional target angle plus derivative
        # damping from the car's current angular velocity.
        #
        # Reverse changes the steering sign because the rear of the
        # car is the effective direction of travel.
        direction_sign = -1.0 if reverse else 1.0

        control = (
            self.config.proportional_gain * angle
            - self.config.derivative_gain * angular_rate
        )

        steer = _cap(
            control * direction_sign,
            self.config.max_steer,
        )

        # Smooth small changes to avoid rapid left-right oscillation.
        smoothing = clamp(
            self.config.smoothing,
            0.0,
            0.95,
        )

        if smoothing > 0.0:
            steer = (
                self.previous_steer * smoothing
                + steer * (1.0 - smoothing)
            )

        if abs(steer) < self.config.deadzone:
            steer = 0.0

        aligned = abs(angle) <= self.config.alignment_tolerance

        if aligned:
            reason = "aligned with target"
        elif reverse:
            reason = "reverse steering"
        elif abs(angular_rate) > 1.0:
            reason = "PD steering with angular damping"
        else:
            reason = "steering toward target"

        result = SteeringResult(
            steer=steer,
            angle=angle,
            angular_rate=angular_rate,
            reverse=reverse,
            aligned=aligned,
            reason=reason,
        )

        self.previous_steer = steer
        self.last_angle = angle
        self.last_result = result

        return result


def steer_toward_target(
    car,
    target: Vec3,
    dt: float = 1.0 / 120.0,
    *,
    reverse: bool = False,
    allow_reverse: bool = True,
) -> SteeringResult:
    """Stateless convenience steering calculation."""

    controller = SteeringController()
    return controller.update(
        car,
        target,
        dt,
        reverse=reverse,
        allow_reverse=allow_reverse,
    )


def steering_angle(
    forward: Vec3,
    target_direction: Vec3,
) -> float:
    """Return signed flat angle from forward to target direction."""

    return _signed_angle(forward, target_direction)


def pd_steer(
    angle: float,
    angular_rate: float = 0.0,
    proportional_gain: float = 3.8,
    derivative_gain: float = 0.75,
) -> float:
    """Standalone PD steering function."""

    output = (
        proportional_gain * angle
        - derivative_gain * angular_rate
    )

    return clamp(output, -1.0, 1.0)
