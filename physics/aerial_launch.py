
"""Aerial launch timing and pre-launch turning.

Inspired by Party Cannon's AerialShot.IsValid().

Evaluates:
- Whether the car can launch immediately.
- Whether it should turn before jumping.
- Approximate turn time and post-turn state.
- Whether the aerial target is high enough to justify launching.

This module does not issue controller inputs. It supplies a launch
decision for the aerial mechanic.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, cos, sin, sqrt

from physics.math3d import Vec3, clamp


MIN_AERIAL_HEIGHT = 300.0
MAX_CAR_SPEED = 2300.0
MIN_TURN_SPEED = 350.0
TURN_ACCELERATION = 3.5
TURN_RADIUS_MIN = 180.0
TURN_RADIUS_MAX = 1200.0


@dataclass
class AerialLaunchDecision:
    valid: bool
    jump_immediately: bool
    should_turn: bool
    turn_angle: float
    turn_time: float
    predicted_location: Vec3
    predicted_forward: Vec3
    predicted_velocity: Vec3
    reason: str


def _unit(vector: Vec3) -> Vec3:
    if vector.magnitude < 0.001:
        return Vec3(0.0, 0.0, 0.0)
    return vector.normalized()


def _flat(vector: Vec3) -> Vec3:
    return _unit(Vec3(vector.x, vector.y, 0.0))


def _angle(first: Vec3, second: Vec3) -> float:
    a = _flat(first)
    b = _flat(second)

    if a.magnitude < 0.001 or b.magnitude < 0.001:
        return 0.0

    return acos(clamp(a.dot(b), -1.0, 1.0))


def _turn_radius(speed: float) -> float:
    """Approximate Rocket League turn radius."""

    speed = clamp(speed, 0.0, MAX_CAR_SPEED)
    radius = 160.0 + speed * 0.38
    return clamp(radius, TURN_RADIUS_MIN, TURN_RADIUS_MAX)


def _turn_time(angle: float, speed: float) -> float:
    """Estimate time required to rotate toward a target."""

    if angle < 0.04:
        return 0.0

    speed = max(MIN_TURN_SPEED, speed)
    radius = _turn_radius(speed)

    angular_speed = max(
        1.0,
        speed / radius,
    )

    acceleration_time = sqrt(angle / TURN_ACCELERATION)

    return max(
        angle / angular_speed,
        acceleration_time,
    )


def _predict_turn(
    location: Vec3,
    velocity: Vec3,
    forward: Vec3,
    target: Vec3,
    angle: float,
    turn_time: float,
) -> tuple[Vec3, Vec3, Vec3]:
    """Approximate car state after completing a ground turn."""

    speed = velocity.magnitude
    direction = _flat(target - location)
    current_forward = _flat(forward)

    if direction.magnitude < 0.001:
        return location, current_forward, velocity

    if current_forward.magnitude < 0.001:
        current_forward = direction

    side = 1.0

    cross_z = (
        current_forward.x * direction.y
        - current_forward.y * direction.x
    )

    if cross_z < 0:
        side = -1.0

    right = Vec3(
        -current_forward.y * side,
        current_forward.x * side,
        0.0,
    )

    radius = _turn_radius(speed)

    # Circular-arc approximation of the turn.
    displacement = (
        right * (radius * (1.0 - cos(angle)))
        + current_forward * (radius * sin(angle))
    )

    predicted_location = location + displacement
    predicted_forward = direction

    # Turning costs some speed.
    predicted_speed = speed * max(
        0.55,
        1.0 - angle * 0.12,
    )

    predicted_velocity = predicted_forward * predicted_speed

    return (
        predicted_location,
        predicted_forward,
        predicted_velocity,
    )


def evaluate_aerial_launch(
    car,
    target_location: Vec3,
    time_remaining: float,
    drive_location: Vec3 | None = None,
) -> AerialLaunchDecision:
    """Decide whether to jump now or turn before launching."""

    current_forward = _flat(car.orientation.forward)
    current_velocity = car.velocity
    current_speed = current_velocity.magnitude

    if time_remaining <= 0.0:
        return AerialLaunchDecision(
            False, False, False, 0.0, 0.0,
            car.location, current_forward, current_velocity,
            "aerial intercept has expired",
        )

    if target_location.z < MIN_AERIAL_HEIGHT:
        return AerialLaunchDecision(
            False, False, False, 0.0, 0.0,
            car.location, current_forward, current_velocity,
            "target is too low for an aerial",
        )

    if drive_location is None:
        drive_location = Vec3(
            target_location.x,
            target_location.y,
            car.location.z,
        )

    target_direction = _flat(drive_location - car.location)

    if target_direction.magnitude < 0.001:
        target_direction = current_forward

    angle = _angle(current_forward, target_direction)

    turn_time = _turn_time(angle, current_speed)

    # Party Cannon launches immediately when already aligned,
    # moving slowly sideways, or already airborne.
    jump_immediately = (
        angle < 0.6
        or _flat(current_velocity).magnitude < 500.0
        or not car.on_ground
    )

    if jump_immediately:
        return AerialLaunchDecision(
            True,
            True,
            False,
            angle,
            0.0,
            car.location,
            current_forward,
            current_velocity,
            "immediate aerial launch is preferred",
        )

    predicted_location, predicted_forward, predicted_velocity = (
        _predict_turn(
            car.location,
            current_velocity,
            current_forward,
            drive_location,
            angle,
            turn_time,
        )
    )

    if turn_time >= time_remaining:
        return AerialLaunchDecision(
            False,
            False,
            True,
            angle,
            turn_time,
            predicted_location,
            predicted_forward,
            predicted_velocity,
            "not enough time to turn before the intercept",
        )

    return AerialLaunchDecision(
        True,
        False,
        True,
        angle,
        turn_time,
        predicted_location,
        predicted_forward,
        predicted_velocity,
        "turn first, then launch the aerial",
    )
