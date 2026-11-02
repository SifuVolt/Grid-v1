
"""Rocket League ground turning model.

Inspired by Party Cannon's Drive.cs and RLBotPack's
Bumblebee driving maneuver.

Provides:
- Speed-dependent turn radius.
- Speed remaining after a turn.
- Turn geometry and direction.
- Powerslide eligibility.
- Safe target speed for a turn.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, exp, pi

from physics.math3d import Vec3, clamp


MAX_SPEED = 2300.0
MIN_TURN_RADIUS = 145.0
MAX_TURN_RADIUS = 1136.0


@dataclass
class TurnGeometry:
    angle: float
    signed_angle: float
    radius: float
    turn_distance: float
    speed_after_turn: float
    direction: int


@dataclass
class PowerslideDecision:
    allowed: bool
    steer: float
    angle: float
    reason: str


def _flat(vector: Vec3) -> Vec3:
    return Vec3(vector.x, vector.y, 0.0)


def _unit(vector: Vec3) -> Vec3:
    magnitude = vector.magnitude
    if magnitude < 0.001:
        return Vec3(0.0, 0.0, 0.0)
    return vector / magnitude


def _lerp(value: float, start: float, end: float) -> float:
    return start + (end - start) * value


def turn_radius(speed: float) -> float:
    """Party Cannon's piecewise speed-to-turn-radius approximation."""

    speed = clamp(abs(speed), 0.01, MAX_SPEED)

    if speed <= 500.0:
        return _lerp(speed / 500.0, 145.0, 251.0)

    if speed <= 1000.0:
        return _lerp((speed - 500.0) / 500.0, 251.0, 425.0)

    if speed <= 1500.0:
        return _lerp((speed - 1000.0) / 500.0, 425.0, 727.0)

    if speed <= 1750.0:
        return _lerp((speed - 1500.0) / 250.0, 727.0, 909.0)

    return _lerp((speed - 1750.0) / 550.0, 909.0, 1136.0)


def speed_from_turn_radius(radius: float) -> float:
    """Inverse of the approximate turn-radius curve."""

    radius = clamp(radius, MIN_TURN_RADIUS, MAX_TURN_RADIUS)

    if radius <= 251.0:
        return _lerp((radius - 145.0) / 106.0, 0.0, 500.0)

    if radius <= 425.0:
        return _lerp((radius - 251.0) / 174.0, 500.0, 1000.0)

    if radius <= 727.0:
        return _lerp((radius - 425.0) / 302.0, 1000.0, 1500.0)

    if radius <= 909.0:
        return _lerp((radius - 727.0) / 182.0, 1500.0, 1750.0)

    return _lerp((radius - 909.0) / 227.0, 1750.0, MAX_SPEED)


def speed_after_turn(
    current_speed: float,
    angle: float,
    modifier: float = 1.0,
) -> float:
    """Estimate speed after completing a turn.

    Uses Party Cannon's empirical speed-loss equation.
    """

    current_speed = clamp(current_speed, -MAX_SPEED, MAX_SPEED)
    angle = clamp(abs(angle), 0.0, 2.0 * pi)
    modifier = max(0.01, modifier)

    factor = angle * 0.49 * modifier

    if current_speed > 1234.0:
        factor *= 0.2

    exponential = exp(factor) * factor

    result = (
        1234.0 * exponential + current_speed
    ) / (exponential + 1.0)

    return clamp(result, 0.0, MAX_SPEED)


def calculate_turn(
    forward: Vec3,
    velocity: Vec3,
    target_direction: Vec3,
) -> TurnGeometry:
    """Calculate approximate turn geometry toward a target direction."""

    forward = _unit(_flat(forward))
    target = _unit(_flat(target_direction))
    speed = _flat(velocity).magnitude

    if forward.magnitude < 0.001 or target.magnitude < 0.001:
        return TurnGeometry(
            0.0, 0.0, turn_radius(speed), 0.0,
            speed, 0,
        )

    dot_value = clamp(forward.dot(target), -1.0, 1.0)
    angle = acos(dot_value)

    cross_z = forward.x * target.y - forward.y * target.x
    direction = 1 if cross_z >= 0.0 else -1
    signed_angle = angle * direction

    radius = turn_radius(speed)
    distance = radius * angle
    remaining_speed = speed_after_turn(speed, angle)

    return TurnGeometry(
        angle=angle,
        signed_angle=signed_angle,
        radius=radius,
        turn_distance=distance,
        speed_after_turn=remaining_speed,
        direction=direction,
    )


def powerslide_decision(
    forward: Vec3,
    velocity: Vec3,
    target_direction: Vec3,
    grounded: bool,
    height: float,
    distance_to_target: float,
    steer: float,
) -> PowerslideDecision:
    """Decide whether a sharp ground turn should use handbrake.

    Avoids the common problem of holding handbrake through an entire
    approach or while the car is already sliding sideways.
    """

    forward_flat = _unit(_flat(forward))
    velocity_flat = _unit(_flat(velocity))
    target_flat = _unit(_flat(target_direction))

    if not grounded:
        return PowerslideDecision(
            False, steer, 0.0, "car is airborne"
        )

    if height >= 300.0:
        return PowerslideDecision(
            False, steer, 0.0, "car is too high"
        )

    if (
        forward_flat.magnitude < 0.001
        or velocity_flat.magnitude < 0.001
        or target_flat.magnitude < 0.001
    ):
        return PowerslideDecision(
            False, steer, 0.0, "direction is undefined"
        )

    alignment = velocity_flat.dot(forward_flat)

    if alignment < 0.85:
        return PowerslideDecision(
            False, steer, 0.0,
            "car is already sliding or moving sideways",
        )

    angle = acos(clamp(
        forward_flat.dot(target_flat), -1.0, 1.0
    ))

    if angle < 1.5:
        return PowerslideDecision(
            False, steer, angle, "turn does not require powerslide"
        )

    if distance_to_target > 3500.0 and abs(steer) < 0.8:
        return PowerslideDecision(
            False, steer, angle, "target is too far for powerslide"
        )

    return PowerslideDecision(
        True, steer, angle, "sharp aligned turn: powerslide allowed"
    )


def target_speed_for_turn(
    current_speed: float,
    angle: float,
    minimum_speed: float = 400.0,
) -> float:
    """Estimate a speed that allows the car to complete a turn."""

    radius = turn_radius(current_speed)
    turn_speed = speed_from_turn_radius(radius)

    # Larger turns need more speed reduction.
    angle_factor = clamp(abs(angle) / pi, 0.0, 1.0)
    estimated = current_speed + (
        turn_speed - current_speed
    ) * angle_factor

    return clamp(estimated, minimum_speed, MAX_SPEED)
