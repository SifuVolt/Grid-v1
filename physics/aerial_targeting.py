
"""Aerial shot targeting.

Inspired by Party Cannon's AerialShot.SetTargetLocation.

Calculates:
- Desired outgoing ball direction.
- Car contact position behind the ball.
- A surface-aware adjustment near walls and the floor.
- A stable target for aerial reachability and steering.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import asin, cos, sin

from physics.math3d import Vec3, clamp
from physics.field import FIELD_HALF_WIDTH, FIELD_HALF_LENGTH


BALL_RADIUS = 92.75
CAR_CONTACT_OFFSET = 155.0
SURFACE_CLEARANCE = 50.0


@dataclass
class AerialTarget:
    contact_location: Vec3
    shot_direction: Vec3
    shot_target: Vec3
    valid: bool
    reason: str


def _unit(vector: Vec3) -> Vec3:
    if vector.magnitude < 0.001:
        return Vec3(0.0, 0.0, 0.0)
    return vector.normalized()


def _flat_unit(vector: Vec3) -> Vec3:
    return _unit(Vec3(vector.x, vector.y, 0.0))

def _cap_magnitude(vector: Vec3, maximum: float) -> Vec3:
    magnitude = vector.magnitude
    if magnitude > maximum and magnitude > 0.001:
        return vector.normalized() * maximum
    return vector

def _nearest_surface(location: Vec3) -> tuple[Vec3, float]:
    """Return an approximate nearest surface normal and distance."""

    candidates = [
        (Vec3(1.0, 0.0, 0.0), FIELD_HALF_WIDTH - abs(location.x)),
        (Vec3(0.0, 1.0, 0.0), FIELD_HALF_LENGTH - abs(location.y)),
        (Vec3(0.0, 0.0, 1.0), location.z),
    ]

    normal, distance = min(candidates, key=lambda item: item[1])

    if normal.x and location.x < 0:
        normal = Vec3(-1.0, 0.0, 0.0)
    elif normal.y and location.y < 0:
        normal = Vec3(0.0, -1.0, 0.0)

    return normal, max(0.0, distance)


def _adjust_for_surface(
    ball_location: Vec3,
    direction: Vec3,
) -> Vec3:
    contact = ball_location - direction * CAR_CONTACT_OFFSET
    normal, distance = _nearest_surface(contact)

    if distance >= SURFACE_CLEARANCE:
        return direction

    tangent = direction - normal * direction.dot(normal)
    tangent = _unit(tangent)

    if tangent.magnitude < 0.001:
        return direction

    angle = asin(
        clamp(
            (distance - SURFACE_CLEARANCE) / CAR_CONTACT_OFFSET,
            -1.0,
            1.0,
        )
    )

    adjusted = tangent * cos(angle) + normal * sin(angle)
    return _unit(adjusted)


def calculate_aerial_target(
    ball_location: Vec3,
    ball_velocity: Vec3,
    car_location: Vec3,
    time_remaining: float,
    shot_target: Vec3,
    contact_offset: float = CAR_CONTACT_OFFSET,
) -> AerialTarget:
    """Calculate the car's desired aerial contact location.

    The predicted car finish velocity is blended with ball velocity
    to estimate the outgoing shot, following Party Cannon's approach.
    """

    if time_remaining <= 0.0:
        return AerialTarget(
            ball_location,
            Vec3(0.0, 0.0, 0.0),
            shot_target,
            False,
            "contact time has passed",
        )

    if contact_offset <= 0.0:
        return AerialTarget(
            ball_location,
            Vec3(0.0, 0.0, 0.0),
            shot_target,
            False,
            "contact offset must be positive",
        )

    car_finish_velocity = (ball_location - car_location) / time_remaining
    car_finish_velocity = _cap_magnitude(car_finish_velocity, 2300.0)

    direction_to_target = _flat_unit(shot_target - ball_location)

    if direction_to_target.magnitude < 0.001:
        return AerialTarget(
            ball_location,
            Vec3(0.0, 0.0, 0.0),
            shot_target,
            False,
            "shot target is at the ball location",
        )

    # Estimate the ball's outgoing velocity after contact.
    outgoing_velocity = (
        car_finish_velocity + ball_velocity * 2.0
    ) / 3.0

    # Blend the desired shot direction with the estimated outgoing
    # velocity so the car doesn't aim for an unrealistic sharp turn.
    shot_direction = _unit(
        direction_to_target * 0.75
        + _unit(outgoing_velocity) * 0.25
    )

    if shot_direction.magnitude < 0.001:
        shot_direction = direction_to_target

    shot_direction = _adjust_for_surface(
        ball_location,
        shot_direction,
    )

    contact_location = (
        ball_location - shot_direction * contact_offset
    )

    return AerialTarget(
        contact_location=contact_location,
        shot_direction=shot_direction,
        shot_target=shot_target,
        valid=True,
        reason="aerial contact target calculated",
    )


def calculate_clear_target(
    ball_location: Vec3,
    team: int,
    distance: float = 5000.0,
) -> Vec3:
    """Choose a safe forward clear target toward the opponent's half."""

    direction = 1.0 if team == 0 else -1.0

    return Vec3(
        ball_location.x * 0.35,
        direction * distance,
        max(500.0, ball_location.z),
    )
