
"""Grid aerial orientation controller.

Uses local-space target direction, orientation error and angular velocity
to produce aerial pitch, yaw and roll controls.

Inspired by Party Cannon's AimAt aerial control and RLBotPack aerial
controllers.

RLBot controller convention:
- Pitch negative: nose up
- Pitch positive: nose down
- Yaw positive: turn right
- Roll positive: roll right

The controller returns control values; it does not manage jump or boost.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, pi

from physics.math3d import Vec3, clamp
from physics.vehicle import VehicleState


@dataclass
class AirControlConfig:
    direction_gain: float = 3.2
    angular_damping: float = 0.55

    pitch_gain: float = 1.0
    yaw_gain: float = 1.0
    roll_gain: float = 1.0

    maximum_pitch: float = 1.0
    maximum_yaw: float = 1.0
    maximum_roll: float = 1.0

    angular_velocity_deadzone: float = 0.025
    direction_deadzone: float = 0.015

    roll_correction_gain: float = 2.0
    upright_priority: float = 0.65


@dataclass
class AirControlOutput:
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0

    target_direction: Vec3 = Vec3(1.0, 0.0, 0.0)
    local_direction: Vec3 = Vec3(1.0, 0.0, 0.0)

    direction_error: float = 0.0
    angular_speed: float = 0.0

    aligned: bool = False


def _deadzone(value: float, threshold: float) -> float:
    if abs(value) <= threshold:
        return 0.0

    return value


def _safe_normalize(vector: Vec3) -> Vec3:
    if vector.length() < 1e-6:
        return Vec3(0.0, 0.0, 0.0)

    return vector.normalized()


def _wrap_angle(angle: float) -> float:
    while angle > pi:
        angle -= 2.0 * pi

    while angle < -pi:
        angle += 2.0 * pi

    return angle


def _target_up(
    target_direction: Vec3,
    preferred_up: Vec3,
) -> Vec3:
    """Construct a stable desired up vector for the target orientation."""

    forward = _safe_normalize(target_direction)
    up = _safe_normalize(preferred_up)

    if forward.length() < 0.5:
        return Vec3(0.0, 0.0, 1.0)

    if up.length() < 0.5:
        up = Vec3(0.0, 0.0, 1.0)

    right = _safe_normalize(up.cross(forward))

    if right.length() < 0.5:
        fallback = Vec3(0.0, 1.0, 0.0)
        right = _safe_normalize(fallback.cross(forward))

    if right.length() < 0.5:
        return up

    return _safe_normalize(forward.cross(right))


def _orientation_errors(
    car: VehicleState,
    target_direction: Vec3,
    preferred_up: Vec3,
) -> tuple[Vec3, Vec3, float]:
    """Return local direction, local angular error and angular error size."""

    orientation = car.orientation

    direction = _safe_normalize(target_direction)

    if direction.length() < 0.5:
        direction = orientation.forward

    local_direction = orientation.world_to_local(direction)

    desired_up = _target_up(direction, preferred_up)

    # The cross product gives the world-space axis that rotates
    # the current up vector toward the desired up vector.
    up_error_world = orientation.up.cross(desired_up)

    up_error_local = orientation.world_to_local(up_error_world)

    return (
        local_direction,
        up_error_local,
        direction.angle_to(orientation.forward),
    )


def aerial_control(
    car: VehicleState,
    target_direction: Vec3,
    *,
    preferred_up: Vec3 = Vec3(0.0, 0.0, 1.0),
    config: AirControlConfig | None = None,
) -> AirControlOutput:
    """Aim the car toward a world-space direction."""

    config = config or AirControlConfig()

    target_direction = _safe_normalize(
        Vec3.from_value(target_direction)
    )

    if target_direction.length() < 0.5:
        return AirControlOutput(
            target_direction=car.orientation.forward,
            local_direction=Vec3(1.0, 0.0, 0.0),
            aligned=True,
        )

    local_direction, up_error, direction_error = _orientation_errors(
        car,
        target_direction,
        preferred_up,
    )

    local_angular_velocity = car.orientation.world_to_local(
        car.angular_velocity
    )

    local_angular_velocity = Vec3(
        _deadzone(
            local_angular_velocity.x,
            config.angular_velocity_deadzone,
        ),
        _deadzone(
            local_angular_velocity.y,
            config.angular_velocity_deadzone,
        ),
        _deadzone(
            local_angular_velocity.z,
            config.angular_velocity_deadzone,
        ),
    )

    # Direction errors in the car's local coordinate frame.
    #
    # local_direction.y is the horizontal aiming error.
    # local_direction.z is the vertical aiming error.
    yaw_error = atan2(
        local_direction.y,
        max(0.001, local_direction.x),
    )

    pitch_error = atan2(
        local_direction.z,
        max(0.001, local_direction.x),
    )

    yaw_error = _deadzone(
        yaw_error,
        config.direction_deadzone,
    )

    pitch_error = _deadzone(
        pitch_error,
        config.direction_deadzone,
    )

    # RLBot pitch is negative for nose-up.
    pitch = (
        -pitch_error * config.direction_gain
        -local_angular_velocity.y * config.angular_damping
    )

    yaw = (
        yaw_error * config.direction_gain
        -local_angular_velocity.z * config.angular_damping
    )

    # Correct roll using the local up-axis error.
    roll = (
        -up_error.z * config.roll_correction_gain
        -local_angular_velocity.x * config.angular_damping
    )

    # When the target is almost directly above or below, prioritise
    # keeping the car upright instead of producing unstable yaw input.
    vertical_alignment = abs(local_direction.z)

    if vertical_alignment > 0.92:
        yaw *= 1.0 - config.upright_priority

    pitch = clamp(
        pitch * config.pitch_gain,
        -config.maximum_pitch,
        config.maximum_pitch,
    )

    yaw = clamp(
        yaw * config.yaw_gain,
        -config.maximum_yaw,
        config.maximum_yaw,
    )

    roll = clamp(
        roll * config.roll_gain,
        -config.maximum_roll,
        config.maximum_roll,
    )

    angular_speed = car.angular_velocity.length()

    return AirControlOutput(
        pitch=pitch,
        yaw=yaw,
        roll=roll,
        target_direction=target_direction,
        local_direction=local_direction,
        direction_error=direction_error,
        angular_speed=angular_speed,
        aligned=direction_error < 0.08 and angular_speed < 0.5,
    )


def aim_at_position(
    car: VehicleState,
    target: Vec3,
    *,
    preferred_up: Vec3 = Vec3(0.0, 0.0, 1.0),
    config: AirControlConfig | None = None,
) -> AirControlOutput:
    """Aim toward a world-space target position."""

    direction = car.location.direction_to(
        Vec3.from_value(target)
    )

    return aerial_control(
        car,
        direction,
        preferred_up=preferred_up,
        config=config,
    )


def aim_at_shot(
    car: VehicleState,
    shot_direction: Vec3,
    ball_direction: Vec3,
    config: AirControlConfig | None = None,
) -> AirControlOutput:
    """Aim toward the shot direction while retaining ball awareness."""

    shot_direction = _safe_normalize(shot_direction)
    ball_direction = _safe_normalize(ball_direction)

    if shot_direction.length() < 0.5:
        shot_direction = ball_direction

    if shot_direction.length() < 0.5:
        shot_direction = car.orientation.forward

    # Keep the car's nose aimed toward the shot, but use the ball
    # direction as the preferred orientation reference.
    preferred_up = _target_up(
        ball_direction,
        Vec3(0.0, 0.0, 1.0),
    )

    return aerial_control(
        car,
        shot_direction,
        preferred_up=preferred_up,
        config=config,
    )
