
"""Grid jump trajectory prediction.

Predicts the car's future position and velocity during a single or
double jump. This is a lightweight model intended for shot planning
and aerial reachability.

The interface follows the future-position approach used by
Party Cannon's LocationAfterJump / LocationAfterDoubleJump logic.
"""

from __future__ import annotations

from dataclasses import dataclass

from physics.math3d import Vec3
from physics.vehicle import VehicleState


GRAVITY = 650.0
JUMP_IMPULSE = 291.67
JUMP_ACCELERATION = 1458.33

DEFAULT_JUMP_HOLD = 0.20
DEFAULT_RELEASE_FRAMES = 3
DEFAULT_SECOND_JUMP_FRAMES = 3


@dataclass
class JumpPrediction:
    location: Vec3
    velocity: Vec3
    time: float
    double_jump: bool
    valid: bool = True
    reason: str = ""


def _integrate_vertical(
    position: float,
    velocity: float,
    acceleration: float,
    duration: float,
) -> tuple[float, float]:
    """Integrate one constant-acceleration vertical segment."""
    if duration <= 0.0:
        return position, velocity

    new_position = (
        position
        + velocity * duration
        + 0.5 * acceleration * duration * duration
    )
    new_velocity = velocity + acceleration * duration

    return new_position, new_velocity


def _simulate_jump_vertical(
    initial_height: float,
    initial_velocity: float,
    future_time: float,
    jump_elapsed: float,
    double_jump: bool,
    jump_hold_time: float,
    release_frames: int,
    second_jump_frames: int,
    started_on_ground: bool,
) -> tuple[float, float]:
    """Predict vertical position and velocity from the current snapshot.

    jump_elapsed is the time since the first jump began.

    If the car is still grounded and jump_elapsed is zero, the first
    jump impulse is applied at the beginning of this prediction.

    If the car is already airborne, its current vertical velocity is
    used as-is; the first jump impulse is not applied again.
    """
    position = initial_height
    velocity = initial_velocity

    if future_time <= 0.0:
        return position, velocity

    if started_on_ground and jump_elapsed <= 0.0:
        velocity += JUMP_IMPULSE
        jump_elapsed = 0.0

    hold_end = max(0.0, jump_hold_time)
    release_end = hold_end + max(0, release_frames) / 120.0
    second_jump_time = release_end

    current_elapsed = max(0.0, jump_elapsed)
    remaining = future_time

    # First-jump hold: jump acceleration is applied against gravity.
    if current_elapsed < hold_end and remaining > 0.0:
        segment = min(
            remaining,
            hold_end - current_elapsed,
        )

        position, velocity = _integrate_vertical(
            position,
            velocity,
            JUMP_ACCELERATION - GRAVITY,
            segment,
        )

        current_elapsed += segment
        remaining -= segment

    # Release period: normal gravity only.
    if current_elapsed < release_end and remaining > 0.0:
        segment = min(
            remaining,
            release_end - current_elapsed,
        )

        position, velocity = _integrate_vertical(
            position,
            velocity,
            -GRAVITY,
            segment,
        )

        current_elapsed += segment
        remaining -= segment

    # Second jump impulse is applied once, at the end of the release.
    if double_jump and current_elapsed >= second_jump_time:
        # If the snapshot was taken before the second-jump moment,
        # apply the impulse when this prediction crosses that moment.
        if jump_elapsed <= second_jump_time:
            velocity += JUMP_IMPULSE

            # Account for the second-jump press duration.
            press_duration = max(0, second_jump_frames) / 120.0
            segment = min(remaining, press_duration)

            if segment > 0.0:
                position, velocity = _integrate_vertical(
                    position,
                    velocity,
                    JUMP_ACCELERATION - GRAVITY,
                    segment,
                )

                current_elapsed += segment
                remaining -= segment

    # Remaining flight uses gravity.
    if remaining > 0.0:
        position, velocity = _integrate_vertical(
            position,
            velocity,
            -GRAVITY,
            remaining,
        )

    return position, velocity


def predict_jump_state(
    car: VehicleState,
    future_time: float,
    jump_elapsed: float = 0.0,
    double_jump: bool = False,
    jump_hold_time: float = DEFAULT_JUMP_HOLD,
    release_frames: int = DEFAULT_RELEASE_FRAMES,
    second_jump_frames: int = DEFAULT_SECOND_JUMP_FRAMES,
) -> JumpPrediction:
    """Predict future car state during a jump.

    Args:
        car: Current vehicle state.
        future_time: Seconds into the future from this snapshot.
        jump_elapsed: Seconds since the first jump began.
        double_jump: Whether to include a second jump impulse.
        jump_hold_time: Duration of the first jump hold.
        release_frames: Frames released between jumps.
        second_jump_frames: Duration of the second jump press.

    Horizontal movement uses constant current horizontal velocity.
    Air steering, boost acceleration, and collisions are not included.
    """
    if future_time < 0.0:
        return JumpPrediction(
            location=car.location,
            velocity=car.velocity,
            time=future_time,
            double_jump=double_jump,
            valid=False,
            reason="future_time cannot be negative",
        )

    if jump_elapsed < 0.0:
        return JumpPrediction(
            location=car.location,
            velocity=car.velocity,
            time=future_time,
            double_jump=double_jump,
            valid=False,
            reason="jump_elapsed cannot be negative",
        )

    if not car.on_ground and jump_elapsed <= 0.0:
        # An airborne snapshot has already received its first jump
        # impulse. Avoid applying it again.
        jump_elapsed = 0.000001

    x = car.location.x + car.velocity.x * future_time
    y = car.location.y + car.velocity.y * future_time

    z, vertical_velocity = _simulate_jump_vertical(
        initial_height=car.location.z,
        initial_velocity=car.velocity.z,
        future_time=future_time,
        jump_elapsed=jump_elapsed,
        double_jump=double_jump,
        jump_hold_time=jump_hold_time,
        release_frames=release_frames,
        second_jump_frames=second_jump_frames,
        started_on_ground=car.on_ground and jump_elapsed <= 0.0,
    )

    return JumpPrediction(
        location=Vec3(x, y, z),
        velocity=Vec3(
            car.velocity.x,
            car.velocity.y,
            vertical_velocity,
        ),
        time=future_time,
        double_jump=double_jump,
        valid=True,
        reason="jump trajectory predicted",
    )


def predict_jump_position(
    car: VehicleState,
    future_time: float,
    jump_elapsed: float = 0.0,
    double_jump: bool = False,
) -> Vec3:
    """Convenience function returning only the predicted position."""
    return predict_jump_state(
        car,
        future_time,
        jump_elapsed=jump_elapsed,
        double_jump=double_jump,
    ).location


def predict_jump_error(
    car: VehicleState,
    target: Vec3,
    future_time: float,
    jump_elapsed: float = 0.0,
    double_jump: bool = False,
) -> tuple[Vec3, float]:
    """Return target offset and distance at the predicted contact time."""
    prediction = predict_jump_state(
        car,
        future_time,
        jump_elapsed=jump_elapsed,
        double_jump=double_jump,
    )

    offset = target - prediction.location
    return offset, offset.magnitude
