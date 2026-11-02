
"""Grid aerial reachability and boost estimation.

Inspired by Party Cannon AerialShot.GetBoostEstimate / CanHit
and RLBotPack Molten's aerial prediction.

Estimates:
- Position after a single or double jump.
- Remaining aerial displacement.
- Required acceleration and boost.
- Turning delay.
- Whether the aerial is feasible with current boost.

This is a reachability estimator, not an aerial controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import inf, sqrt

from physics.jump import JumpConfig, simulate_jump
from physics.math3d import Vec3, clamp
from physics.vehicle import VehicleState


GRAVITY = 650.0
BOOST_ACCELERATION = 991.667
AIR_THROTTLE_ACCELERATION = 66.667
BOOST_CONSUMPTION = 33.333
MAX_CAR_SPEED = 2300.0


@dataclass
class AerialConfig:
    gravity: float = GRAVITY
    boost_acceleration: float = BOOST_ACCELERATION
    air_throttle_acceleration: float = AIR_THROTTLE_ACCELERATION
    boost_consumption: float = BOOST_CONSUMPTION
    max_speed: float = MAX_CAR_SPEED

    boost_safety_fraction: float = 0.90
    speed_safety_fraction: float = 0.90

    turn_time_factor: float = 0.60
    turn_time_scale: float = 9.0

    minimum_remaining_time: float = 0.08
    minimum_acceleration_time: float = 0.04

    jump_config: JumpConfig | None = None


@dataclass
class AerialReachability:
    reachable: bool
    double_jump: bool

    time_remaining: float
    turn_time: float
    acceleration_time: float

    predicted_position: Vec3
    predicted_velocity: Vec3

    target_position: Vec3
    remaining_displacement: Vec3

    required_acceleration: float
    estimated_boost: float
    estimated_final_speed: float

    reason: str = ""

    @property
    def boost_available(self) -> bool:
        return self.estimated_boost >= 0.0


def _jump_displacement(
    time_remaining: float,
    double_jump: bool,
    config: AerialConfig,
) -> tuple[float, float]:
    """Return jump displacement and velocity along the car's up axis."""

    trajectory = simulate_jump(
        double_jump=double_jump,
        config=config.jump_config,
    )

    if time_remaining <= 0:
        return 0.0, 0.0

    if time_remaining <= trajectory.total_time:
        sample_height = trajectory.height_at(time_remaining)
        sample_velocity = trajectory.velocity_at(time_remaining)
        return sample_height, sample_velocity

    # After the simulated jump lands, do not continue adding jump lift.
    # This is a conservative approximation for longer aerial windows.
    final = trajectory.samples[-1]
    return final.height, final.velocity


def _predict_car_state(
    car: VehicleState,
    time_remaining: float,
    double_jump: bool,
    config: AerialConfig,
) -> tuple[Vec3, Vec3]:
    """Predict car position and velocity before aerial acceleration."""

    if car.on_ground:
        # simulate_jump already integrates gravity. Do not add a second
        # world-gravity trajectory on top of the jump trajectory.
        jump_height, jump_velocity = _jump_displacement(
            time_remaining,
            double_jump,
            config,
        )
        up = car.orientation.up
        position = car.location + car.velocity * time_remaining + up * jump_height
        velocity = car.velocity + up * jump_velocity
    else:
        gravity = Vec3(0.0, 0.0, -config.gravity)
        position = (
            car.location
            + car.velocity * time_remaining
            + gravity * (0.5 * time_remaining * time_remaining)
        )
        velocity = car.velocity + gravity * time_remaining

    return position, velocity


def _turn_time(
    car: VehicleState,
    target_direction: Vec3,
    config: AerialConfig,
) -> float:
    """Estimate orientation delay using Party Cannon's angle-based model."""

    forward = car.orientation.forward.normalized()
    direction = target_direction.normalized()

    if forward.length() < 0.5 or direction.length() < 0.5:
        return 0.0

    angle = forward.angle_to(direction)

    if angle <= 0.05:
        return 0.0

    return (
        config.turn_time_factor
        * 2.0
        * sqrt(angle / config.turn_time_scale)
    )


def estimate_aerial_reachability(
    car: VehicleState,
    target: Vec3,
    time_remaining: float,
    *,
    double_jump: bool = False,
    config: AerialConfig | None = None,
) -> AerialReachability:
    """Estimate whether the car can reach a 3D target in the given time."""

    config = config or AerialConfig()
    target = Vec3.from_value(target)

    time_remaining = float(time_remaining)

    if car.demolished:
        return AerialReachability(
            False, double_jump, time_remaining, 0.0, 0.0,
            car.location, car.velocity, target, ZERO,
            inf, inf, inf, "car demolished",
        )

    if time_remaining <= config.minimum_remaining_time:
        return AerialReachability(
            False, double_jump, time_remaining, 0.0, 0.0,
            car.location, car.velocity, target,
            target - car.location, inf, inf, inf,
            "not enough time remaining",
        )

    predicted_position, predicted_velocity = _predict_car_state(
        car,
        time_remaining,
        double_jump,
        config,
    )

    displacement = target - predicted_position
    distance = displacement.length()

    if distance <= 50.0:
        return AerialReachability(
            True, double_jump, time_remaining, 0.0, 0.0,
            predicted_position, predicted_velocity, target,
            displacement, 0.0, 0.0, predicted_velocity.length(),
            "predicted position already within contact range",
        )

    direction = displacement.normalized()

    turn_time = _turn_time(car, direction, config)
    acceleration_time = time_remaining - turn_time

    if acceleration_time <= config.minimum_acceleration_time:
        return AerialReachability(
            False, double_jump, time_remaining, turn_time,
            acceleration_time, predicted_position, predicted_velocity,
            target, displacement, inf, inf, inf,
            "insufficient time after turning",
        )

    # Required acceleration for a constant-acceleration displacement.
    required_acceleration = (
        2.0 * distance / (acceleration_time * acceleration_time)
    )

    total_acceleration = (
        config.boost_acceleration
        + config.air_throttle_acceleration
    )

    ratio = required_acceleration / total_acceleration

    if ratio >= 1.0:
        return AerialReachability(
            False, double_jump, time_remaining, turn_time,
            acceleration_time, predicted_position, predicted_velocity,
            target, displacement, required_acceleration, inf, inf,
            "required acceleration exceeds boost and throttle",
        )

    # Approximate the boost interval using Party Cannon's square-root
    # acceleration model.
    boost_time = acceleration_time * (
        1.0 - sqrt(max(0.0, 1.0 - ratio))
    )

    estimated_boost = boost_time * config.boost_consumption

    final_velocity = (
        predicted_velocity
        + direction * total_acceleration * boost_time
    )

    final_speed = final_velocity.length()

    if estimated_boost > car.boost * config.boost_safety_fraction:
        return AerialReachability(
            False, double_jump, time_remaining, turn_time,
            acceleration_time, predicted_position, predicted_velocity,
            target, displacement, required_acceleration,
            estimated_boost, final_speed,
            "insufficient boost reserve",
        )

    if final_speed > config.max_speed * config.speed_safety_fraction:
        return AerialReachability(
            False, double_jump, time_remaining, turn_time,
            acceleration_time, predicted_position, predicted_velocity,
            target, displacement, required_acceleration,
            estimated_boost, final_speed,
            "estimated final speed too high",
        )

    return AerialReachability(
        True,
        double_jump,
        time_remaining,
        turn_time,
        acceleration_time,
        predicted_position,
        predicted_velocity,
        target,
        displacement,
        required_acceleration,
        estimated_boost,
        final_speed,
        "aerial reachable",
    )


def compare_aerial_launches(
    car: VehicleState,
    target: Vec3,
    time_remaining: float,
    config: AerialConfig | None = None,
) -> tuple[AerialReachability, AerialReachability]:
    """Compare single-jump and double-jump aerial launches."""

    single = estimate_aerial_reachability(
        car,
        target,
        time_remaining,
        double_jump=False,
        config=config,
    )

    double = estimate_aerial_reachability(
        car,
        target,
        time_remaining,
        double_jump=True,
        config=config,
    )

    return single, double


def best_aerial_launch(
    car: VehicleState,
    target: Vec3,
    time_remaining: float,
    config: AerialConfig | None = None,
) -> AerialReachability | None:
    """Choose the feasible launch with the lower estimated boost cost."""

    single, double = compare_aerial_launches(
        car,
        target,
        time_remaining,
        config,
    )

    options = [
        result
        for result in (single, double)
        if result.reachable
    ]

    if not options:
        return None

    return min(
        options,
        key=lambda result: result.estimated_boost,
    )
