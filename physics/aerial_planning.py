
"""Aerial planning and jump selection.

Based on the aerial feasibility approach in Party Cannon's AerialShot:
- Estimate the car's position after a single or double jump.
- Estimate how long turning toward the target takes.
- Calculate the acceleration and boost needed after turning.
- Reject plans that exceed time, boost, or speed limits.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

from physics.math3d import Vec3, clamp
from physics.vehicle import VehicleState
from physics.jump_prediction import predict_jump_state


BOOST_ACCELERATION = 991.667
AIR_THROTTLE_ACCELERATION = 66.667
BOOST_CONSUMPTION = 33.333

MAX_CAR_SPEED = 2300.0
SAFE_MAX_SPEED = MAX_CAR_SPEED * 0.90
MIN_TIME = 0.08
MIN_BOOST_RESERVE = 0.10


@dataclass
class AerialOption:
    reachable: bool
    double_jump: bool
    boost_required: float
    required_acceleration: float
    turn_time: float
    time_after_turn: float
    distance_error: float
    speed_estimate: float
    reason: str


@dataclass
class AerialPlan:
    valid: bool
    double_jump: bool
    boost_required: float
    single_jump: AerialOption
    double_jump_option: AerialOption
    selected: AerialOption | None
    reason: str


def _unit(vector: Vec3) -> Vec3:
    if vector.magnitude < 0.001:
        return Vec3(0.0, 0.0, 0.0)
    return vector.normalized()


def _estimate_option(
    car: VehicleState,
    target: Vec3,
    time_remaining: float,
    double_jump: bool,
    jump_elapsed: float,
) -> AerialOption:
    if time_remaining <= MIN_TIME:
        return AerialOption(
            False, double_jump, float("inf"), float("inf"),
            0.0, 0.0, 0.0, car.velocity.magnitude,
            "not enough time remaining",
        )

    prediction = predict_jump_state(
        car,
        time_remaining,
        jump_elapsed=jump_elapsed,
        double_jump=double_jump,
    )

    offset = target - prediction.location
    distance = offset.magnitude
    direction = _unit(offset)

    if distance <= 50.0:
        return AerialOption(
            True, double_jump, 0.0, 0.0,
            0.0, time_remaining, distance,
            prediction.velocity.magnitude,
            "already within contact tolerance",
        )

    forward = _unit(car.orientation.forward)
    angle = 0.0

    if direction.magnitude > 0.0 and forward.magnitude > 0.0:
        alignment = clamp(direction.dot(forward), -1.0, 1.0)
        # acos without requiring another dependency.
        from math import acos
        angle = acos(alignment)

    # Party Cannon's approximate aerial turn-time model.
    turn_time = 0.6 * (2.0 * sqrt(max(0.0, angle) / 9.0))
    turn_time = clamp(turn_time, 0.0, time_remaining * 0.65)

    # A target nearly behind the car needs more time to orient.
    turn_fraction = clamp(
        1.0 - 0.4 / max(angle, 0.0001),
        0.0,
        1.0,
    )
    turn_delay = turn_time * turn_fraction

    time_after_turn = time_remaining - turn_delay

    if time_after_turn <= MIN_TIME:
        return AerialOption(
            False, double_jump, float("inf"), float("inf"),
            turn_time, time_after_turn, distance,
            prediction.velocity.magnitude,
            "turning leaves insufficient flight time",
        )

    required_acceleration = (
        2.0 * distance / (time_after_turn * time_after_turn)
    )

    total_acceleration = (
        BOOST_ACCELERATION + AIR_THROTTLE_ACCELERATION
    )

    ratio = required_acceleration / total_acceleration

    # Estimate when acceleration must begin and how long boost is needed.
    # This follows Party Cannon's tau2 calculation.
    ratio_for_time = clamp(ratio, 0.0, 1.0)
    acceleration_start = time_remaining - (
        time_after_turn * sqrt(max(0.0, 1.0 - ratio_for_time))
    )

    boost_time = max(0.0, acceleration_start - turn_delay)
    boost_required = boost_time * BOOST_CONSUMPTION

    # Estimate final velocity after the required boost interval.
    velocity_estimate = (
        prediction.velocity + direction * total_acceleration * boost_time
    )
    speed_estimate = velocity_estimate.magnitude

    enough_boost = (
        boost_required <= car.boost * (1.0 - MIN_BOOST_RESERVE)
    )
    enough_time = ratio < 0.95
    enough_speed = speed_estimate < SAFE_MAX_SPEED

    reachable = enough_boost and enough_time and enough_speed

    if not enough_time:
        reason = "required acceleration is too high"
    elif not enough_boost:
        reason = "not enough boost after keeping a reserve"
    elif not enough_speed:
        reason = "estimated speed exceeds safe maximum"
    else:
        reason = "jump option is reachable"

    return AerialOption(
        reachable=reachable,
        double_jump=double_jump,
        boost_required=boost_required,
        required_acceleration=required_acceleration,
        turn_time=turn_time,
        time_after_turn=time_after_turn,
        distance_error=distance,
        speed_estimate=speed_estimate,
        reason=reason,
    )


def plan_aerial(
    car: VehicleState,
    target: Vec3,
    time_remaining: float,
    jump_elapsed: float = 0.0,
) -> AerialPlan:
    """Choose the least expensive viable single/double-jump aerial."""

    single = _estimate_option(
        car, target, time_remaining, False, jump_elapsed
    )
    double = _estimate_option(
        car, target, time_remaining, True, jump_elapsed
    )

    candidates = [
        option for option in (single, double)
        if option.reachable
    ]

    if not candidates:
        return AerialPlan(
            valid=False,
            double_jump=False,
            boost_required=float("inf"),
            single_jump=single,
            double_jump_option=double,
            selected=None,
            reason="neither jump option is reachable",
        )

    # Prefer the option that spends less boost. If effectively tied,
    # prefer the single jump and preserve the second jump for flexibility.
    candidates.sort(
        key=lambda option: (
            option.boost_required,
            option.double_jump,
        )
    )

    selected = candidates[0]

    return AerialPlan(
        valid=True,
        double_jump=selected.double_jump,
        boost_required=selected.boost_required,
        single_jump=single,
        double_jump_option=double,
        selected=selected,
        reason=(
            "selected double jump"
            if selected.double_jump
            else "selected single jump"
        ),
    )
