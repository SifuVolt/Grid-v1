
"""Aerial reachability and acceleration planning.

Inspired by Party Cannon's AerialShot and Noob Black's aerial
acceleration calculations.

This module estimates whether the car can correct its predicted
position error before an aerial contact, and calculates sensible
throttle and boost requirements.
"""

from __future__ import annotations

from dataclasses import dataclass

from physics.math3d import Vec3, clamp
from physics.vehicle import VehicleState
from physics.jump_prediction import predict_jump_state


MAX_CAR_SPEED = 2300.0
BOOST_ACCELERATION = 991.667
AIR_THROTTLE_ACCELERATION = 66.667
BOOST_CONSUMPTION = 33.333

MIN_CONTROL_TIME = 1.0 / 120.0
BOOST_LOOKAHEAD = 13.0 / 120.0


@dataclass
class AerialReachability:
    reachable: bool
    required_acceleration: float
    available_acceleration: float
    acceleration_ratio: float
    distance_error: float
    time_remaining: float
    boost_required: float
    boost_available: float
    direction_alignment: float
    reason: str


@dataclass
class AerialControls:
    throttle: float = 0.0
    boost: bool = False
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0
    required_acceleration: float = 0.0
    boost_required: float = 0.0
    reachable: bool = False
    reason: str = ""


def _dot(a: Vec3, b: Vec3) -> float:
    return a.dot(b)


def _unit(vector: Vec3) -> Vec3:
    if vector.magnitude < 0.001:
        return Vec3(0.0, 0.0, 0.0)

    return vector.normalized()


def _future_position(
    car: VehicleState,
    time_remaining: float,
    jump_elapsed: float,
    double_jump: bool,
) -> Vec3:
    prediction = predict_jump_state(
        car,
        time_remaining,
        jump_elapsed=jump_elapsed,
        double_jump=double_jump,
    )

    return prediction.location


def evaluate_aerial_reachability(
    car: VehicleState,
    target: Vec3,
    time_remaining: float,
    jump_elapsed: float = 0.0,
    double_jump: bool = False,
    use_boost: bool = True,
    safety_factor: float = 0.85,
) -> AerialReachability:
    """Estimate whether the car can reach an aerial contact target.

    Uses the jump predictor for vertical motion and constant horizontal
    velocity for the initial position estimate.

    The remaining positional error is converted into a required
    acceleration using the same 2*d/t² relationship used by
    Party Cannon.

    This is a planning estimate, not a full physics simulation.
    """

    if time_remaining <= 0.0:
        return AerialReachability(
            False, 0.0, 0.0, float("inf"),
            0.0, time_remaining, 0.0, car.boost,
            0.0, "no time remaining",
        )

    predicted = _future_position(
        car,
        time_remaining,
        jump_elapsed,
        double_jump,
    )

    offset = target - predicted
    distance = offset.magnitude

    required_acceleration = (
        2.0 * distance / (time_remaining * time_remaining)
    )

    direction = _unit(offset)
    forward = _unit(car.orientation.forward)

    alignment = clamp(_dot(direction, forward), -1.0, 1.0)

    available_acceleration = AIR_THROTTLE_ACCELERATION

    if use_boost and car.boost > 0.0:
        available_acceleration += BOOST_ACCELERATION

    effective_acceleration = available_acceleration * safety_factor

    ratio = (
        required_acceleration / effective_acceleration
        if effective_acceleration > 0.0
        else float("inf")
    )

    # Estimate boost duration using the acceleration needed beyond
    # ordinary air throttle. This is intentionally conservative.
    boost_acceleration_needed = max(
        0.0,
        required_acceleration - AIR_THROTTLE_ACCELERATION,
    )

    boost_fraction = clamp(
        boost_acceleration_needed / BOOST_ACCELERATION,
        0.0,
        1.0,
    )

    boost_required = (
        time_remaining * boost_fraction * BOOST_CONSUMPTION
    )

    reachable = (
        ratio <= 1.0
        and (
            boost_required <= car.boost
            or not use_boost
            or alignment < 0.75
        )
    )

    if distance < 50.0:
        reachable = True
        reason = "already within contact tolerance"
    elif ratio > 1.0:
        reason = "required acceleration exceeds available acceleration"
    elif boost_required > car.boost and alignment >= 0.75:
        reason = "insufficient boost for estimated correction"
    else:
        reason = "aerial target appears reachable"

    return AerialReachability(
        reachable=reachable,
        required_acceleration=required_acceleration,
        available_acceleration=effective_acceleration,
        acceleration_ratio=ratio,
        distance_error=distance,
        time_remaining=time_remaining,
        boost_required=boost_required,
        boost_available=car.boost,
        direction_alignment=alignment,
        reason=reason,
    )


def calculate_aerial_controls(
    car: VehicleState,
    target: Vec3,
    time_remaining: float,
    jump_elapsed: float = 0.0,
    double_jump: bool = False,
    boost_alignment: float = 0.75,
    throttle_alignment: float = 0.50,
) -> AerialControls:
    """Calculate basic boost and throttle decisions for an aerial.

    Orientation control remains the responsibility of Grid's
    air-control module. This function decides whether forward
    acceleration is useful and whether boost should be spent.
    """

    if time_remaining <= 0.0:
        return AerialControls(
            reachable=False,
            reason="contact time has passed",
        )

    predicted = _future_position(
        car,
        time_remaining,
        jump_elapsed,
        double_jump,
    )

    offset = target - predicted
    distance = offset.magnitude

    if distance < 50.0:
        return AerialControls(
            reachable=True,
            reason="within contact tolerance",
        )

    direction = _unit(offset)
    forward = _unit(car.orientation.forward)
    alignment = clamp(_dot(direction, forward), -1.0, 1.0)

    required_acceleration = (
        2.0 * distance / (time_remaining * time_remaining)
    )

    # Estimate how much boost is required to make the correction.
    boost_acceleration_needed = max(
        0.0,
        required_acceleration - AIR_THROTTLE_ACCELERATION,
    )

    boost_fraction = clamp(
        boost_acceleration_needed / BOOST_ACCELERATION,
        0.0,
        1.0,
    )

    boost_required = (
        time_remaining * boost_fraction * BOOST_CONSUMPTION
    )

    # Party Cannon's boost logic requires a forward-facing target
    # and enough projected acceleration to justify spending boost.
    boost_threshold = (
        BOOST_ACCELERATION + AIR_THROTTLE_ACCELERATION
    ) * max(time_remaining, BOOST_LOOKAHEAD)

    forward_error = _dot(offset, forward)

    boost = (
        car.boost > 0.0
        and alignment > boost_alignment
        and forward_error / time_remaining >= boost_threshold
        and boost_required <= car.boost * 0.9
    )

    throttle = (
        1.0
        if alignment > throttle_alignment
        else 0.0
    )

    reachable = (
        required_acceleration
        <= AIR_THROTTLE_ACCELERATION
        + (BOOST_ACCELERATION if car.boost > 0.0 else 0.0)
    )

    return AerialControls(
        throttle=throttle,
        boost=boost,
        required_acceleration=required_acceleration,
        boost_required=boost_required,
        reachable=reachable,
        reason=(
            "accelerating toward predicted contact"
            if reachable
            else "correction exceeds available acceleration"
        ),
    )
