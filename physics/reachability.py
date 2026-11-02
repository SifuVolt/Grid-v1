
"""Vehicle reachability and arrival-time estimation for Grid.

Estimates how quickly a vehicle can reach a world-space target.

The model includes:
- Speed-dependent turning geometry.
- Turn-time penalties.
- Throttle acceleration.
- Boost acceleration and consumption.
- Reverse-facing penalties.
- Braking and target-speed constraints.

This is an interception planning model, not a full vehicle simulator.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import atan2, copysign, inf, pi

from .math3d import Vec3, clamp
from .vehicle import (
    VehicleState,
    MAX_FORWARD_SPEED,
    MAX_REVERSE_SPEED,
    MAX_THROTTLE_SPEED,
    BOOST_ACCELERATION,
    BOOST_CONSUMPTION,
    THROTTLE_ACCELERATION,
    BRAKE_ACCELERATION,
)
from .steering import (
    signed_angle_to,
    turning_radius,
    estimate_turn_time,
)


# ---------------------------------------------------------------------------
# Reachability constants
# ---------------------------------------------------------------------------

MIN_ESTIMATE_SPEED = 400.0
LOW_SPEED_ACCELERATION = 1600.0

BOOST_SPEED_CAP = MAX_FORWARD_SPEED
THROTTLE_SPEED_CAP = MAX_THROTTLE_SPEED

TURN_TIME_MULTIPLIER = 1.05
TURN_ALIGNMENT_THRESHOLD = 0.5

REVERSE_TURN_PENALTY = 0.45
AERIAL_APPROACH_PENALTY = 0.12

DEFAULT_TARGET_RADIUS = 120.0


@dataclass(frozen=True, slots=True)
class Reachability:
    """Result of a vehicle-to-target reachability estimate."""

    reachable: bool
    time: float
    distance: float
    turn_time: float
    travel_time: float
    final_speed: float
    boost_used: float
    target: Vec3

    @property
    def arrival_time(self) -> float:
        return self.time

    @property
    def eta(self) -> float:
        return self.time


@dataclass(frozen=True, slots=True)
class AccelerationResult:
    """Distance and speed after an acceleration interval."""

    distance: float
    final_speed: float
    time: float
    boost_used: float


def _positive_distance(value: float) -> float:
    return max(0.0, value)


def _time_to_accelerate(
    initial_speed: float,
    target_speed: float,
    acceleration: float,
) -> float:
    if target_speed <= initial_speed:
        return 0.0

    if acceleration <= 1e-6:
        return inf

    return (target_speed - initial_speed) / acceleration


def simulate_acceleration(
    initial_speed: float,
    duration: float,
    boost_available: float = 0.0,
    use_boost: bool = True,
    maximum_speed: float = MAX_FORWARD_SPEED,
) -> AccelerationResult:
    """Estimate travel distance and final speed over a time interval.

    The model uses throttle to accelerate toward the throttle speed cap,
    then boost to accelerate beyond it. If boost is unavailable, it
    continues at the throttle speed cap.

    Boost is consumed at the standard approximate rate.
    """
    duration = max(0.0, duration)
    speed = clamp(initial_speed, 0.0, maximum_speed)
    remaining = duration
    distance = 0.0
    boost_used = 0.0

    boost_available = clamp(boost_available, 0.0, 100.0)

    # First accelerate with throttle to the throttle speed cap.
    throttle_target = min(
        THROTTLE_SPEED_CAP,
        maximum_speed,
    )

    if speed < throttle_target and remaining > 0.0:
        time_needed = _time_to_accelerate(
            speed,
            throttle_target,
            THROTTLE_ACCELERATION,
        )

        step_time = min(remaining, time_needed)

        distance += (
            speed * step_time
            + 0.5 * THROTTLE_ACCELERATION * step_time**2
        )

        speed = min(
            throttle_target,
            speed + THROTTLE_ACCELERATION * step_time,
        )

        remaining -= step_time

    # Boost is useful above the throttle speed cap.
    if (
        use_boost
        and boost_available > 0.0
        and remaining > 0.0
        and speed < maximum_speed
    ):
        boost_time_available = boost_available / BOOST_CONSUMPTION
        boost_time = min(remaining, boost_time_available)

        time_to_speed_cap = _time_to_accelerate(
            speed,
            maximum_speed,
            BOOST_ACCELERATION,
        )

        boost_time = min(boost_time, time_to_speed_cap)

        distance += (
            speed * boost_time
            + 0.5 * BOOST_ACCELERATION * boost_time**2
        )

        speed = min(
            maximum_speed,
            speed + BOOST_ACCELERATION * boost_time,
        )

        boost_used = boost_time * BOOST_CONSUMPTION
        remaining -= boost_time

    # Coast at the achieved speed for the rest of the interval.
    if remaining > 0.0:
        distance += speed * remaining

    return AccelerationResult(
        distance=distance,
        final_speed=speed,
        time=duration,
        boost_used=boost_used,
    )


def simulate_braking(
    initial_speed: float,
    duration: float,
    deceleration: float = BRAKE_ACCELERATION,
) -> AccelerationResult:
    """Estimate distance and final speed while braking."""
    duration = max(0.0, duration)
    speed = max(0.0, initial_speed)
    deceleration = max(0.0, deceleration)

    if duration <= 0.0 or deceleration <= 1e-6:
        return AccelerationResult(
            distance=speed * duration,
            final_speed=speed,
            time=duration,
            boost_used=0.0,
        )

    time_to_stop = speed / deceleration
    braking_time = min(duration, time_to_stop)

    distance = (
        speed * braking_time
        - 0.5 * deceleration * braking_time**2
    )

    final_speed = max(
        0.0,
        speed - deceleration * braking_time,
    )

    remaining = duration - braking_time
    distance += final_speed * remaining

    return AccelerationResult(
        distance=distance,
        final_speed=final_speed,
        time=duration,
        boost_used=0.0,
    )


def _travel_time_for_distance(
    distance: float,
    initial_speed: float,
    boost: float,
    use_boost: bool,
    maximum_speed: float = MAX_FORWARD_SPEED,
) -> tuple[float, float, float]:
    """Return (travel time, final speed, boost used)."""
    distance = max(0.0, distance)

    if distance <= 1.0:
        return 0.0, max(0.0, initial_speed), 0.0

    speed = clamp(initial_speed, 0.0, maximum_speed)
    remaining_distance = distance
    elapsed = 0.0
    boost_used = 0.0

    # Integrate in short intervals to account for speed caps and boost.
    step = 0.05
    maximum_time = 15.0

    while remaining_distance > 0.0 and elapsed < maximum_time:
        step_time = step

        throttle_target = min(
            THROTTLE_SPEED_CAP,
            maximum_speed,
        )

        if speed < throttle_target:
            acceleration = THROTTLE_ACCELERATION
            next_speed = min(
                throttle_target,
                speed + acceleration * step_time,
            )
            average_speed = (speed + next_speed) * 0.5
            step_distance = average_speed * step_time
            speed = next_speed

        elif (
            use_boost
            and boost_used < boost
            and speed < maximum_speed
        ):
            available_boost_time = (
                boost - boost_used
            ) / BOOST_CONSUMPTION

            step_time = min(step_time, available_boost_time)

            acceleration = BOOST_ACCELERATION
            next_speed = min(
                maximum_speed,
                speed + acceleration * step_time,
            )

            average_speed = (speed + next_speed) * 0.5
            step_distance = average_speed * step_time
            speed = next_speed

            boost_used += step_time * BOOST_CONSUMPTION

        else:
            step_distance = speed * step_time

        if step_distance <= 1e-6:
            return inf, speed, boost_used

        remaining_distance -= step_distance
        elapsed += step_time

    if remaining_distance > 0.0:
        return inf, speed, boost_used

    return elapsed, speed, boost_used


def estimate_reachability(
    vehicle: VehicleState,
    target: Vec3,
    use_boost: bool = True,
    target_speed: float = MAX_FORWARD_SPEED,
    target_radius: float = DEFAULT_TARGET_RADIUS,
    include_turn: bool = True,
) -> Reachability:
    """Estimate time for a vehicle to reach a target.

    The result is a relative time in seconds.

    target_radius reduces the distance needed to count as reaching the
    target, useful when estimating ball contact rather than exact point
    overlap.
    """
    if vehicle.demolished:
        return Reachability(
            reachable=False,
            time=inf,
            distance=inf,
            turn_time=0.0,
            travel_time=inf,
            final_speed=0.0,
            boost_used=0.0,
            target=target,
        )

    distance = max(
        0.0,
        vehicle.location.flat_distance(target) - target_radius,
    )

    if distance <= 1.0:
        return Reachability(
            reachable=True,
            time=0.0,
            distance=distance,
            turn_time=0.0,
            travel_time=0.0,
            final_speed=vehicle.flat_speed,
            boost_used=0.0,
            target=target,
        )

    angle = signed_angle_to(vehicle, target)

    turn_time = 0.0
    if include_turn:
        turn_time = estimate_turn_time(vehicle, angle)
        turn_time *= TURN_TIME_MULTIPLIER

    # When the target is substantially behind the car, account for the
    # time needed to turn around. Do not assume instant reverse travel.
    if abs(angle) > 2.2:
        turn_time += REVERSE_TURN_PENALTY

    # Travel distance after the turn. This is intentionally conservative:
    # the vehicle is assumed to lose some speed while turning.
    starting_speed = vehicle.flat_speed

    if turn_time > 0.0:
        estimated_turn_distance = min(
            distance * 0.25,
            turning_radius(starting_speed) * abs(angle),
        )
        distance = max(0.0, distance - estimated_turn_distance)

        # Turning and acceleration are not perfectly independent.
        starting_speed *= 0.85

    available_boost = vehicle.boost if use_boost else 0.0

    travel_time, final_speed, boost_used = _travel_time_for_distance(
        distance=distance,
        initial_speed=starting_speed,
        boost=available_boost,
        use_boost=use_boost,
        maximum_speed=min(
            MAX_FORWARD_SPEED,
            max(1.0, target_speed),
        ),
    )

    total_time = turn_time + travel_time

    return Reachability(
        reachable=total_time < inf,
        time=total_time,
        distance=distance,
        turn_time=turn_time,
        travel_time=travel_time,
        final_speed=final_speed,
        boost_used=boost_used,
        target=target,
    )


def estimate_arrival_time(
    vehicle: VehicleState,
    target: Vec3,
    use_boost: bool = True,
    target_radius: float = DEFAULT_TARGET_RADIUS,
) -> float:
    """Simple ETA interface for teamplay and interception code."""
    return estimate_reachability(
        vehicle,
        target,
        use_boost=use_boost,
        target_radius=target_radius,
    ).time


def can_reach_before(
    vehicle: VehicleState,
    target: Vec3,
    deadline: float,
    use_boost: bool = True,
    target_radius: float = DEFAULT_TARGET_RADIUS,
) -> bool:
    """Whether the vehicle is estimated to reach a target before a deadline."""
    if deadline < 0.0:
        return False

    result = estimate_reachability(
        vehicle,
        target,
        use_boost=use_boost,
        target_radius=target_radius,
    )

    return result.reachable and result.time <= deadline


def estimate_arrival_without_boost(
    vehicle: VehicleState,
    target: Vec3,
    target_radius: float = DEFAULT_TARGET_RADIUS,
) -> float:
    return estimate_arrival_time(
        vehicle,
        target,
        use_boost=False,
        target_radius=target_radius,
    )


def compare_arrival_times(
    first: VehicleState,
    second: VehicleState,
    target: Vec3,
    use_boost: bool = True,
) -> int:
    """Compare two cars' arrival estimates.

    Returns:
        -1 if first is earlier
         0 if effectively tied
         1 if second is earlier
    """
    first_time = estimate_arrival_time(
        first,
        target,
        use_boost=use_boost,
    )
    second_time = estimate_arrival_time(
        second,
        target,
        use_boost=use_boost,
    )

    if abs(first_time - second_time) < 0.05:
        return 0

    return -1 if first_time < second_time else 1


def estimate_intercept_margin(
    vehicle: VehicleState,
    target: Vec3,
    available_time: float,
    use_boost: bool = True,
) -> float:
    """Positive margin means the car is expected to arrive in time."""
    arrival = estimate_arrival_time(
        vehicle,
        target,
        use_boost=use_boost,
    )

    return available_time - arrival
