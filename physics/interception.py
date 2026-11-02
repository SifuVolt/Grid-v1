"""Grid interception and arrival-time estimation.

The estimator follows the broad approach used by Party Cannon's
InterceptCalculator: estimate turn time, integrate acceleration/boost,
then compare arrival time with sampled ball prediction.

This module is deliberately self-contained and keeps the public API used
by Grid's existing mechanics.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import sqrt

from physics.field import BALL_RADIUS, FIELD_HALF_LENGTH, FIELD_HALF_WIDTH
from physics.math3d import Vec3, clamp
from physics.vehicle import VehicleState


MAX_SPEED = 2300.0
BOOST_ACCEL = 991.666
BOOST_CONSUMPTION = 33.333
THROTTLE_ACCEL = 1600.0
MAX_THROTTLE_SPEED = 1410.0
MAX_REVERSE_SPEED = 1400.0
MAX_TURNING_SPEED = 1800.0

# Speed gain from boost, measured from an initial speed of zero.
# These are increments, not absolute final speeds.
BOOST_SPEEDS = (0.0, 500.0, 1000.0, 1410.0, 1800.0, 2000.0, 2300.0)
BOOST_TIMES = (0.0, 0.5, 1.0, 1.4, 1.8, 2.1, 2.4)


@dataclass
class InterceptCandidate:
    time: float
    position: Vec3
    velocity: Vec3
    arrival_time: float
    margin: float
    score: float
    aerial: bool = False
    reachable: bool = True


@dataclass
class InterceptResult:
    valid: bool = False
    candidate: InterceptCandidate | None = None
    candidates: list[InterceptCandidate] | None = None
    reason: str = "no reachable intercept"

    @property
    def time(self) -> float:
        return self.candidate.time if self.candidate else 0.0

    @property
    def position(self) -> Vec3 | None:
        return self.candidate.position if self.candidate else None

    @property
    def arrival_time(self) -> float:
        return self.candidate.arrival_time if self.candidate else float("inf")

    @property
    def margin(self) -> float:
        return self.candidate.margin if self.candidate else float("-inf")


@dataclass
class InterceptAdvantage:
    our_best_time: float = float("inf")
    opponent_best_time: float = float("inf")
    our_best_car: VehicleState | None = None
    opponent_best_car: VehicleState | None = None
    our_best_position: Vec3 | None = None
    opponent_best_position: Vec3 | None = None

    @property
    def opponents_are_faster(self) -> bool:
        return self.opponent_best_time < self.our_best_time - 0.1

    @property
    def we_are_faster(self) -> bool:
        return self.our_best_time < self.opponent_best_time - 0.1


def _interpolate_speed_gain(
    times: tuple[float, ...],
    gains: tuple[float, ...],
    target_time: float,
) -> float:
    """Interpolate the boost speed-gain curve, clamped to its endpoints."""
    if not times or not gains:
        return 0.0
    if target_time <= times[0]:
        return gains[0]
    if target_time >= times[-1]:
        return gains[-1]

    for i in range(len(times) - 1):
        if target_time <= times[i + 1]:
            span = max(times[i + 1] - times[i], 1e-6)
            fraction = (target_time - times[i]) / span
            return gains[i] + (gains[i + 1] - gains[i]) * fraction

    return gains[-1]


def _speed_after_boost(base_speed: float, boost_time: float) -> float:
    """Approximate forward speed after boosting for ``boost_time`` seconds."""
    gain = _interpolate_speed_gain(BOOST_TIMES, BOOST_SPEEDS, boost_time)
    return clamp(base_speed + gain, 0.0, MAX_SPEED)


def _boost_distance(base_speed: float, duration: float) -> float:
    """Integrate the capped boost-speed curve with trapezoidal samples."""
    if duration <= 0.0:
        return 0.0

    # Integrate at no more than 50 ms spacing. Include curve breakpoints so
    # the piecewise-linear speed model is not sampled across sharp changes.
    points = {0.0, duration}
    points.update(t for t in BOOST_TIMES if 0.0 < t < duration)
    count = max(1, int(duration / 0.05))
    points.update(duration * i / count for i in range(1, count))

    ordered = sorted(points)
    distance = 0.0
    for left, right in zip(ordered, ordered[1:]):
        v0 = _speed_after_boost(base_speed, left)
        v1 = _speed_after_boost(base_speed, right)
        distance += (v0 + v1) * 0.5 * (right - left)
    return distance


def _boost_time_for_distance(base_speed: float, distance: float, max_time: float) -> float:
    """Find the earliest boost time that covers a given distance."""
    low, high = 0.0, max_time
    for _ in range(24):
        middle = (low + high) * 0.5
        if _boost_distance(base_speed, middle) >= distance:
            high = middle
        else:
            low = middle
    return high


def _solve_time_for_distance(
    initial_speed: float,
    acceleration: float,
    distance: float,
) -> float:
    if distance <= 0.0:
        return 0.0
    if acceleration <= 1e-6:
        return distance / max(initial_speed, 400.0)

    discriminant = initial_speed * initial_speed + 2.0 * acceleration * distance
    if discriminant <= 0.0:
        return distance / max(initial_speed, 400.0)

    return max(0.0, (-initial_speed + sqrt(discriminant)) / acceleration)


def _turning_radius(speed: float) -> float:
    """Party-Cannon-style speed-dependent turn-radius approximation."""
    speed = clamp(speed, 0.0, MAX_SPEED)
    curvature = 0.0069 - 6.19e-6 * speed + 1.1e-8 * speed * speed
    return 1.0 / max(curvature, 0.001)


def estimate_time_directional(
    car: VehicleState,
    target: Vec3,
    direction: int = 1,
    *,
    use_boost: bool = True,
) -> float:
    """Estimate travel time in forward or reverse.

    The estimate includes a turn-time penalty, acceleration and a small
    safety factor. Reverse travel never uses boost.
    """
    direction = 1 if direction >= 0 else -1
    displacement = target - car.location
    distance = displacement.flat_magnitude

    if distance < 100.0:
        return 0.0

    forward = car.orientation.forward.flatten().normalized() * direction
    target_direction = displacement.flatten().normalized()
    angle = forward.angle_to(target_direction)

    signed_speed = car.velocity.dot(car.orientation.forward)
    speed = max(signed_speed * direction, 0.0)
    speed_cap = MAX_SPEED if direction > 0 else MAX_REVERSE_SPEED
    speed = min(speed, speed_cap)

    turn_time = angle * _turning_radius(max(speed, 500.0)) / MAX_TURNING_SPEED
    if angle < 0.5:
        turn_time = 0.0

    total_time = turn_time
    distance_left = max(0.0, distance - 200.0)
    if distance_left <= 0.0:
        return total_time

    if use_boost and direction > 0 and car.boost > 0.0 and speed < MAX_SPEED:
        boost_time = min(car.boost / BOOST_CONSUMPTION, BOOST_TIMES[-1])
        boost_distance = _boost_distance(speed, boost_time)

        if boost_distance >= distance_left:
            exact_time = _boost_time_for_distance(speed, distance_left, boost_time)
            return (total_time + exact_time) * 1.05

        total_time += boost_time
        distance_left -= boost_distance
        speed = _speed_after_boost(speed, boost_time)

    acceleration = THROTTLE_ACCEL
    if direction > 0:
        target_speed = MAX_THROTTLE_SPEED
    else:
        target_speed = MAX_REVERSE_SPEED

    if speed < target_speed and distance_left > 0.0:
        accel_time = (target_speed - speed) / acceleration
        accel_distance = speed * accel_time + 0.5 * acceleration * accel_time * accel_time

        if accel_distance >= distance_left:
            return (total_time + _solve_time_for_distance(speed, acceleration, distance_left)) * 1.05

        total_time += accel_time
        distance_left -= accel_distance
        speed = target_speed

    if distance_left > 0.0:
        total_time += distance_left / max(speed, 800.0)

    return total_time * 1.05


def estimate_arrival_time(
    car: VehicleState,
    target: Vec3,
    *,
    allow_backwards: bool = True,
    use_boost: bool = True,
) -> float:
    """Return the quickest sensible forward/reverse arrival estimate."""
    forward_eta = estimate_time_directional(car, target, 1, use_boost=use_boost)
    if not allow_backwards:
        return forward_eta

    reverse_speed = max(-car.velocity.dot(car.orientation.forward), 0.0)
    distance = (target - car.location).flat_magnitude

    # Only consider reverse when already moving backwards or when the target
    # is close enough that a reverse turn can genuinely be competitive.
    if reverse_speed > 500.0 or (300.0 <= distance <= 3000.0):
        reverse_eta = estimate_time_directional(car, target, -1, use_boost=False) + 0.5
        return min(forward_eta, reverse_eta)

    return forward_eta


def sample_trajectory(
    trajectory,
    *,
    step: float = 0.1,
    maximum_time: float = 3.0,
    minimum_height: float = BALL_RADIUS,
) -> list[tuple[float, Vec3, Vec3]]:
    """Sample predicted ball states at regular time intervals."""
    if trajectory is None:
        return []

    duration = min(float(getattr(trajectory, "duration", maximum_time)), maximum_time)
    if duration <= 0.0:
        return []

    samples: list[tuple[float, Vec3, Vec3]] = []
    sample_step = max(0.02, step)
    t = 0.0

    while t <= duration + 1e-6:
        position = trajectory.position_at(t)
        velocity = trajectory.velocity_at(t)
        if position is not None and velocity is not None:
            position = Vec3.from_value(position)
            velocity = Vec3.from_value(velocity)
            position = Vec3(
                clamp(position.x, -FIELD_HALF_WIDTH, FIELD_HALF_WIDTH),
                clamp(position.y, -FIELD_HALF_LENGTH, FIELD_HALF_LENGTH),
                max(minimum_height, position.z),
            )
            samples.append((t, position, velocity))
        t += sample_step

    return samples


def _contact_target(position: Vec3, car: VehicleState) -> Vec3:
    """Estimate a car-centre contact point behind the ball."""
    direction = (position - car.location).flatten()
    if direction.flat_magnitude < 1e-5:
        direction = car.orientation.forward.flatten()
    direction = direction.normalized()
    return Vec3(position.x - direction.x * 120.0,
                position.y - direction.y * 120.0,
                max(17.0, position.z - 35.0))


def find_intercepts(
    car: VehicleState,
    trajectory,
    *,
    step: float = 0.1,
    maximum_time: float = 3.0,
    minimum_margin: float = -0.05,
    maximum_height: float = 1200.0,
    allow_aerial: bool = False,
) -> list[InterceptCandidate]:
    """Find reachable predicted ball contacts and rank useful windows."""
    candidates: list[InterceptCandidate] = []
    for t, position, velocity in sample_trajectory(
        trajectory, step=step, maximum_time=maximum_time
    ):
        if position.z > maximum_height and not allow_aerial:
            continue

        target = _contact_target(position, car)
        eta = estimate_arrival_time(car, target)
        margin = t - eta
        if margin < minimum_margin:
            continue

        aerial = position.z > 300.0
        # Prefer an early usable touch, but value a modest setup margin.
        # This avoids selecting an impossible razor-thin first contact.
        useful_margin = min(max(margin, 0.0), 0.75)
        score = t - useful_margin * 0.35 + max(0.0, position.z - 300.0) / 10000.0

        candidates.append(InterceptCandidate(
            time=t,
            position=position,
            velocity=velocity,
            arrival_time=eta,
            margin=margin,
            score=score,
            aerial=aerial,
            reachable=True,
        ))

    candidates.sort(key=lambda candidate: candidate.score)
    return candidates


def find_earliest_intercept(car: VehicleState, trajectory, **kwargs) -> InterceptResult:
    candidates = find_intercepts(car, trajectory, **kwargs)
    if not candidates:
        return InterceptResult(valid=False, candidates=[], reason="no reachable intercept")
    best = min(candidates, key=lambda candidate: candidate.time)
    return InterceptResult(valid=True, candidate=best, candidates=candidates,
                           reason="earliest reachable intercept")


def find_best_intercept(car: VehicleState, trajectory, **kwargs) -> InterceptResult:
    candidates = find_intercepts(car, trajectory, **kwargs)
    if not candidates:
        return InterceptResult(valid=False, candidates=[], reason="no reachable intercept")
    best = min(candidates, key=lambda candidate: candidate.score)
    return InterceptResult(valid=True, candidate=best, candidates=candidates,
                           reason="best scored intercept")


def intercept_advantage(
    our_team: list[VehicleState],
    opponents: list[VehicleState],
    trajectory,
    *,
    maximum_time: float = 3.0,
    step: float = 0.1,
) -> InterceptAdvantage:
    """Compare both teams' best arrival times over the same prediction samples."""
    result = InterceptAdvantage()
    samples = sample_trajectory(trajectory, step=step, maximum_time=maximum_time)

    for t, position, _ in samples:
        for car in our_team:
            eta = estimate_arrival_time(car, _contact_target(position, car))
            if eta <= t + 0.05 and eta < result.our_best_time:
                result.our_best_time = eta
                result.our_best_car = car
                result.our_best_position = position

        for car in opponents:
            eta = estimate_arrival_time(car, _contact_target(position, car))
            if eta <= t + 0.05 and eta < result.opponent_best_time:
                result.opponent_best_time = eta
                result.opponent_best_car = car
                result.opponent_best_position = position

        if result.our_best_car is not None and result.opponent_best_car is not None:
            break

    return result
