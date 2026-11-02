
"""Grid interception and arrival-time estimation.

Inspired by Party Cannon's InterceptCalculator and Noob Black's
prediction-based interception routines.

Supports both physics.trajectory.Trajectory and
physics.ball_prediction.BallPrediction through their shared sample API.
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
MAX_TURNING_SPEED = 1800.0

BOOST_SPEEDS = (0.0, 500.0, 1000.0, 1410.0, 1800.0, 2000.0, 2300.0)
BOOST_TIMES = (0.0, 0.5, 1.0, 1.4, 1.8, 2.1, 2.4)

THROTTLE_SPEEDS = (0.0, 500.0, 1000.0, 1410.0)
THROTTLE_TIMES = (0.0, 0.8, 1.6, 2.5)


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


def _interpolate_speed(
    times: tuple[float, ...],
    speeds: tuple[float, ...],
    target_time: float,
    base_speed: float,
) -> float:
    if target_time <= 0:
        return base_speed

    for i in range(len(times) - 1):
        if target_time <= times[i + 1]:
            fraction = (
                (target_time - times[i])
                / (times[i + 1] - times[i])
            )
            return base_speed + speeds[i] + fraction * (
                speeds[i + 1] - speeds[i]
            )

    return base_speed + speeds[-1]


def _solve_time_for_distance(
    initial_speed: float,
    acceleration: float,
    distance: float,
) -> float:
    if distance <= 0:
        return 0.0

    discriminant = initial_speed ** 2 + 2.0 * acceleration * distance

    if discriminant <= 0:
        return distance / max(initial_speed, 400.0)

    return (
        -initial_speed + sqrt(discriminant)
    ) / acceleration


def _turning_radius(speed: float) -> float:
    speed = clamp(speed, 0.0, MAX_SPEED)

    curvature = (
        0.0069
        - 6.19e-6 * speed
        + 1.1e-8 * speed * speed
    )

    return 1.0 / max(curvature, 0.001)


def estimate_time_directional(
    car: VehicleState,
    target: Vec3,
    direction: int = 1,
    *,
    use_boost: bool = True,
) -> float:
    """Estimate travel time using turn, boost and throttle phases."""

    direction = 1 if direction >= 0 else -1

    displacement = target - car.location
    distance = displacement.length()

    if distance < 100.0:
        return 0.0

    forward = car.orientation.forward * direction
    speed = max(car.velocity.dot(car.orientation.forward) * direction, 0.0)

    target_direction = displacement.normalized()
    angle = forward.angle_to(target_direction)

    turning_radius = _turning_radius(max(speed, 500.0))
    turn_time = angle * turning_radius / MAX_TURNING_SPEED

    if angle < 0.5:
        turn_time = 0.0

    total_time = turn_time
    distance_left = max(0.0, distance - 200.0)

    if distance_left <= 0:
        return turn_time

    if use_boost and direction > 0 and car.boost > 0:
        boost_time = min(
            car.boost / BOOST_CONSUMPTION,
            BOOST_TIMES[-1],
        )

        speed_after_boost = _interpolate_speed(
            BOOST_TIMES,
            BOOST_SPEEDS,
            boost_time,
            speed,
        )

        boost_distance = (
            (speed + speed_after_boost) * 0.5 * boost_time
        )

        if boost_distance >= distance_left:
            return (
                total_time
                + _solve_time_for_distance(
                    speed,
                    BOOST_ACCEL,
                    distance_left,
                )
            ) * 1.05

        total_time += boost_time
        distance_left -= boost_distance
        speed = speed_after_boost

    if speed < MAX_THROTTLE_SPEED and distance_left > 0:
        throttle_time = (
            MAX_THROTTLE_SPEED - speed
        ) / THROTTLE_ACCEL

        throttle_distance = (
            speed * throttle_time
            + 0.5 * THROTTLE_ACCEL * throttle_time ** 2
        )

        if throttle_distance >= distance_left:
            return (
                total_time
                + _solve_time_for_distance(
                    speed,
                    THROTTLE_ACCEL,
                    distance_left,
                )
            ) * 1.05

        total_time += throttle_time
        distance_left -= throttle_distance
        speed = MAX_THROTTLE_SPEED

    if distance_left > 0:
        total_time += distance_left / max(speed, 800.0)

    return total_time * 1.05


def estimate_arrival_time(
    car: VehicleState,
    target: Vec3,
    *,
    allow_backwards: bool = True,
    use_boost: bool = True,
) -> float:
    """Estimate the fastest reasonable forward or reverse arrival."""

    forward_eta = estimate_time_directional(
        car,
        target,
        1,
        use_boost=use_boost,
    )

    if not allow_backwards:
        return forward_eta

    reverse_speed = max(
        -car.velocity.dot(car.orientation.forward),
        0.0,
    )

    distance = (target - car.location).length()

    if (
        reverse_speed > 500.0
        or distance < 300.0
        or distance > 3000.0
    ):
        reverse_eta = estimate_time_directional(
            car,
            target,
            -1,
            use_boost=False,
        ) + 0.5

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

    duration = min(
        float(getattr(trajectory, "duration", maximum_time)),
        maximum_time,
    )

    if duration <= 0:
        return []

    samples = []
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

        t += max(0.02, step)

    return samples


def _contact_target(
    position: Vec3,
    car: VehicleState,
) -> Vec3:
    """Estimate a car-centre contact position behind the ball."""

    direction = position - car.location

    if direction.length() < 1e-5:
        direction = car.orientation.forward

    direction = direction.normalized()

    # Approximate front-of-car contact offset.
    return position - direction * 120.0


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
    """Find predicted ball samples the car can arrive at in time."""

    candidates: list[InterceptCandidate] = []

    for t, position, velocity in sample_trajectory(
        trajectory,
        step=step,
        maximum_time=maximum_time,
    ):
        if position.z > maximum_height and not allow_aerial:
            continue

        target = _contact_target(position, car)
        eta = estimate_arrival_time(car, target)

        margin = t - eta

        if margin < minimum_margin:
            continue

        aerial = position.z > 300.0

        # Prefer earlier contacts, but reward a useful arrival margin.
        score = (
            t
            - min(max(margin, 0.0), 0.75) * 0.35
            + max(0.0, position.z - 300.0) / 10000.0
        )

        candidates.append(
            InterceptCandidate(
                time=t,
                position=position,
                velocity=velocity,
                arrival_time=eta,
                margin=margin,
                score=score,
                aerial=aerial,
                reachable=True,
            )
        )

    candidates.sort(key=lambda candidate: candidate.score)
    return candidates


def find_earliest_intercept(
    car: VehicleState,
    trajectory,
    **kwargs,
) -> InterceptResult:
    candidates = find_intercepts(car, trajectory, **kwargs)

    if not candidates:
        return InterceptResult(
            valid=False,
            candidates=[],
            reason="no reachable intercept",
        )

    return InterceptResult(
        valid=True,
        candidate=candidates[0],
        candidates=candidates,
        reason="earliest reachable intercept",
    )


def find_best_intercept(
    car: VehicleState,
    trajectory,
    **kwargs,
) -> InterceptResult:
    candidates = find_intercepts(car, trajectory, **kwargs)

    if not candidates:
        return InterceptResult(
            valid=False,
            candidates=[],
            reason="no reachable intercept",
        )

    best = min(candidates, key=lambda candidate: candidate.score)

    return InterceptResult(
        valid=True,
        candidate=best,
        candidates=candidates,
        reason="best scored intercept",
    )


def intercept_advantage(
    our_team: list[VehicleState],
    opponents: list[VehicleState],
    trajectory,
    *,
    maximum_time: float = 3.0,
    step: float = 0.1,
) -> InterceptAdvantage:
    """Compare the earliest reachable ball arrival for both teams."""

    result = InterceptAdvantage()

    samples = sample_trajectory(
        trajectory,
        step=step,
        maximum_time=maximum_time,
    )

    for t, position, _ in samples:
        for car in our_team:
            eta = estimate_arrival_time(
                car,
                _contact_target(position, car),
            )

            if eta <= t + 0.05 and eta < result.our_best_time:
                result.our_best_time = eta
                result.our_best_car = car
                result.our_best_position = position

        for car in opponents:
            eta = estimate_arrival_time(
                car,
                _contact_target(position, car),
            )

            if eta <= t + 0.05 and eta < result.opponent_best_time:
                result.opponent_best_time = eta
                result.opponent_best_car = car
                result.opponent_best_position = position

        if (
            result.our_best_car is not None
            and result.opponent_best_car is not None
        ):
            break

    return result
