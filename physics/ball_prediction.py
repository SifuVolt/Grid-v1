
"""Grid ball prediction and collision-aware fallback simulation.

Uses RLBot's supplied ball prediction whenever available.
When prediction data is missing, simulates gravity and approximate
Soccar collisions with the floor, side walls, end walls and ceiling.

The fallback is deliberately approximate. RLBot's game prediction
remains the preferred source for real match decisions.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import ceil

from physics.math3d import Vec3, clamp
from physics.trajectory import BallState


# Soccar dimensions.
FIELD_HALF_WIDTH = 4096.0
FIELD_HALF_LENGTH = 5120.0
GOAL_HALF_WIDTH = 892.75
GOAL_HEIGHT = 642.775
GOAL_DEPTH = 880.0
CEILING_HEIGHT = 2044.0
BALL_RADIUS = 91.25

# Approximate collision response.
FLOOR_RESTITUTION = 0.60
WALL_RESTITUTION = 0.60
CEILING_RESTITUTION = 0.45
GROUND_FRICTION = 0.985
AIR_DRAG = 0.999
MAX_BALL_SPEED = 6000.0
GRAVITY = 650.0


@dataclass
class BallPredictionConfig:
    gravity: float = GRAVITY
    simulation_step: float = 1.0 / 120.0
    sample_interval: float = 1.0 / 30.0
    horizon: float = 6.0

    floor_restitution: float = FLOOR_RESTITUTION
    wall_restitution: float = WALL_RESTITUTION
    ceiling_restitution: float = CEILING_RESTITUTION

    ground_friction: float = GROUND_FRICTION
    air_drag: float = AIR_DRAG

    max_ball_speed: float = MAX_BALL_SPEED
    ball_radius: float = BALL_RADIUS

    field_half_width: float = FIELD_HALF_WIDTH
    field_half_length: float = FIELD_HALF_LENGTH
    goal_half_width: float = GOAL_HALF_WIDTH
    goal_height: float = GOAL_HEIGHT
    goal_depth: float = GOAL_DEPTH
    ceiling_height: float = CEILING_HEIGHT


@dataclass
class PredictedBallSample:
    time: float
    position: Vec3
    velocity: Vec3

    @property
    def game_seconds(self) -> float:
        return self.time


@dataclass
class BallPrediction:
    samples: list[PredictedBallSample] = field(default_factory=list)
    source: str = "fallback"

    @property
    def duration(self) -> float:
        if not self.samples:
            return 0.0
        return self.samples[-1].time

    def __len__(self) -> int:
        return len(self.samples)

    def position_at(self, time: float) -> Vec3:
        sample = self._sample_at(time)
        return sample.position if sample else Vec3(0.0, 0.0, 0.0)

    def velocity_at(self, time: float) -> Vec3:
        sample = self._sample_at(time)
        return sample.velocity if sample else Vec3(0.0, 0.0, 0.0)

    def _sample_at(self, time: float) -> PredictedBallSample | None:
        if not self.samples:
            return None

        if time <= self.samples[0].time:
            return self.samples[0]

        if time >= self.samples[-1].time:
            return self.samples[-1]

        low = 0
        high = len(self.samples) - 1

        while low + 1 < high:
            middle = (low + high) // 2

            if self.samples[middle].time <= time:
                low = middle
            else:
                high = middle

        a = self.samples[low]
        b = self.samples[high]

        span = b.time - a.time
        if span <= 1e-8:
            return a

        alpha = clamp((time - a.time) / span, 0.0, 1.0)

        return PredictedBallSample(
            time=time,
            position=a.position + (b.position - a.position) * alpha,
            velocity=a.velocity + (b.velocity - a.velocity) * alpha,
        )


def _read_value(obj, *names, default=None):
    for name in names:
        try:
            value = getattr(obj, name)
        except (AttributeError, TypeError):
            continue

        if value is not None:
            return value

    return default


def _read_vector(obj) -> Vec3 | None:
    if obj is None:
        return None

    try:
        return Vec3(
            float(_read_value(obj, "x", default=0.0)),
            float(_read_value(obj, "y", default=0.0)),
            float(_read_value(obj, "z", default=0.0)),
        )
    except (TypeError, ValueError):
        return None


def _prediction_slices(prediction):
    if prediction is None:
        return []

    slices = _read_value(
        prediction,
        "slices",
        "ball_prediction",
        default=None,
    )

    if slices is None:
        return []

    try:
        return list(slices)
    except TypeError:
        return []


def prediction_from_rlbot(
    prediction,
    current_game_seconds: float = 0.0,
) -> BallPrediction | None:
    """Convert RLBot prediction slices into relative-time samples."""

    raw_slices = _prediction_slices(prediction)

    if not raw_slices:
        return None

    samples: list[PredictedBallSample] = []

    for item in raw_slices:
        physics = _read_value(item, "physics", default=item)

        location = _read_vector(
            _read_value(physics, "location", default=None)
        )
        velocity = _read_vector(
            _read_value(physics, "velocity", default=None)
        )

        game_time = _read_value(
            item,
            "game_seconds",
            "game_time",
            "time",
            default=None,
        )

        if location is None or velocity is None or game_time is None:
            continue

        try:
            relative_time = float(game_time) - current_game_seconds
        except (TypeError, ValueError):
            continue

        if relative_time < -0.02:
            continue

        samples.append(
            PredictedBallSample(
                time=max(0.0, relative_time),
                position=location,
                velocity=velocity,
            )
        )

    if len(samples) < 2:
        return None

    samples.sort(key=lambda sample: sample.time)

    # Remove duplicate timestamps.
    unique: list[PredictedBallSample] = []

    for sample in samples:
        if unique and sample.time <= unique[-1].time + 1e-6:
            continue
        unique.append(sample)

    if len(unique) < 2:
        return None

    return BallPrediction(
        samples=unique,
        source="rlbot",
    )


def _inside_goal_tunnel(
    position: Vec3,
    config: BallPredictionConfig,
) -> bool:
    return (
        abs(position.x) < config.goal_half_width - config.ball_radius
        and position.z < config.goal_height - config.ball_radius
        and abs(position.y) > config.field_half_length
        and abs(position.y) < config.field_half_length + config.goal_depth
    )


def _collide_floor(
    position: Vec3,
    velocity: Vec3,
    config: BallPredictionConfig,
) -> tuple[Vec3, Vec3]:
    floor_z = config.ball_radius

    if position.z >= floor_z:
        return position, velocity

    position = position.with_z(floor_z)

    if velocity.z < 0:
        velocity = velocity.with_z(
            -velocity.z * config.floor_restitution
        )

    velocity = Vec3(
        velocity.x * config.ground_friction,
        velocity.y * config.ground_friction,
        velocity.z,
    )

    if abs(velocity.z) < 35.0:
        velocity = velocity.with_z(0.0)

    return position, velocity


def _collide_ceiling(
    position: Vec3,
    velocity: Vec3,
    config: BallPredictionConfig,
) -> tuple[Vec3, Vec3]:
    ceiling_z = config.ceiling_height - config.ball_radius

    if position.z <= ceiling_z:
        return position, velocity

    position = position.with_z(ceiling_z)

    if velocity.z > 0:
        velocity = velocity.with_z(
            -velocity.z * config.ceiling_restitution
        )

    return position, velocity


def _collide_side_walls(
    position: Vec3,
    velocity: Vec3,
    config: BallPredictionConfig,
) -> tuple[Vec3, Vec3]:
    limit = config.field_half_width - config.ball_radius

    if position.x > limit:
        position = Vec3(limit, position.y, position.z)

        if velocity.x > 0:
            velocity = Vec3(
                -velocity.x * config.wall_restitution,
                velocity.y,
                velocity.z,
            )

    elif position.x < -limit:
        position = Vec3(-limit, position.y, position.z)

        if velocity.x < 0:
            velocity = Vec3(
                -velocity.x * config.wall_restitution,
                velocity.y,
                velocity.z,
            )

    return position, velocity


def _collide_end_walls(
    position: Vec3,
    velocity: Vec3,
    config: BallPredictionConfig,
) -> tuple[Vec3, Vec3]:
    if _inside_goal_tunnel(position, config):
        return position, velocity

    limit = config.field_half_length - config.ball_radius

    if position.y > limit:
        position = Vec3(position.x, limit, position.z)

        if velocity.y > 0:
            velocity = Vec3(
                velocity.x,
                -velocity.y * config.wall_restitution,
                velocity.z,
            )

    elif position.y < -limit:
        position = Vec3(position.x, -limit, position.z)

        if velocity.y < 0:
            velocity = Vec3(
                velocity.x,
                -velocity.y * config.wall_restitution,
                velocity.z,
            )

    return position, velocity


def _apply_collisions(
    position: Vec3,
    velocity: Vec3,
    config: BallPredictionConfig,
) -> tuple[Vec3, Vec3]:
    position, velocity = _collide_floor(
        position, velocity, config
    )
    position, velocity = _collide_ceiling(
        position, velocity, config
    )
    position, velocity = _collide_side_walls(
        position, velocity, config
    )
    position, velocity = _collide_end_walls(
        position, velocity, config
    )

    return position, velocity


def simulate_ball_prediction(
    ball: BallState,
    config: BallPredictionConfig | None = None,
    *,
    horizon: float | None = None,
) -> BallPrediction:
    """Simulate a collision-aware fallback ball trajectory."""

    config = config or BallPredictionConfig()

    horizon = (
        config.horizon
        if horizon is None
        else max(0.0, float(horizon))
    )

    dt = max(1.0 / 1000.0, config.simulation_step)
    sample_interval = max(dt, config.sample_interval)

    position = Vec3.from_value(ball.location)
    velocity = Vec3.from_value(ball.velocity)

    samples = [
        PredictedBallSample(
            time=0.0,
            position=position,
            velocity=velocity,
        )
    ]

    elapsed = 0.0
    next_sample_time = sample_interval

    steps = int(ceil(horizon / dt))

    for _ in range(steps):
        step = min(dt, horizon - elapsed)

        if step <= 0:
            break

        # Semi-implicit Euler integration.
        velocity = velocity + Vec3(
            0.0,
            0.0,
            -config.gravity * step,
        )

        velocity = velocity * config.air_drag
        speed = velocity.length()
        if speed > config.max_ball_speed:
            velocity = velocity.normalized() * config.max_ball_speed

        position = position + velocity * step

        position, velocity = _apply_collisions(
            position,
            velocity,
            config,
        )

        elapsed += step

        if elapsed + 1e-6 >= next_sample_time:
            samples.append(
                PredictedBallSample(
                    time=elapsed,
                    position=position,
                    velocity=velocity,
                )
            )
            next_sample_time += sample_interval

    if samples[-1].time < horizon:
        samples.append(
            PredictedBallSample(
                time=horizon,
                position=position,
                velocity=velocity,
            )
        )

    return BallPrediction(
        samples=samples,
        source="collision_fallback",
    )


def predict_ball(
    ball: BallState,
    prediction=None,
    current_game_seconds: float = 0.0,
    config: BallPredictionConfig | None = None,
    *,
    horizon: float | None = None,
) -> BallPrediction:
    """Prefer RLBot prediction; fall back to local collision simulation."""

    supplied = prediction_from_rlbot(
        prediction,
        current_game_seconds=current_game_seconds,
    )

    if supplied is not None:
        return supplied

    return simulate_ball_prediction(
        ball,
        config,
        horizon=horizon,
    )
