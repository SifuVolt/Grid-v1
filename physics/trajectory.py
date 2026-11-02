
"""Ball trajectory and prediction handling for Grid.

Prediction times exposed by this module are always relative to the current
game time. RLBot prediction slices commonly contain absolute game_seconds;
those are converted when a current_game_seconds value is supplied.

The ballistic fallback is deliberately simple. It does not simulate
wall, floor, goal, or car collisions.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from bisect import bisect_right
from math import isfinite
from typing import Iterable, Sequence

from .math3d import Vec3, ZERO, clamp


BALL_RADIUS = 91.25
GRAVITY = 650.0
DEFAULT_PREDICTION_HORIZON = 6.0
DEFAULT_SAMPLE_INTERVAL = 1.0 / 60.0


def _read_component(value, name: str, default: float = 0.0) -> float:
    """Read a numeric component from an object or mapping."""
    if value is None:
        return default

    if isinstance(value, dict):
        result = value.get(name, default)
    else:
        result = getattr(value, name, default)

    try:
        result = float(result)
    except (TypeError, ValueError, OverflowError):
        return default

    return result if isfinite(result) else default


def _read_vector(value, default: Vec3 = ZERO) -> Vec3:
    """Convert a packet vector, mapping, sequence, or Vec3 to Vec3."""
    if value is None:
        return default

    if isinstance(value, Vec3):
        return value

    if isinstance(value, dict):
        return Vec3(
            _read_component(value, "x"),
            _read_component(value, "y"),
            _read_component(value, "z"),
        )

    if hasattr(value, "x") and hasattr(value, "y") and hasattr(value, "z"):
        return Vec3(
            _read_component(value, "x"),
            _read_component(value, "y"),
            _read_component(value, "z"),
        )

    try:
        return Vec3(
            float(value[0]),
            float(value[1]),
            float(value[2]),
        )
    except (TypeError, ValueError, IndexError, KeyError, OverflowError):
        return default


def _read_member(value, name: str, default=None):
    if value is None:
        return default
    if isinstance(value, dict):
        return value.get(name, default)
    return getattr(value, name, default)


@dataclass(frozen=True, slots=True)
class BallState:
    """A snapshot of the ball."""

    location: Vec3
    velocity: Vec3 = ZERO
    angular_velocity: Vec3 = ZERO
    radius: float = BALL_RADIUS

    @property
    def position(self) -> Vec3:
        return self.location

    @property
    def speed(self) -> float:
        return self.velocity.length()

    @property
    def flat_speed(self) -> float:
        return self.velocity.flat_length()

    @property
    def height(self) -> float:
        return self.location.z

    def distance_to(self, point: Vec3) -> float:
        return self.location.distance(point)

    def direction_to(self, point: Vec3) -> Vec3:
        return self.location.direction_to(point)

    @classmethod
    def from_packet_ball(cls, ball) -> BallState:
        """Build a BallState from an RLBot ball object."""
        physics = _read_member(ball, "physics", ball)

        location = _read_vector(
            _read_member(physics, "location"),
        )
        velocity = _read_vector(
            _read_member(physics, "velocity"),
        )
        angular_velocity = _read_vector(
            _read_member(physics, "angular_velocity"),
        )

        return cls(
            location=location,
            velocity=velocity,
            angular_velocity=angular_velocity,
        )


@dataclass(frozen=True, slots=True)
class TrajectorySample:
    """One point in a ball trajectory.

    time is relative to the current frame, in seconds.
    """

    time: float
    position: Vec3
    velocity: Vec3 = ZERO

    @property
    def location(self) -> Vec3:
        return self.position

    @property
    def speed(self) -> float:
        return self.velocity.length()

    def ball_state(self) -> BallState:
        return BallState(
            location=self.position,
            velocity=self.velocity,
        )


@dataclass(slots=True)
class Trajectory:
    """Ordered ball prediction samples."""

    samples: list[TrajectorySample] = field(default_factory=list)
    source: str = "unknown"

    def __post_init__(self) -> None:
        self.samples = sorted(
            (
                sample
                for sample in self.samples
                if isfinite(sample.time)
            ),
            key=lambda sample: sample.time,
        )

        # Remove duplicate timestamps. Keep the last sample at each time.
        unique: list[TrajectorySample] = []
        for sample in self.samples:
            if unique and abs(sample.time - unique[-1].time) < 1e-7:
                unique[-1] = sample
            else:
                unique.append(sample)

        self.samples = unique

    def __len__(self) -> int:
        return len(self.samples)

    def __bool__(self) -> bool:
        return bool(self.samples)

    @property
    def times(self) -> list[float]:
        return [sample.time for sample in self.samples]

    @property
    def start_time(self) -> float | None:
        return self.samples[0].time if self.samples else None

    @property
    def end_time(self) -> float | None:
        return self.samples[-1].time if self.samples else None

    @property
    def duration(self) -> float:
        if len(self.samples) < 2:
            return 0.0
        return self.samples[-1].time - self.samples[0].time

    @property
    def current(self) -> TrajectorySample | None:
        return self.samples[0] if self.samples else None

    def sample_at(self, time: float) -> TrajectorySample | None:
        """Interpolate a sample at a relative time.

        Times outside the prediction range are clamped to the first or last
        sample. No extrapolation is performed.
        """
        if not self.samples:
            return None

        if len(self.samples) == 1:
            return self.samples[0]

        if time <= self.samples[0].time:
            return self.samples[0]

        if time >= self.samples[-1].time:
            return self.samples[-1]

        times = self.times
        upper_index = bisect_right(times, time)
        lower = self.samples[upper_index - 1]
        upper = self.samples[upper_index]

        interval = upper.time - lower.time
        if interval <= 1e-9:
            return upper

        alpha = clamp((time - lower.time) / interval, 0.0, 1.0)

        position = lower.position + (
            upper.position - lower.position
        ) * alpha

        velocity = lower.velocity + (
            upper.velocity - lower.velocity
        ) * alpha

        return TrajectorySample(
            time=time,
            position=position,
            velocity=velocity,
        )

    def position_at(self, time: float) -> Vec3 | None:
        sample = self.sample_at(time)
        return sample.position if sample is not None else None

    def velocity_at(self, time: float) -> Vec3 | None:
        sample = self.sample_at(time)
        return sample.velocity if sample is not None else None

    def nearest_sample(
        self,
        time: float,
    ) -> TrajectorySample | None:
        if not self.samples:
            return None

        return min(
            self.samples,
            key=lambda sample: abs(sample.time - time),
        )

    def samples_between(
        self,
        start_time: float,
        end_time: float,
    ) -> list[TrajectorySample]:
        if end_time < start_time:
            start_time, end_time = end_time, start_time

        return [
            sample
            for sample in self.samples
            if start_time <= sample.time <= end_time
        ]

    def first_sample(
        self,
        predicate,
        start_time: float = 0.0,
        end_time: float | None = None,
    ) -> TrajectorySample | None:
        """Return the first sample satisfying a supplied condition."""
        for sample in self.samples:
            if sample.time < start_time:
                continue
            if end_time is not None and sample.time > end_time:
                break
            if predicate(sample):
                return sample
        return None

    @classmethod
    def from_prediction(
        cls,
        prediction,
        current_game_seconds: float | None = None,
        max_horizon: float = DEFAULT_PREDICTION_HORIZON,
    ) -> Trajectory:
        """Parse an RLBot ball prediction structure.

        Expected RLBot-style structure:
            prediction.slices[i].game_seconds
            prediction.slices[i].physics.location
            prediction.slices[i].physics.velocity

        If current_game_seconds is supplied, game_seconds is converted
        into a relative time. If it is omitted, the first prediction
        timestamp is treated as time zero.
        """
        raw_slices = _read_member(prediction, "slices", None)

        if raw_slices is None:
            return cls(source="invalid_prediction")

        try:
            raw_slices = list(raw_slices)
        except (TypeError, ValueError):
            return cls(source="invalid_prediction")

        parsed: list[tuple[float, Vec3, Vec3]] = []

        for prediction_slice in raw_slices:
            absolute_time = _read_member(
                prediction_slice,
                "game_seconds",
                None,
            )

            if absolute_time is None:
                continue

            try:
                absolute_time = float(absolute_time)
            except (TypeError, ValueError, OverflowError):
                continue

            if not isfinite(absolute_time):
                continue

            physics = _read_member(
                prediction_slice,
                "physics",
                None,
            )

            if physics is None:
                continue

            location = _read_vector(
                _read_member(physics, "location"),
            )
            velocity = _read_vector(
                _read_member(physics, "velocity"),
            )

            if current_game_seconds is None:
                parsed.append((absolute_time, location, velocity))
            else:
                relative_time = absolute_time - current_game_seconds
                parsed.append((relative_time, location, velocity))

        if not parsed:
            return cls(source="empty_prediction")

        parsed.sort(key=lambda item: item[0])

        if current_game_seconds is None:
            origin = parsed[0][0]
            parsed = [
                (time - origin, location, velocity)
                for time, location, velocity in parsed
            ]

        samples = [
            TrajectorySample(
                time=time,
                position=location,
                velocity=velocity,
            )
            for time, location, velocity in parsed
            if -0.02 <= time <= max_horizon
        ]

        # Discard tiny negative time differences caused by packet timing.
        samples = [
            TrajectorySample(
                time=max(0.0, sample.time),
                position=sample.position,
                velocity=sample.velocity,
            )
            for sample in samples
        ]

        return cls(samples=samples, source="rlbot_prediction")

    @classmethod
    def ballistic(
        cls,
        ball: BallState,
        horizon: float = DEFAULT_PREDICTION_HORIZON,
        interval: float = DEFAULT_SAMPLE_INTERVAL,
        gravity: float = GRAVITY,
    ) -> Trajectory:
        """Create a basic no-collision ballistic prediction.

        This is only a fallback when the real RLBot prediction is missing.
        It does not account for bounces or field collisions.
        """
        horizon = max(0.0, horizon)
        interval = max(0.005, interval)

        count = max(1, int(horizon / interval))
        samples: list[TrajectorySample] = []

        for index in range(count + 1):
            time = min(horizon, index * interval)

            position = Vec3(
                ball.location.x + ball.velocity.x * time,
                ball.location.y + ball.velocity.y * time,
                ball.location.z
                + ball.velocity.z * time
                - 0.5 * gravity * time * time,
            )

            velocity = Vec3(
                ball.velocity.x,
                ball.velocity.y,
                ball.velocity.z - gravity * time,
            )

            samples.append(
                TrajectorySample(
                    time=time,
                    position=position,
                    velocity=velocity,
                )
            )

        return cls(
            samples=samples,
            source="ballistic_fallback",
        )

    @classmethod
    def from_ball_or_prediction(
        cls,
        ball: BallState,
        prediction=None,
        current_game_seconds: float | None = None,
        max_horizon: float = DEFAULT_PREDICTION_HORIZON,
    ) -> Trajectory:
        """Prefer real RLBot prediction; fall back to ballistic motion."""
        if prediction is not None:
            trajectory = cls.from_prediction(
                prediction,
                current_game_seconds=current_game_seconds,
                max_horizon=max_horizon,
            )

            if trajectory:
                return trajectory

        return cls.ballistic(
            ball,
            horizon=max_horizon,
        )


def parse_ball_prediction(
    prediction,
    current_game_seconds: float | None = None,
    max_horizon: float = DEFAULT_PREDICTION_HORIZON,
) -> Trajectory:
    """Convenience wrapper for prediction parsing."""
    return Trajectory.from_prediction(
        prediction,
        current_game_seconds=current_game_seconds,
        max_horizon=max_horizon,
    )


def predict_ball_position(
    ball: BallState,
    time: float,
    gravity: float = GRAVITY,
) -> Vec3:
    """Basic ballistic position estimate at a future time."""
    time = max(0.0, time)

    return Vec3(
        ball.location.x + ball.velocity.x * time,
        ball.location.y + ball.velocity.y * time,
        ball.location.z
        + ball.velocity.z * time
        - 0.5 * gravity * time * time,
    )


def trajectory_from_samples(
    samples: Iterable[TrajectorySample],
    source: str = "manual",
) -> Trajectory:
    return Trajectory(
        samples=list(samples),
        source=source,
    )
