
"""Grid jump physics and vertical reachability.

Inspired by Party Cannon:
- Utils.TimeToJump()
- JumpShot.cs
- DoubleJumpShot.cs

Models:
- Initial jump impulse.
- Variable first-jump hold acceleration.
- Gravity.
- Second-jump impulse after a release period.
- Single and double jump height reachability.

This is a configurable approximation. Constants should eventually be
calibrated against live Rocket League observations.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import inf
from typing import Iterable

from physics.math3d import Vec3, UP, ZERO, clamp


# Approximate Rocket League jump constants.
GRAVITY = 650.0
JUMP_IMPULSE = 291.667
JUMP_ACCELERATION = 1458.333
JUMP_MAX_HOLD = 0.20

# Party Cannon releases jump for three frames before its second jump.
DOUBLE_JUMP_RELEASE = 3.0 / 120.0

SIMULATION_DT = 1.0 / 240.0


@dataclass
class JumpConfig:
    gravity: float = GRAVITY
    jump_impulse: float = JUMP_IMPULSE
    jump_acceleration: float = JUMP_ACCELERATION
    max_hold_time: float = JUMP_MAX_HOLD

    second_jump_release: float = DOUBLE_JUMP_RELEASE
    simulation_dt: float = SIMULATION_DT

    # Small tolerance for numerical integration.
    height_tolerance: float = 2.0


@dataclass
class JumpSample:
    time: float
    height: float
    velocity: float


@dataclass
class JumpTrajectory:
    samples: list[JumpSample] = field(default_factory=list)

    maximum_height: float = 0.0
    time_to_peak: float = 0.0
    total_time: float = 0.0

    double_jump: bool = False
    valid: bool = True

    def height_at(self, time: float) -> float:
        if not self.samples:
            return 0.0

        if time <= self.samples[0].time:
            return self.samples[0].height

        for a, b in zip(self.samples, self.samples[1:]):
            if a.time <= time <= b.time:
                span = b.time - a.time

                if span <= 0:
                    return b.height

                alpha = (time - a.time) / span

                return a.height + (b.height - a.height) * alpha

        return self.samples[-1].height

    def velocity_at(self, time: float) -> float:
        if not self.samples:
            return 0.0

        if time <= self.samples[0].time:
            return self.samples[0].velocity

        for a, b in zip(self.samples, self.samples[1:]):
            if a.time <= time <= b.time:
                span = b.time - a.time

                if span <= 0:
                    return b.velocity

                alpha = (time - a.time) / span

                return a.velocity + (b.velocity - a.velocity) * alpha

        return self.samples[-1].velocity

    def time_to_height(self, height: float) -> float:
        """Return the first time the jump reaches a requested height."""

        if height <= 0:
            return 0.0

        for a, b in zip(self.samples, self.samples[1:]):
            if a.height < height <= b.height:
                span = b.height - a.height

                if span <= 0:
                    return a.time

                alpha = (height - a.height) / span

                return a.time + (b.time - a.time) * alpha

        return inf


@dataclass
class JumpReachability:
    reachable: bool

    target_height: float
    maximum_height: float
    time: float

    double_jump: bool
    reason: str = ""

    @property
    def margin(self) -> float:
        if not self.reachable:
            return -inf

        return self.time


def _normalise_surface_normal(normal: Vec3 | None) -> Vec3:
    if normal is None:
        return UP

    normal = Vec3.from_value(normal).normalized()

    if normal.length() < 0.5:
        return UP

    return normal


def _gravity_along_normal(
    normal: Vec3,
    gravity: float,
) -> float:
    """Gravity acceleration projected onto the jump surface normal."""

    world_gravity = Vec3(0.0, 0.0, -gravity)

    return world_gravity.dot(normal)


def simulate_jump(
    *,
    double_jump: bool = False,
    initial_velocity: float = 0.0,
    hold_time: float | None = None,
    config: JumpConfig | None = None,
    surface_normal: Vec3 = UP,
) -> JumpTrajectory:
    """Simulate a jump along a surface normal.

    initial_velocity is the car's existing velocity along the normal.
    hold_time controls how long the first jump button is held.
    """

    config = config or JumpConfig()

    normal = _normalise_surface_normal(surface_normal)

    dt = max(0.001, config.simulation_dt)

    hold_time = (
        config.max_hold_time
        if hold_time is None
        else clamp(hold_time, 0.0, config.max_hold_time)
    )

    gravity_accel = _gravity_along_normal(
        normal,
        config.gravity,
    )

    velocity = initial_velocity + config.jump_impulse
    height = 0.0
    elapsed = 0.0

    second_jump_time = (
        hold_time + config.second_jump_release
        if double_jump
        else inf
    )

    second_jump_used = False

    samples = [
        JumpSample(
            time=0.0,
            height=height,
            velocity=velocity,
        )
    ]

    maximum_height = height
    time_to_peak = 0.0

    # Stop once the car has returned close to the starting height
    # and is travelling downwards.
    maximum_duration = 3.0

    while elapsed < maximum_duration:
        step = min(dt, maximum_duration - elapsed)

        next_time = elapsed + step

        if (
            double_jump
            and not second_jump_used
            and elapsed < second_jump_time <= next_time
        ):
            # Apply the second jump impulse at the release boundary.
            velocity += config.jump_impulse
            second_jump_used = True

        # First-jump acceleration only applies while jump is held.
        jump_accel = (
            config.jump_acceleration
            if elapsed < hold_time
            else 0.0
        )

        acceleration = gravity_accel + jump_accel

        next_height = (
            height
            + velocity * step
            + 0.5 * acceleration * step * step
        )

        next_velocity = velocity + acceleration * step

        elapsed = next_time
        height = next_height
        velocity = next_velocity

        samples.append(
            JumpSample(
                time=elapsed,
                height=height,
                velocity=velocity,
            )
        )

        if height > maximum_height:
            maximum_height = height
            time_to_peak = elapsed

        if elapsed > hold_time and height <= 0 and velocity < 0:
            break

    return JumpTrajectory(
        samples=samples,
        maximum_height=maximum_height,
        time_to_peak=time_to_peak,
        total_time=elapsed,
        double_jump=double_jump,
        valid=bool(samples),
    )


def maximum_jump_height(
    *,
    double_jump: bool = False,
    initial_velocity: float = 0.0,
    hold_time: float | None = None,
    config: JumpConfig | None = None,
    surface_normal: Vec3 = UP,
) -> float:
    """Return maximum height above the starting surface."""

    trajectory = simulate_jump(
        double_jump=double_jump,
        initial_velocity=initial_velocity,
        hold_time=hold_time,
        config=config,
        surface_normal=surface_normal,
    )

    return trajectory.maximum_height


def time_to_jump_height(
    height: float,
    *,
    double_jump: bool = False,
    initial_velocity: float = 0.0,
    hold_time: float | None = None,
    config: JumpConfig | None = None,
    surface_normal: Vec3 = UP,
) -> float:
    """Estimate the first time the car reaches a requested height."""

    if height <= 0:
        return 0.0

    trajectory = simulate_jump(
        double_jump=double_jump,
        initial_velocity=initial_velocity,
        hold_time=hold_time,
        config=config,
        surface_normal=surface_normal,
    )

    return trajectory.time_to_height(height)


def estimate_jump_reachability(
    car_location: Vec3,
    target_location: Vec3,
    *,
    car_velocity: Vec3 = ZERO,
    surface_normal: Vec3 = UP,
    double_jump: bool = False,
    available_time: float | None = None,
    config: JumpConfig | None = None,
) -> JumpReachability:
    """Check whether a jump can reach a target's height in time.

    The target and car positions are projected onto the surface normal.
    Horizontal travel and aerial steering are deliberately not included.
    """

    normal = _normalise_surface_normal(surface_normal)

    car_location = Vec3.from_value(car_location)
    target_location = Vec3.from_value(target_location)
    car_velocity = Vec3.from_value(car_velocity)

    target_height = (
        target_location - car_location
    ).dot(normal)

    initial_velocity = car_velocity.dot(normal)

    trajectory = simulate_jump(
        double_jump=double_jump,
        initial_velocity=initial_velocity,
        config=config,
        surface_normal=normal,
    )

    maximum_height = trajectory.maximum_height

    if target_height < -10:
        return JumpReachability(
            reachable=False,
            target_height=target_height,
            maximum_height=maximum_height,
            time=inf,
            double_jump=double_jump,
            reason="target is behind the jump surface",
        )

    if target_height > maximum_height + (
        config.height_tolerance if config else 2.0
    ):
        return JumpReachability(
            reachable=False,
            target_height=target_height,
            maximum_height=maximum_height,
            time=inf,
            double_jump=double_jump,
            reason="target exceeds maximum jump height",
        )

    time = trajectory.time_to_height(
        max(0.0, target_height)
    )

    if time == inf:
        return JumpReachability(
            reachable=False,
            target_height=target_height,
            maximum_height=maximum_height,
            time=inf,
            double_jump=double_jump,
            reason="target height not reached",
        )

    if available_time is not None and time > available_time:
        return JumpReachability(
            reachable=False,
            target_height=target_height,
            maximum_height=maximum_height,
            time=time,
            double_jump=double_jump,
            reason="insufficient time to jump",
        )

    return JumpReachability(
        reachable=True,
        target_height=target_height,
        maximum_height=maximum_height,
        time=time,
        double_jump=double_jump,
        reason="height reachable",
    )


def can_reach_height(
    height: float,
    available_time: float,
    *,
    double_jump: bool = False,
    config: JumpConfig | None = None,
) -> bool:
    """Convenience check for a height and time budget."""

    time = time_to_jump_height(
        height,
        double_jump=double_jump,
        config=config,
    )

    return time <= available_time


def compare_jump_types(
    height: float,
    available_time: float,
    config: JumpConfig | None = None,
) -> tuple[JumpReachability, JumpReachability]:
    """Compare single and double jump reachability."""

    single = estimate_jump_reachability(
        ZERO,
        Vec3(0.0, 0.0, height),
        available_time=available_time,
        config=config,
    )

    double = estimate_jump_reachability(
        ZERO,
        Vec3(0.0, 0.0, height),
        double_jump=True,
        available_time=available_time,
        config=config,
    )

    return single, double


def jump_height_samples(
    *,
    double_jump: bool = False,
    config: JumpConfig | None = None,
) -> Iterable[JumpSample]:
    """Expose the simulated jump curve for debugging or plotting."""

    return simulate_jump(
        double_jump=double_jump,
        config=config,
    ).samples
