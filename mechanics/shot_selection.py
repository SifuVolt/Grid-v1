
"""Grid shot selection.

Combines:
- Party Cannon-style contact positioning and shot direction.
- Ground reachability.
- Single and double jump reachability.
- Aerial reachability and boost estimates.
- Candidate scoring across future ball predictions.

This module selects opportunities. It does not execute mechanics.
The selected mechanic must still validate the contact before committing.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from math import inf

from physics.field import (
    BALL_RADIUS,
    CEILING_HEIGHT,
    enemy_goal_for_team,
)
from physics.math3d import Vec3, clamp
from physics.reachability import estimate_reachability
from physics.jump import estimate_jump_reachability
from physics.aerial import estimate_aerial_reachability
from physics.trajectory import Trajectory
from physics.vehicle import VehicleState


class ShotType(Enum):
    NONE = auto()
    GROUND = auto()
    JUMP = auto()
    DOUBLE_JUMP = auto()
    AERIAL = auto()


@dataclass
class ShotSelectionConfig:
    horizon: float = 3.0
    sample_interval: float = 0.10
    minimum_contact_time: float = 0.15

    contact_offset: float = 165.0

    ground_height: float = 60.0
    jump_height: float = 350.0
    double_jump_height: float = 650.0

    safety_margin: float = 0.10
    maximum_contact_distance: float = 5000.0

    time_weight: float = 1.5
    margin_weight: float = 0.75
    alignment_weight: float = 0.45
    goal_progress_weight: float = 0.00015
    boost_cost_weight: float = 0.015

    behind_car_penalty: float = 0.8

    # Leave some boost for recovery and follow-up play.
    minimum_boost_reserve: float = 8.0


@dataclass
class ShotCandidate:
    shot_type: ShotType
    ball_position: Vec3
    ball_velocity: Vec3
    contact_position: Vec3
    shot_direction: Vec3

    contact_time: float
    arrival_time: float
    arrival_margin: float

    alignment: float
    goal_progress: float
    score: float

    reachable: bool
    estimated_boost: float = 0.0
    reason: str = ""


@dataclass
class ShotPlan:
    best: ShotCandidate | None = None
    candidates: list[ShotCandidate] = field(default_factory=list)

    valid: bool = False
    shot_type: ShotType = ShotType.NONE
    target: Vec3 | None = None
    contact_position: Vec3 | None = None
    contact_time: float = 0.0
    reason: str = "no viable shot"

    @property
    def arrival_margin(self) -> float:
        return self.best.arrival_margin if self.best else -inf


def _valid_ball_position(position: Vec3) -> bool:
    return BALL_RADIUS <= position.z <= CEILING_HEIGHT


def _trajectory_samples(
    trajectory: Trajectory,
    config: ShotSelectionConfig,
) -> list[tuple[float, Vec3, Vec3]]:
    """Extract valid future ball samples."""

    samples: list[tuple[float, Vec3, Vec3]] = []

    source = getattr(trajectory, "samples", None)

    if source:
        for sample in source:
            try:
                time = float(sample.time)
                position = Vec3.from_value(sample.position)
                velocity = Vec3.from_value(sample.velocity)
            except (AttributeError, TypeError, ValueError):
                continue

            if not (
                config.minimum_contact_time
                <= time
                <= config.horizon
            ):
                continue

            if not _valid_ball_position(position):
                continue

            samples.append((time, position, velocity))

        if samples:
            return samples

    time = config.minimum_contact_time

    while time <= config.horizon:
        try:
            position = Vec3.from_value(
                trajectory.position_at(time)
            )
            velocity = Vec3.from_value(
                trajectory.velocity_at(time)
            )
        except (AttributeError, TypeError, ValueError):
            break

        if _valid_ball_position(position):
            samples.append((time, position, velocity))

        time += config.sample_interval

    return samples


def _shot_direction(
    ball_position: Vec3,
    ball_velocity: Vec3,
    goal_position: Vec3,
) -> Vec3:
    """Choose a goalward shot direction, lightly accounting for ball motion."""

    to_goal = ball_position.flat_direction_to(goal_position)

    if to_goal.flat_magnitude < 0.5:
        return ball_velocity.flatten().flat_normalized()

    velocity_direction = ball_velocity.flatten().flat_normalized()

    if velocity_direction.flat_magnitude < 0.5:
        return to_goal

    return (
        to_goal * 0.75 + velocity_direction * 0.25
    ).flat_normalized()


def _contact_type(height: float) -> ShotType:
    if height <= 0:
        return ShotType.GROUND
    if height <= 170:
        return ShotType.GROUND
    if height <= 350:
        return ShotType.JUMP
    if height <= 650:
        return ShotType.DOUBLE_JUMP
    return ShotType.AERIAL


def _contact_height(position: Vec3) -> float:
    return max(0.0, position.z - 100.0)


def _select_reachable_mechanic(
    car: VehicleState,
    contact: Vec3,
    sample_time: float,
    horizontal_eta: float,
    config: ShotSelectionConfig,
) -> tuple[ShotType, float, str] | None:
    """Choose the simplest mechanic that can reach the contact."""

    height = _contact_height(contact)
    jump_time_budget = sample_time - horizontal_eta

    # A low contact can be reached without jumping.
    if height <= config.ground_height:
        if horizontal_eta + config.safety_margin <= sample_time:
            return (
                ShotType.GROUND,
                0.0,
                "ground contact reachable",
            )

    # Jump and double-jump need time after the ground approach.
    if jump_time_budget > 0:
        single = estimate_jump_reachability(
            car.location,
            contact,
            car_velocity=car.velocity,
            available_time=jump_time_budget,
            double_jump=False,
        )

        if single.reachable:
            return (
                ShotType.JUMP,
                0.0,
                "single jump contact reachable",
            )

        double = estimate_jump_reachability(
            car.location,
            contact,
            car_velocity=car.velocity,
            available_time=jump_time_budget,
            double_jump=True,
        )

        if double.reachable:
            return (
                ShotType.DOUBLE_JUMP,
                0.0,
                "double jump contact reachable",
            )

    # For higher contacts, use the full 3D aerial estimate.
    aerial = estimate_aerial_reachability(
        car,
        contact,
        sample_time,
        double_jump=False,
    )

    if aerial.reachable:
        if aerial.estimated_boost + config.minimum_boost_reserve <= car.boost:
            return (
                ShotType.AERIAL,
                aerial.estimated_boost,
                "aerial contact reachable",
            )

    double_aerial = estimate_aerial_reachability(
        car,
        contact,
        sample_time,
        double_jump=True,
    )

    if double_aerial.reachable:
        if (
            double_aerial.estimated_boost
            + config.minimum_boost_reserve
            <= car.boost
        ):
            return (
                ShotType.AERIAL,
                double_aerial.estimated_boost,
                "double-jump aerial contact reachable",
            )

    return None


def _calculate_candidate(
    car: VehicleState,
    sample_time: float,
    ball_position: Vec3,
    ball_velocity: Vec3,
    goal_position: Vec3,
    config: ShotSelectionConfig,
) -> ShotCandidate | None:
    direction = _shot_direction(
        ball_position,
        ball_velocity,
        goal_position,
    )

    if direction.flat_magnitude < 0.5:
        return None

    contact = ball_position - direction * config.contact_offset

    if not _valid_ball_position(ball_position):
        return None

    distance = car.location.flat_distance(contact)

    if distance > config.maximum_contact_distance:
        return None

    # Ground travel estimate is used for the approach phase.
    result = estimate_reachability(car, contact)

    if not bool(getattr(result, "reachable", False)):
        return None

    try:
        arrival_time = float(result.time)
    except (AttributeError, TypeError, ValueError):
        return None

    selected = _select_reachable_mechanic(
        car,
        contact,
        sample_time,
        arrival_time,
        config,
    )

    if selected is None:
        return None

    shot_type, estimated_boost, reason = selected

    margin = sample_time - arrival_time

    if margin < config.safety_margin:
        return None

    car_to_contact = car.location.flat_direction_to(contact)

    alignment = car.orientation.forward.flatten().flat_normalized().dot(
        car_to_contact
    )

    alignment = clamp(alignment, -1.0, 1.0)

    goal_progress = ball_position.flat_distance(goal_position)

    score = (
        -sample_time * config.time_weight
        + margin * config.margin_weight
        + alignment * config.alignment_weight
        - goal_progress * config.goal_progress_weight
        - estimated_boost * config.boost_cost_weight
    )

    if alignment < 0:
        score -= config.behind_car_penalty

    return ShotCandidate(
        shot_type=shot_type,
        ball_position=ball_position,
        ball_velocity=ball_velocity,
        contact_position=contact,
        shot_direction=direction,
        contact_time=sample_time,
        arrival_time=arrival_time,
        arrival_margin=margin,
        alignment=alignment,
        goal_progress=goal_progress,
        score=score,
        reachable=True,
        estimated_boost=estimated_boost,
        reason=reason,
    )


def find_shot(
    car: VehicleState,
    trajectory: Trajectory,
    team: int,
    config: ShotSelectionConfig | None = None,
) -> ShotPlan:
    """Find and rank reachable future shot contacts."""

    config = config or ShotSelectionConfig()

    if car.demolished:
        return ShotPlan(reason="car demolished")

    goal = enemy_goal_for_team(team)
    goal_position = goal.center

    samples = _trajectory_samples(trajectory, config)

    if not samples:
        return ShotPlan(reason="no usable future ball samples")

    candidates: list[ShotCandidate] = []

    for time, ball_position, ball_velocity in samples:
        candidate = _calculate_candidate(
            car,
            time,
            ball_position,
            ball_velocity,
            goal_position,
            config,
        )

        if candidate is not None:
            candidates.append(candidate)

    if not candidates:
        return ShotPlan(
            reason="no reachable contact with sufficient time and height margin"
        )

    candidates.sort(
        key=lambda candidate: candidate.score,
        reverse=True,
    )

    best = candidates[0]

    return ShotPlan(
        best=best,
        candidates=candidates,
        valid=True,
        shot_type=best.shot_type,
        target=best.ball_position,
        contact_position=best.contact_position,
        contact_time=best.contact_time,
        reason=best.reason,
    )


def find_best_shot(
    car: VehicleState,
    trajectory: Trajectory,
    team: int,
    config: ShotSelectionConfig | None = None,
) -> ShotCandidate | None:
    """Return only the best shot candidate."""

    return find_shot(
        car,
        trajectory,
        team,
        config,
    ).best
