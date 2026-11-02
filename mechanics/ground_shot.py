
"""Grid persistent ground-shot action.

Uses prediction-based interception and a persistent Arrive sub-action.

Design inspired by Party Cannon's JumpShot and Drive:
- Select a future ball contact.
- Aim the contact toward the opponent's goal.
- Approach a point behind the ball.
- Update the target periodically without resetting the drive action.
- Abort when the intercept is no longer viable.
- Keep the action interruptible until contact commitment.

This is the ground-shot layer. Jump and aerial shots are handled separately.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from math import atan2

from mechanics.arrive import Arrive, ArriveConfig, ArrivePhase
from physics.field import BALL_RADIUS, enemy_goal_for_team
from physics.interception import (
    InterceptResult,
    find_best_intercept,
)
from physics.math3d import Vec3, clamp
from physics.vehicle import VehicleState


class GroundShotPhase(Enum):
    IDLE = auto()
    SELECT = auto()
    APPROACH = auto()
    COMMITTED = auto()
    COMPLETE = auto()
    ABORTED = auto()


@dataclass
class GroundShotConfig:
    selection_interval: float = 0.20
    maximum_prediction_time: float = 2.5
    prediction_step: float = 0.10

    maximum_ball_height: float = 170.0
    minimum_intercept_margin: float = -0.05

    contact_offset: float = 125.0
    contact_height: float = 17.0

    arrival_radius: float = 115.0
    target_speed: float = 1900.0

    minimum_shot_speed: float = 450.0
    maximum_shot_angle: float = 1.25

    commitment_distance: float = 260.0
    maximum_action_time: float = 5.0

    boost_allowed: bool = True


@dataclass
class GroundShotOutput:
    phase: GroundShotPhase = GroundShotPhase.IDLE

    throttle: float = 0.0
    steer: float = 0.0
    boost: bool = False
    handbrake: bool = False

    jump: bool = False
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0

    target: Vec3 = Vec3(0.0, 0.0, 0.0)
    ball_position: Vec3 = Vec3(0.0, 0.0, 0.0)
    shot_direction: Vec3 = Vec3(1.0, 0.0, 0.0)

    intercept_time: float = 0.0
    arrival_time: float = 0.0
    margin: float = 0.0

    active: bool = False
    interruptible: bool = True
    reason: str = ""


class GroundShot:
    """Persistent ground-shot action."""

    def __init__(
        self,
        config: GroundShotConfig | None = None,
        arrive_config: ArriveConfig | None = None,
    ):
        self.config = config or GroundShotConfig()
        self.arrive_config = arrive_config or ArriveConfig()

        self.arrive = Arrive(self.arrive_config)

        self.phase = GroundShotPhase.IDLE

        self.intercept: InterceptResult | None = None
        self.ball_position: Vec3 | None = None
        self.ball_velocity: Vec3 | None = None

        self.contact_target: Vec3 | None = None
        self.shot_direction: Vec3 | None = None

        self.elapsed = 0.0
        self.selection_elapsed = 0.0
        self.intercept_time = 0.0
        self.arrival_time = 0.0
        self.margin = 0.0

        self.team = 0
        self.reason = ""

    @property
    def active(self) -> bool:
        return self.phase in (
            GroundShotPhase.SELECT,
            GroundShotPhase.APPROACH,
            GroundShotPhase.COMMITTED,
        )

    @property
    def interruptible(self) -> bool:
        return self.phase != GroundShotPhase.COMMITTED

    def reset(self) -> None:
        self.arrive.reset()

        self.phase = GroundShotPhase.IDLE

        self.intercept = None
        self.ball_position = None
        self.ball_velocity = None

        self.contact_target = None
        self.shot_direction = None

        self.elapsed = 0.0
        self.selection_elapsed = 0.0
        self.intercept_time = 0.0
        self.arrival_time = 0.0
        self.margin = 0.0

        self.reason = ""

    def start(
        self,
        car: VehicleState,
        trajectory,
        team: int = 0,
    ) -> bool:
        """Select an intercept and begin approaching it."""

        if car.demolished:
            self.reason = "car demolished"
            return False

        self.reset()
        self.team = int(team)

        if not self._select_intercept(car, trajectory):
            self.phase = GroundShotPhase.ABORTED
            return False
        if self.contact_target is None or self.shot_direction is None:
            self.phase = GroundShotPhase.ABORTED
            self.reason = "missing initial contact target"
            return False

        if not self.arrive.start(
            car,
            self.contact_target,
            self.shot_direction,
            intercept_time=self.intercept_time,
        ):
            self.phase = GroundShotPhase.ABORTED
            self.reason = "could not start arrive action"
            return False
        self.phase = GroundShotPhase.APPROACH
        self.reason = "ground shot selected"

        return True

    def abort(self, reason: str) -> None:
        self.phase = GroundShotPhase.ABORTED
        self.reason = reason
        self.arrive.abort(reason)

    def finish(self, reason: str) -> None:
        self.phase = GroundShotPhase.COMPLETE
        self.reason = reason

    def _goal_target(self) -> Vec3:
        goal = enemy_goal_for_team(self.team)

        if hasattr(goal, "center"):
            return Vec3.from_value(goal.center)

        if hasattr(goal, "location"):
            return Vec3.from_value(goal.location)

        return Vec3(0.0, 5120.0 if self.team == 0 else -5120.0, 300.0)

    def _calculate_shot_direction(
        self,
        ball_position: Vec3,
        ball_velocity: Vec3,
    ) -> Vec3:
        goal_target = self._goal_target()

        # Aim toward the goal, with a small correction for ball movement.
        to_goal = goal_target - ball_position
        to_goal = Vec3(to_goal.x, to_goal.y, 0.0)

        if to_goal.length() < 1e-6:
            to_goal = Vec3(0.0, 1.0 if self.team == 0 else -1.0, 0.0)

        direction = to_goal.normalized()

        if ball_velocity.flat_magnitude > 100.0:
            velocity_direction = ball_velocity.flatten().normalized()

            # Avoid aiming so sharply that the car cannot make contact.
            direction = (
                direction * 0.85
                + velocity_direction * 0.15
            ).normalized()

        return direction

    def _make_contact_target(
        self,
        car: VehicleState,
        ball_position: Vec3,
        shot_direction: Vec3,
    ) -> Vec3:
        # Position the car behind the ball relative to the intended shot.
        target = ball_position - shot_direction * self.config.contact_offset

        return Vec3(
            target.x,
            target.y,
            self.config.contact_height,
        )

    def _select_intercept(
        self,
        car: VehicleState,
        trajectory,
    ) -> bool:
        result = find_best_intercept(
            car,
            trajectory,
            step=self.config.prediction_step,
            maximum_time=self.config.maximum_prediction_time,
            minimum_margin=self.config.minimum_intercept_margin,
            maximum_height=self.config.maximum_ball_height,
            allow_aerial=False,
        )

        if not result.valid or result.candidate is None:
            self.reason = "no reachable ground contact"
            return False

        candidate = result.candidate

        if candidate.position.z > self.config.maximum_ball_height:
            self.reason = "intercept too high for ground shot"
            return False

        self.intercept = result
        self.ball_position = candidate.position
        self.ball_velocity = candidate.velocity

        self.intercept_time = candidate.time
        self.arrival_time = candidate.arrival_time
        self.margin = candidate.margin

        self.shot_direction = self._calculate_shot_direction(
            candidate.position,
            candidate.velocity,
        )

        self.contact_target = self._make_contact_target(
            car,
            candidate.position,
            self.shot_direction,
        )

        return True

    def _refresh_intercept(
        self,
        car: VehicleState,
        trajectory,
    ) -> bool:
        old_target = self.contact_target

        if not self._select_intercept(car, trajectory):
            return False

        assert self.contact_target is not None
        assert self.shot_direction is not None

        # Preserve the existing Arrive action and update only its target.
        if old_target is not None and self.arrive.active:
            self.arrive.set_target(
                self.contact_target,
                self.shot_direction,
            )
        else:
            if not self.arrive.start(
                car,
                self.contact_target,
                self.shot_direction,
                intercept_time=self.intercept_time,
            ):
                return False

        return True

    def _approach(
        self,
        car: VehicleState,
        dt: float,
        trajectory,
    ) -> GroundShotOutput:
        if self.contact_target is None:
            self.abort("missing contact target")
            return self._inactive_output(car)

        if self.selection_elapsed >= self.config.selection_interval:
            self.selection_elapsed = 0.0

            if not self._refresh_intercept(car, trajectory):
                self.abort("intercept became unreachable")
                return self._inactive_output(car)

        # Commit only when close enough to the intended contact.
        distance_to_ball = (
            self.ball_position - car.location
        ).length() if self.ball_position is not None else float("inf")

        if distance_to_ball <= self.config.commitment_distance:
            self.phase = GroundShotPhase.COMMITTED

        arrive_output = self.arrive.update(car, dt)

        if arrive_output.phase == ArrivePhase.ABORTED:
            self.abort(arrive_output.reason)
            return self._inactive_output(car)

        if arrive_output.phase == ArrivePhase.COMPLETE:
            self.finish("arrived at shot contact position")
            return self._inactive_output(car)

        return GroundShotOutput(
            phase=self.phase,
            throttle=arrive_output.throttle,
            steer=arrive_output.steer,
            boost=(
                arrive_output.boost
                and self.config.boost_allowed
            ),
            handbrake=arrive_output.handbrake,
            target=self.contact_target,
            ball_position=self.ball_position or Vec3(0.0, 0.0, 0.0),
            shot_direction=self.shot_direction or Vec3(1.0, 0.0, 0.0),
            intercept_time=self.intercept_time,
            arrival_time=self.arrival_time,
            margin=self.margin,
            active=True,
            interruptible=self.interruptible,
            reason="approaching ground-shot contact",
        )

    def update(
        self,
        car: VehicleState,
        trajectory,
        dt: float = 1.0 / 120.0,
    ) -> GroundShotOutput:
        """Advance the shot action."""

        if not self.active:
            return self._inactive_output(car)

        dt = max(1e-4, float(dt))

        self.elapsed += dt
        self.selection_elapsed += dt

        if car.demolished:
            self.abort("car demolished")
            return self._inactive_output(car)

        if self.elapsed > self.config.maximum_action_time:
            self.abort("ground shot timed out")
            return self._inactive_output(car)

        if self.phase in (
            GroundShotPhase.APPROACH,
            GroundShotPhase.COMMITTED,
        ):
            return self._approach(car, dt, trajectory)

        return self._inactive_output(car)

    def _inactive_output(
        self,
        car: VehicleState,
    ) -> GroundShotOutput:
        return GroundShotOutput(
            phase=self.phase,
            target=self.contact_target or car.location,
            ball_position=self.ball_position or car.location,
            shot_direction=self.shot_direction or car.orientation.forward,
            intercept_time=self.intercept_time,
            arrival_time=self.arrival_time,
            margin=self.margin,
            active=False,
            interruptible=self.interruptible,
            reason=self.reason,
        )


def find_ground_shot(
    car: VehicleState,
    trajectory,
    team: int = 0,
    config: GroundShotConfig | None = None,
) -> GroundShot:
    """Convenience factory retained for Grid's shot-selection layer."""

    shot = GroundShot(config)

    if not shot.start(car, trajectory, team=team):
        return shot

    return shot
