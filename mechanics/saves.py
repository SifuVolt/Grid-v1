"""Grid defensive saves.

Detects dangerous ball trajectories toward the team's own goal and selects
a reachable interception point. Uses ground driving for low saves and the
aerial mechanic for elevated saves.

Team convention:
    0 = blue
    1 = orange
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from physics.math3d import Vec3, clamp
from physics.vehicle import VehicleState
from physics.trajectory import Trajectory
from mechanics.drive import Drive, DriveOutput
from mechanics.aerial import Aerial, AerialConfig, AerialOutput


FIELD_HALF_LENGTH = 5120.0
FIELD_HALF_WIDTH = 4096.0
GOAL_HALF_WIDTH = 892.75
GOAL_HEIGHT = 642.775
BALL_RADIUS = 91.25


class SavePhase(Enum):
    IDLE = auto()
    APPROACH = auto()
    AERIAL = auto()
    CONTACT = auto()
    FINISHED = auto()
    ABORTED = auto()


@dataclass
class SaveConfig:
    danger_depth: float = 1200.0
    emergency_depth: float = 3000.0
    goal_line_margin: float = 700.0

    minimum_inward_speed: float = 180.0
    maximum_prediction_time: float = 3.0
    sample_interval: float = 0.10

    ground_save_height: float = 300.0
    contact_offset: float = 125.0
    ground_target_height: float = 17.0

    minimum_boost_for_aerial: float = 5.0
    maximum_action_time: float = 3.5

    aerial_config: AerialConfig | None = None


@dataclass
class SaveThreat:
    dangerous: bool = False
    emergency: bool = False
    time: float = 0.0
    ball_position: Vec3 | None = None
    ball_velocity: Vec3 | None = None
    goal_distance: float = 0.0
    reason: str = ""


@dataclass
class SaveOutput:
    phase: SavePhase = SavePhase.IDLE
    active: bool = False
    finished: bool = False

    throttle: float = 0.0
    steer: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0
    jump: bool = False
    boost: bool = False
    handbrake: bool = False

    target: Vec3 | None = None
    threat_time: float = 0.0
    emergency: bool = False
    reason: str = ""


def own_goal_y(team: int) -> float:
    return -FIELD_HALF_LENGTH if team == 0 else FIELD_HALF_LENGTH


def distance_to_own_goal(position: Vec3, team: int) -> float:
    return abs(position.y - own_goal_y(team))


def ball_is_in_own_half(position: Vec3, team: int) -> bool:
    return position.y < 0.0 if team == 0 else position.y > 0.0


def ball_is_moving_toward_own_goal(
    velocity: Vec3,
    team: int,
    minimum_speed: float = 180.0,
) -> bool:
    inward = velocity.y if team == 0 else -velocity.y
    return inward < -minimum_speed


def evaluate_save_threat(
    trajectory: Trajectory,
    team: int,
    config: SaveConfig | None = None,
) -> SaveThreat:
    """Find the earliest predicted ball position that threatens the own goal."""

    config = config or SaveConfig()

    duration = min(
        float(getattr(trajectory, "duration", 0.0)),
        config.maximum_prediction_time,
    )

    if duration <= 0.0:
        return SaveThreat(reason="prediction unavailable")

    goal_y = own_goal_y(team)
    inward_sign = 1.0 if team == 0 else -1.0

    best: SaveThreat | None = None
    sample_time = 0.0

    while sample_time <= duration:
        position = trajectory.position_at(sample_time)
        velocity = trajectory.velocity_at(sample_time)

        if position is None or velocity is None:
            sample_time += config.sample_interval
            continue

        depth = distance_to_own_goal(position, team)
        moving_inward = ball_is_moving_toward_own_goal(
            velocity,
            team,
            config.minimum_inward_speed,
        )

        # A ball inside the goal mouth or approaching the goal line is an
        # immediate threat. A ball farther out must be moving inward.
        near_goal_line = depth <= config.goal_line_margin
        in_own_half = ball_is_in_own_half(position, team)

        if near_goal_line and abs(position.x) <= GOAL_HALF_WIDTH + 250.0:
            dangerous = True
        else:
            dangerous = (
                in_own_half
                and depth <= config.danger_depth
                and moving_inward
                and abs(position.x) <= FIELD_HALF_WIDTH
            )

        if dangerous:
            emergency = (
                depth <= config.emergency_depth
                or near_goal_line
                or (
                    sample_time <= 0.75
                    and moving_inward
                )
            )

            best = SaveThreat(
                dangerous=True,
                emergency=emergency,
                time=sample_time,
                ball_position=position,
                ball_velocity=velocity,
                goal_distance=depth,
                reason=(
                    "immediate goal danger"
                    if emergency
                    else "ball approaching own goal"
                ),
            )
            break

        sample_time += config.sample_interval

    return best or SaveThreat(reason="no immediate goal threat")


def save_contact_target(
    ball_position: Vec3,
    team: int,
    offset: float = 125.0,
) -> Vec3:
    """Place the car's contact point on the goal side of the ball."""

    goal_y = own_goal_y(team)

    toward_goal = Vec3(
        -ball_position.x,
        goal_y - ball_position.y,
        0.0,
    ).flat_normalized()

    if toward_goal.length() < 0.01:
        toward_goal = Vec3(0.0, -1.0 if team == 0 else 1.0, 0.0)

    return ball_position + toward_goal * offset


class Save:
    """Persistent defensive save action."""

    def __init__(self, config: SaveConfig | None = None):
        self.config = config or SaveConfig()
        self.drive = Drive()
        self.aerial = Aerial(self.config.aerial_config)

        self.phase = SavePhase.IDLE
        self.active = False
        self.finished = False

        self.threat = SaveThreat()
        self.target: Vec3 | None = None

        self.elapsed = 0.0
        self.reason = ""
        self._last_output = SaveOutput()

    def reset(self) -> None:
        self.drive = Drive()
        self.aerial.reset()

        self.phase = SavePhase.IDLE
        self.active = False
        self.finished = False

        self.threat = SaveThreat()
        self.target = None

        self.elapsed = 0.0
        self.reason = ""
        self._last_output = SaveOutput()

    def start(
        self,
        car: VehicleState,
        trajectory: Trajectory,
        team: int | None = None,
    ) -> bool:
        self.reset()

        selected_team = car.team if team is None else team

        if selected_team not in (0, 1):
            self.reason = "invalid team"
            self.phase = SavePhase.ABORTED
            self.finished = True
            return False

        if car.demolished:
            self.reason = "car demolished"
            self.phase = SavePhase.ABORTED
            self.finished = True
            return False

        self.threat = evaluate_save_threat(
            trajectory,
            selected_team,
            self.config,
        )

        if not self.threat.dangerous or self.threat.ball_position is None:
            self.reason = self.threat.reason
            self.phase = SavePhase.ABORTED
            self.finished = True
            return False

        self.target = save_contact_target(
            self.threat.ball_position,
            selected_team,
            self.config.contact_offset,
        )

        self.active = True
        self.finished = False

        if (
            self.threat.ball_position.z > self.config.ground_save_height
            and car.boost >= self.config.minimum_boost_for_aerial
        ):
            started = self.aerial.start(
                car,
                target=self.target,
            )

            if started:
                self.phase = SavePhase.AERIAL
                self.reason = "aerial save selected"
                return True

        self.phase = SavePhase.APPROACH
        self.reason = "ground save selected"
        return True

    def _abort(self, reason: str) -> SaveOutput:
        self.phase = SavePhase.ABORTED
        self.active = False
        self.finished = True
        self.reason = reason

        output = SaveOutput(
            phase=self.phase,
            active=False,
            finished=True,
            target=self.target,
            threat_time=self.threat.time,
            emergency=self.threat.emergency,
            reason=reason,
        )
        self._last_output = output
        return output

    def update(
        self,
        car: VehicleState,
        trajectory: Trajectory | None = None,
        dt: float = 1.0 / 120.0,
    ) -> SaveOutput:
        dt = max(0.0, float(dt))

        if not self.active or self.finished:
            return self._last_output

        if car.demolished:
            return self._abort("car demolished")

        self.elapsed += dt

        if self.elapsed >= self.config.maximum_action_time:
            return self._abort("save action timed out")

        # Update the target before the aerial commitment. During an aerial,
        # keep the original contact point to avoid abrupt steering changes.
        if (
            trajectory is not None
            and self.phase == SavePhase.APPROACH
        ):
            updated = evaluate_save_threat(
                trajectory,
                car.team,
                self.config,
            )

            if updated.dangerous and updated.ball_position is not None:
                self.threat = updated
                self.target = save_contact_target(
                    updated.ball_position,
                    car.team,
                    self.config.contact_offset,
                )

        if self.target is None:
            return self._abort("save target missing")

        if self.phase == SavePhase.APPROACH:
            if self.threat.ball_position is not None:
                if (
                    self.threat.ball_position.z > self.config.ground_save_height
                    and car.boost >= self.config.minimum_boost_for_aerial
                ):
                    if self.aerial.start(
                        car,
                        target=self.target,
                    ):
                        self.phase = SavePhase.AERIAL
                        self.reason = "transitioning to aerial save"

                        return self._output(
                            phase=self.phase,
                            target=self.target,
                            reason=self.reason,
                        )

            ground_target = self.target.with_z(
                self.config.ground_target_height
            )

            drive_output: DriveOutput = self.drive.update(
                car,
                ground_target,
                dt,
            )

            distance = car.location.flat_distance(ground_target)

            if distance <= 150.0:
                self.phase = SavePhase.CONTACT
                self.reason = "ground save contact range"

            return self._output(
                phase=self.phase,
                throttle=drive_output.throttle,
                steer=drive_output.steer,
                boost=drive_output.boost,
                handbrake=drive_output.handbrake,
                target=ground_target,
                reason=self.reason or "driving to save point",
            )

        if self.phase == SavePhase.AERIAL:
            aerial_output: AerialOutput = self.aerial.update(
                car,
                dt=dt,
            )

            if aerial_output.finished:
                self.phase = SavePhase.FINISHED
                self.active = False
                self.finished = True
                self.reason = aerial_output.reason

            return self._output(
                phase=self.phase,
                throttle=aerial_output.throttle,
                pitch=aerial_output.pitch,
                yaw=aerial_output.yaw,
                roll=aerial_output.roll,
                jump=aerial_output.jump,
                boost=aerial_output.boost,
                target=aerial_output.target,
                reason=aerial_output.reason,
            )

        if self.phase == SavePhase.CONTACT:
            self.phase = SavePhase.FINISHED
            self.active = False
            self.finished = True
            self.reason = "save contact range reached"

            return self._output(
                phase=self.phase,
                target=self.target,
                reason=self.reason,
            )

        return self._abort("unexpected save phase")

    def _output(
        self,
        phase: SavePhase,
        throttle: float = 0.0,
        steer: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        roll: float = 0.0,
        jump: bool = False,
        boost: bool = False,
        handbrake: bool = False,
        target: Vec3 | None = None,
        reason: str = "",
    ) -> SaveOutput:
        output = SaveOutput(
            phase=phase,
            active=self.active,
            finished=self.finished,
            throttle=throttle,
            steer=steer,
            pitch=pitch,
            yaw=yaw,
            roll=roll,
            jump=jump,
            boost=boost,
            handbrake=handbrake,
            target=target,
            threat_time=self.threat.time,
            emergency=self.threat.emergency,
            reason=reason,
        )
        self._last_output = output
        return output


def create_save(config: SaveConfig | None = None) -> Save:
    return Save(config)
