"""Grid wall-play mechanics.

Finds useful wall-ball opportunities, positions the car for a wall approach,
and transitions into the aerial controller when the car leaves the surface.

This module deliberately separates wall-play selection from wall-driving
physics. It does not assume that ordinary ground steering works on a wall.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from physics.math3d import Vec3, clamp
from physics.vehicle import VehicleState
from physics.trajectory import Trajectory
from mechanics.aerial import Aerial, AerialConfig, AerialOutput
from mechanics.drive import Drive, DriveOutput


FIELD_HALF_LENGTH = 5120.0
FIELD_HALF_WIDTH = 4096.0
WALL_MARGIN = 450.0
WALL_BALL_MARGIN = 1100.0


class WallSide(Enum):
    NONE = auto()
    LEFT = auto()
    RIGHT = auto()
    BLUE_BACK = auto()
    ORANGE_BACK = auto()


class WallPlayPhase(Enum):
    IDLE = auto()
    APPROACH = auto()
    WALL_SETUP = auto()
    LAUNCH = auto()
    AERIAL = auto()
    FINISHED = auto()
    ABORTED = auto()


@dataclass
class WallPlayConfig:
    minimum_ball_height: float = 180.0
    maximum_ball_height: float = 1500.0
    minimum_ball_speed: float = 250.0
    maximum_prediction_time: float = 2.8

    wall_margin: float = WALL_MARGIN
    ball_wall_margin: float = WALL_BALL_MARGIN
    approach_distance: float = 650.0
    setup_distance: float = 500.0

    minimum_boost: float = 8.0
    maximum_action_time: float = 4.0

    aerial: AerialConfig | None = None


@dataclass
class WallBallRead:
    valid: bool = False
    side: WallSide = WallSide.NONE
    position: Vec3 | None = None
    velocity: Vec3 | None = None
    time: float = 0.0
    reason: str = ""


@dataclass
class WallPlayOutput:
    phase: WallPlayPhase = WallPlayPhase.IDLE
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
    wall: WallSide = WallSide.NONE
    ball_time: float = 0.0
    reason: str = ""


def identify_wall(
    position: Vec3,
    margin: float = WALL_MARGIN,
) -> WallSide:
    """Identify the nearest field boundary wall."""

    distances = {
        WallSide.LEFT: abs(position.x + FIELD_HALF_WIDTH),
        WallSide.RIGHT: abs(position.x - FIELD_HALF_WIDTH),
        WallSide.BLUE_BACK: abs(position.y + FIELD_HALF_LENGTH),
        WallSide.ORANGE_BACK: abs(position.y - FIELD_HALF_LENGTH),
    }

    side = min(distances, key=distances.get)
    if distances[side] <= margin:
        return side

    return WallSide.NONE


def wall_normal(side: WallSide) -> Vec3:
    if side == WallSide.LEFT:
        return Vec3(1.0, 0.0, 0.0)
    if side == WallSide.RIGHT:
        return Vec3(-1.0, 0.0, 0.0)
    if side == WallSide.BLUE_BACK:
        return Vec3(0.0, 1.0, 0.0)
    if side == WallSide.ORANGE_BACK:
        return Vec3(0.0, -1.0, 0.0)
    return Vec3(0.0, 0.0, 0.0)


def read_wall_ball(
    trajectory: Trajectory,
    config: WallPlayConfig | None = None,
) -> WallBallRead:
    """Find the earliest useful predicted ball position near a side wall."""

    config = config or WallPlayConfig()

    duration = min(
        float(getattr(trajectory, "duration", 0.0)),
        config.maximum_prediction_time,
    )

    sample_time = 0.1
    while sample_time <= duration:
        position = trajectory.position_at(sample_time)
        velocity = trajectory.velocity_at(sample_time)

        if position is None or velocity is None:
            sample_time += 0.1
            continue

        side = identify_wall(position, config.ball_wall_margin)

        if side != WallSide.NONE:
            if (
                config.minimum_ball_height
                <= position.z
                <= config.maximum_ball_height
                and velocity.length() >= config.minimum_ball_speed
            ):
                return WallBallRead(
                    valid=True,
                    side=side,
                    position=position,
                    velocity=velocity,
                    time=sample_time,
                    reason="predicted ball near wall",
                )

        sample_time += 0.1

    return WallBallRead(
        valid=False,
        reason="no useful wall ball found",
    )


def wall_approach_target(
    ball_position: Vec3,
    side: WallSide,
    distance: float = 650.0,
) -> Vec3:
    """Return a ground-side approach point offset from the wall ball."""

    normal = wall_normal(side)
    target = ball_position - normal * distance

    # Keep the approach point inside the playable field.
    target = Vec3(
        clamp(target.x, -FIELD_HALF_WIDTH + 150.0, FIELD_HALF_WIDTH - 150.0),
        clamp(target.y, -FIELD_HALF_LENGTH + 150.0, FIELD_HALF_LENGTH - 150.0),
        max(17.0, min(ball_position.z, 300.0)),
    )
    return target


class WallPlay:
    """Persistent wall-play action.

    The action can plan and drive to an approach point on the ground.
    Once the car is on the wall, it waits for a suitable launch state rather
    than applying ground steering to a wall orientation.
    """

    def __init__(self, config: WallPlayConfig | None = None):
        self.config = config or WallPlayConfig()
        self.drive = Drive()
        self.aerial = Aerial(self.config.aerial)

        self.phase = WallPlayPhase.IDLE
        self.active = False
        self.finished = False

        self.read = WallBallRead()
        self.approach_target: Vec3 | None = None
        self.launch_target: Vec3 | None = None

        self.elapsed = 0.0
        self.reason = ""
        self._last_output = WallPlayOutput()

    def reset(self) -> None:
        self.drive = Drive()
        self.aerial.reset()

        self.phase = WallPlayPhase.IDLE
        self.active = False
        self.finished = False

        self.read = WallBallRead()
        self.approach_target = None
        self.launch_target = None

        self.elapsed = 0.0
        self.reason = ""
        self._last_output = WallPlayOutput()

    def start(
        self,
        car: VehicleState,
        trajectory: Trajectory,
    ) -> bool:
        self.reset()

        if car.demolished:
            self.reason = "car demolished"
            self.phase = WallPlayPhase.ABORTED
            self.finished = True
            return False

        if car.boost < self.config.minimum_boost:
            self.reason = "insufficient boost"
            self.phase = WallPlayPhase.ABORTED
            self.finished = True
            return False

        self.read = read_wall_ball(trajectory, self.config)

        if not self.read.valid or self.read.position is None:
            self.reason = self.read.reason
            self.phase = WallPlayPhase.ABORTED
            self.finished = True
            return False

        self.approach_target = wall_approach_target(
            self.read.position,
            self.read.side,
            self.config.approach_distance,
        )

        self.launch_target = self.read.position

        self.phase = WallPlayPhase.APPROACH
        self.active = True
        self.finished = False
        self.reason = "wall play selected"
        return True

    def _abort(self, reason: str) -> WallPlayOutput:
        self.phase = WallPlayPhase.ABORTED
        self.active = False
        self.finished = True
        self.reason = reason

        output = WallPlayOutput(
            phase=self.phase,
            active=False,
            finished=True,
            wall=self.read.side,
            ball_time=self.read.time,
            reason=reason,
        )
        self._last_output = output
        return output

    def update(
        self,
        car: VehicleState,
        trajectory: Trajectory | None = None,
        dt: float = 1.0 / 120.0,
    ) -> WallPlayOutput:
        dt = max(0.0, float(dt))

        if not self.active or self.finished:
            return self._last_output

        if car.demolished:
            return self._abort("car demolished")

        self.elapsed += dt

        if self.elapsed >= self.config.maximum_action_time:
            return self._abort("wall play timed out")

        if trajectory is not None and self.phase in (
            WallPlayPhase.APPROACH,
            WallPlayPhase.WALL_SETUP,
        ):
            updated_read = read_wall_ball(trajectory, self.config)
            if updated_read.valid and updated_read.position is not None:
                self.read = updated_read
                self.launch_target = updated_read.position

                self.approach_target = wall_approach_target(
                    updated_read.position,
                    updated_read.side,
                    self.config.approach_distance,
                )

        if self.phase == WallPlayPhase.APPROACH:
            if self.approach_target is None:
                return self._abort("approach target missing")

            # Ground drive is only used before the car reaches the wall.
            if identify_wall(car.location, self.config.wall_margin) != WallSide.NONE:
                self.phase = WallPlayPhase.WALL_SETUP
                self.reason = "car reached wall"
                return self._output(
                    phase=self.phase,
                    reason=self.reason,
                    target=self.launch_target,
                )

            drive_output: DriveOutput = self.drive.update(
                car, self.approach_target, dt
            )

            distance = car.location.flat_distance(self.approach_target)

            if distance <= self.config.setup_distance:
                self.phase = WallPlayPhase.WALL_SETUP
                self.reason = "approach point reached"

            return self._output(
                phase=self.phase,
                throttle=drive_output.throttle,
                steer=drive_output.steer,
                boost=drive_output.boost,
                handbrake=drive_output.handbrake,
                target=self.approach_target,
                reason=self.reason or "approaching wall",
            )

        if self.phase == WallPlayPhase.WALL_SETUP:
            car_wall = identify_wall(car.location, self.config.wall_margin)

            if car_wall == WallSide.NONE:
                # Do not issue ordinary ground controls while the car may be
                # transitioning between floor and wall.
                return self._output(
                    phase=self.phase,
                    target=self.launch_target,
                    reason="waiting for wall contact",
                )

            if self.launch_target is None:
                return self._abort("wall target missing")

            if not car.on_ground:
                self.phase = WallPlayPhase.LAUNCH
                self.reason = "car left wall surface"

            else:
                # Stay committed to the wall setup; don't steer using the
                # ground-only Drive controller.
                return self._output(
                    phase=self.phase,
                    target=self.launch_target,
                    reason="wall setup - awaiting launch",
                )

        if self.phase == WallPlayPhase.LAUNCH:
            if self.launch_target is None:
                return self._abort("launch target missing")

            if not self.aerial.active:
                started = self.aerial.start(
                    car,
                    target=self.launch_target,
                    intercept_time=max(0.2, self.read.time),
                )
                if not started:
                    return self._abort("aerial launch rejected")

            self.phase = WallPlayPhase.AERIAL

        if self.phase == WallPlayPhase.AERIAL:
            aerial_output: AerialOutput = self.aerial.update(
                car, trajectory=trajectory, dt=dt
            )

            if aerial_output.finished:
                self.phase = WallPlayPhase.FINISHED
                self.active = False
                self.finished = True
                self.reason = aerial_output.reason

            return self._output(
                phase=self.phase,
                throttle=aerial_output.throttle,
                steer=aerial_output.steer,
                pitch=aerial_output.pitch,
                yaw=aerial_output.yaw,
                roll=aerial_output.roll,
                jump=aerial_output.jump,
                boost=aerial_output.boost,
                handbrake=aerial_output.handbrake,
                target=aerial_output.target,
                reason=aerial_output.reason,
            )

        return self._abort("unexpected wall-play phase")

    def _output(
        self,
        phase: WallPlayPhase,
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
    ) -> WallPlayOutput:
        output = WallPlayOutput(
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
            wall=self.read.side,
            ball_time=self.read.time,
            reason=reason,
        )
        self._last_output = output
        return output


def create_wall_play(
    config: WallPlayConfig | None = None,
) -> WallPlay:
    return WallPlay(config)