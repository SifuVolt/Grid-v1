
"""Grid jump-shot mechanics.

Persistent single-jump shot action inspired by Party Cannon and VirxERLU.

The action:
- Approaches a calculated contact point using Arrive.
- Rechecks its timing while approaching.
- Commits once the jump begins.
- Holds jump, releases for three frames, then dodges.
- Does not allow ordinary actions to interrupt the committed jump.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from math import sqrt

from mechanics.arrive import Arrive
from physics.math3d import Vec3, clamp
from physics.vehicle import VehicleState


JUMP_HOLD_TIME = 0.20
JUMP_RELEASE_FRAMES = 3
JUMP_SPEED = 291.67
JUMP_ACCELERATION = 1458.33
GRAVITY = 650.0


class JumpShotPhase(Enum):
    IDLE = auto()
    APPROACH = auto()
    JUMP = auto()
    RELEASE = auto()
    DODGE = auto()
    COMPLETE = auto()
    ABORTED = auto()


@dataclass
class JumpShotConfig:
    contact_offset: float = 125.0
    maximum_contact_height: float = 270.0
    minimum_contact_height: float = 40.0
    target_tolerance: float = 65.0
    update_interval: float = 0.20
    timing_tolerance: float = 0.05
    early_arrival_tolerance: float = 0.25
    dodge_time: float = 0.05
    jump_hold_time: float = JUMP_HOLD_TIME
    release_frames: int = JUMP_RELEASE_FRAMES
    maximum_action_time: float = 4.0
    abort_after_intercept: float = 0.15


@dataclass
class JumpShotOutput:
    phase: JumpShotPhase = JumpShotPhase.IDLE
    throttle: float = 0.0
    steer: float = 0.0
    boost: bool = False
    handbrake: bool = False
    jump: bool = False
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0
    active: bool = False
    interruptible: bool = True
    reason: str = ""


class JumpShot:
    def __init__(self, config: JumpShotConfig | None = None):
        self.config = config or JumpShotConfig()

        self.phase = JumpShotPhase.IDLE
        self.reason = ""

        self.target_location: Vec3 | None = None
        self.shot_direction: Vec3 | None = None
        self.dodge_direction: Vec3 | None = None

        self.intercept_time = 0.0
        self.elapsed = 0.0
        self.phase_elapsed = 0.0
        self.refresh_elapsed = 0.0
        self.jump_elapsed = 0.0

        self.release_frames = 0
        self.start_boost = 0.0
        self.initialized = False
        self.left_ground = False

        self.arrive = Arrive()

    @property
    def active(self) -> bool:
        return self.phase in {
            JumpShotPhase.APPROACH,
            JumpShotPhase.JUMP,
            JumpShotPhase.RELEASE,
            JumpShotPhase.DODGE,
        }

    @property
    def interruptible(self) -> bool:
        return self.phase == JumpShotPhase.APPROACH

    @property
    def finished(self) -> bool:
        return self.phase in {
            JumpShotPhase.COMPLETE,
            JumpShotPhase.ABORTED,
        }

    def start(
        self,
        car: VehicleState,
        target_location: Vec3,
        shot_direction: Vec3,
        intercept_time: float,
    ) -> bool:
        """Start a jump shot using an absolute intercept time in seconds."""

        if not car.on_ground:
            self.reason = "jump shot must start on the ground"
            return False

        if intercept_time <= 0:
            self.reason = "invalid intercept time"
            return False

        direction = shot_direction.flatten().normalized()
        if direction.magnitude < 0.01:
            self.reason = "invalid shot direction"
            return False

        self.target_location = Vec3(
            target_location.x,
            target_location.y,
            car.location.z,
        )
        self.shot_direction = direction
        self.dodge_direction = direction
        self.intercept_time = intercept_time

        self.elapsed = 0.0
        self.phase_elapsed = 0.0
        self.refresh_elapsed = 0.0
        self.jump_elapsed = 0.0
        self.release_frames = 0
        self.start_boost = car.boost
        self.left_ground = False
        self.initialized = True

        if not self.arrive.start(
            car,
            self.target_location,
            direction,
            intercept_time=intercept_time,
        ):
            self.phase = JumpShotPhase.ABORTED
            self.reason = "could not start arrival action"
            return False

        self.phase = JumpShotPhase.APPROACH
        self.reason = "approaching jump-shot contact point"
        return True

    def set_intercept_time(self, intercept_time: float) -> None:
        """Update the absolute intercept time without resetting the action."""

        if self.phase != JumpShotPhase.APPROACH:
            return

        if intercept_time > 0:
            self.intercept_time = intercept_time

    def _time_remaining(self, current_time: float) -> float:
        return self.intercept_time - current_time

    def _jump_time(self, height: float) -> float:
        """Approximate time needed to reach a height with a held first jump."""

        height = max(0.0, height)
        if height <= 0:
            return 0.0

        # The first jump has an initial impulse and variable jump acceleration.
        # Solve the vertical displacement during the held-jump interval.
        discriminant = JUMP_SPEED ** 2 + 2.0 * (
            JUMP_ACCELERATION - GRAVITY
        ) * height

        if discriminant < 0:
            return self.config.jump_hold_time

        time_to_height = (
            -JUMP_SPEED + sqrt(discriminant)
        ) / (JUMP_ACCELERATION - GRAVITY)

        return clamp(time_to_height, 0.0, self.config.jump_hold_time)

    def _contact_height(self, car: VehicleState) -> float:
        if self.target_location is None:
            return 0.0

        return max(
            0.0,
            self.target_location.z - car.location.z,
        )

    def _ready_to_jump(
        self,
        car: VehicleState,
        time_remaining: float,
        jump_time: float,
        arrival_eta: float,
    ) -> bool:
        if self.target_location is None:
            return False

        distance = (
            self.target_location - car.location
        ).flatten().magnitude

        # The car should be near the contact point and have enough time
        # to complete the jump.
        if distance > self.config.target_tolerance:
            return False

        if time_remaining > jump_time + self.config.timing_tolerance:
            return False

        if arrival_eta > time_remaining + self.config.timing_tolerance:
            return False

        return True

    def _abort(self, reason: str) -> JumpShotOutput:
        self.phase = JumpShotPhase.ABORTED
        self.reason = reason

        return JumpShotOutput(
            phase=self.phase,
            active=False,
            interruptible=True,
            reason=reason,
        )

    def _output_from_arrive(self, output) -> JumpShotOutput:
        return JumpShotOutput(
            phase=self.phase,
            throttle=output.throttle,
            steer=output.steer,
            boost=output.boost,
            handbrake=getattr(output, "handbrake", False),
            active=True,
            interruptible=True,
            reason=self.reason,
        )

    def _dodge_controls(self, car: VehicleState) -> tuple[float, float]:
        """Convert the world-space dodge direction into local pitch/yaw."""

        if self.dodge_direction is None:
            return -1.0, 0.0

        local = car.orientation.world_to_local(
            self.dodge_direction
        ).normalized()

        # Rocket League uses negative pitch for a forward dodge.
        pitch = clamp(-local.x, -1.0, 1.0)
        yaw = clamp(local.y, -1.0, 1.0)

        if abs(pitch) < 0.15 and abs(yaw) < 0.15:
            pitch = -1.0

        return pitch, yaw

    def update(
        self,
        car: VehicleState,
        delta_time: float = 1.0 / 120.0,
        current_time: float | None = None,
        opponent_touched_ball: bool = False,
    ) -> JumpShotOutput:
        if not self.active:
            return JumpShotOutput(
                phase=self.phase,
                active=False,
                interruptible=True,
                reason=self.reason,
            )

        dt = max(0.0, delta_time)
        self.elapsed += dt
        self.phase_elapsed += dt
        self.refresh_elapsed += dt

        if current_time is None:
            current_time = self.elapsed

        time_remaining = self._time_remaining(current_time)

        if self.elapsed > self.config.maximum_action_time:
            return self._abort("jump shot action timed out")

        if opponent_touched_ball:
            return self._abort("opponent touched the ball")

        if time_remaining < -self.config.abort_after_intercept:
            return self._abort("intercept time passed")

        if self.phase == JumpShotPhase.APPROACH:
            if not car.on_ground:
                self.left_ground = True

            if self.left_ground and car.on_ground:
                return self._abort("car left ground and landed before jumping")

            if car.boost > self.start_boost + 0.5:
                return self._abort("boost increased; shot may have been replaced")

            if self.target_location is None:
                return self._abort("missing contact target")

            height = self._contact_height(car)
            if height > self.config.maximum_contact_height:
                return self._abort("contact is too high for a single jump")

            if height < self.config.minimum_contact_height:
                height = self.config.minimum_contact_height

            jump_time = self._jump_time(height)

            arrive_output = self.arrive.update(car)

            arrival_eta = getattr(
                self.arrive,
                "eta",
                time_remaining,
            )

            if (
                time_remaining
                < jump_time - self.config.timing_tolerance
            ):
                return self._abort("not enough time to jump")

            if (
                arrival_eta
                > max(
                    time_remaining * 1.05,
                    time_remaining + 0.025,
                )
            ):
                return self._abort("cannot reach contact point in time")

            if (
                arrival_eta < time_remaining - self.config.early_arrival_tolerance
                and self.refresh_elapsed > self.config.update_interval
            ):
                return self._abort("arriving too early for selected intercept")

            if self._ready_to_jump(
                car,
                time_remaining,
                jump_time,
                arrival_eta,
            ):
                self.phase = JumpShotPhase.JUMP
                self.phase_elapsed = 0.0
                self.jump_elapsed = 0.0
                self.reason = "committed to jump shot"

                return JumpShotOutput(
                    phase=self.phase,
                    throttle=1.0,
                    jump=True,
                    active=True,
                    interruptible=False,
                    reason=self.reason,
                )

            return self._output_from_arrive(arrive_output)

        if self.phase == JumpShotPhase.JUMP:
            self.jump_elapsed += dt

            if self.jump_elapsed >= self.config.jump_hold_time:
                self.phase = JumpShotPhase.RELEASE
                self.phase_elapsed = 0.0
                self.release_frames = 0

            return JumpShotOutput(
                phase=self.phase,
                throttle=1.0,
                boost=car.boost > 0,
                jump=True,
                active=True,
                interruptible=False,
                reason="holding first jump",
            )

        if self.phase == JumpShotPhase.RELEASE:
            self.release_frames += 1

            if self.release_frames >= self.config.release_frames:
                self.phase = JumpShotPhase.DODGE
                self.phase_elapsed = 0.0

            return JumpShotOutput(
                phase=self.phase,
                jump=False,
                active=True,
                interruptible=False,
                reason="releasing jump before dodge",
            )

        if self.phase == JumpShotPhase.DODGE:
            pitch, yaw = self._dodge_controls(car)

            self.phase = JumpShotPhase.COMPLETE
            self.reason = "dodge initiated"

            return JumpShotOutput(
                phase=JumpShotPhase.DODGE,
                throttle=1.0,
                jump=True,
                pitch=pitch,
                yaw=yaw,
                active=True,
                interruptible=False,
                reason=self.reason,
            )

        return self._abort("unexpected jump-shot phase")

    def cancel(self, reason: str = "cancelled") -> None:
        if self.phase in {
            JumpShotPhase.JUMP,
            JumpShotPhase.RELEASE,
            JumpShotPhase.DODGE,
        }:
            return

        self.phase = JumpShotPhase.ABORTED
        self.reason = reason
