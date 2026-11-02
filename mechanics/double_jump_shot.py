
"""Grid double-jump shot mechanic.

Inspired by Party Cannon's DoubleJumpShot and Noob Black's
jump-time and aerial positioning logic.

The action approaches a planned contact point, commits only when
the timing and position are suitable, performs a double jump, and
then steers/boosts toward the contact point.

The action is non-interruptible after the first jump begins.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from mechanics.arrive import Arrive
from physics.math3d import Vec3, clamp
from physics.vehicle import VehicleState


JUMP_HOLD_TIME = 0.20
RELEASE_FRAMES = 3
SECOND_JUMP_FRAMES = 3


class DoubleJumpShotPhase(Enum):
    IDLE = auto()
    APPROACH = auto()
    FIRST_JUMP = auto()
    RELEASE = auto()
    SECOND_JUMP = auto()
    FLIGHT = auto()
    COMPLETE = auto()
    ABORTED = auto()


@dataclass
class DoubleJumpShotConfig:
    minimum_height: float = 300.0
    maximum_height: float = 510.0
    contact_tolerance: float = 45.0
    update_interval: float = 0.20
    timing_tolerance: float = 0.05
    maximum_action_time: float = 4.0
    abort_after_intercept: float = 0.15
    jump_hold_time: float = JUMP_HOLD_TIME
    release_frames: int = RELEASE_FRAMES
    second_jump_frames: int = SECOND_JUMP_FRAMES
    boost_alignment: float = 0.75
    throttle_alignment: float = 0.50


@dataclass
class DoubleJumpShotOutput:
    phase: DoubleJumpShotPhase = DoubleJumpShotPhase.IDLE
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


class DoubleJumpShot:
    def __init__(
        self,
        config: DoubleJumpShotConfig | None = None,
    ):
        self.config = config or DoubleJumpShotConfig()

        self.phase = DoubleJumpShotPhase.IDLE
        self.reason = ""

        self.target_location: Vec3 | None = None
        self.shot_direction: Vec3 | None = None

        self.intercept_time = 0.0
        self.elapsed = 0.0
        self.phase_elapsed = 0.0
        self.refresh_elapsed = 0.0
        self.jump_elapsed = 0.0

        self.release_frames = 0
        self.second_jump_frames = 0

        self.start_boost = 0.0
        self.left_ground = False
        self.arrive = Arrive()

    @property
    def active(self) -> bool:
        return self.phase in {
            DoubleJumpShotPhase.APPROACH,
            DoubleJumpShotPhase.FIRST_JUMP,
            DoubleJumpShotPhase.RELEASE,
            DoubleJumpShotPhase.SECOND_JUMP,
            DoubleJumpShotPhase.FLIGHT,
        }

    @property
    def interruptible(self) -> bool:
        return self.phase == DoubleJumpShotPhase.APPROACH

    @property
    def finished(self) -> bool:
        return self.phase in {
            DoubleJumpShotPhase.COMPLETE,
            DoubleJumpShotPhase.ABORTED,
        }

    def start(
        self,
        car: VehicleState,
        target_location: Vec3,
        shot_direction: Vec3,
        intercept_time: float,
    ) -> bool:
        """Start a double-jump shot.

        target_location is the desired car contact position.
        intercept_time is the time remaining until the planned contact.
        """

        if not car.on_ground:
            self.reason = "double-jump shot must start on the ground"
            return False

        if intercept_time <= 0:
            self.reason = "invalid intercept time"
            return False

        direction = shot_direction.flatten().normalized()
        if direction.magnitude < 0.01:
            self.reason = "invalid shot direction"
            return False

        height = target_location.z - car.location.z
        if not (
            self.config.minimum_height
            <= height
            <= self.config.maximum_height
        ):
            self.reason = (
                f"target height {height:.1f} outside double-jump range"
            )
            return False

        self.target_location = Vec3(
            target_location.x,
            target_location.y,
            target_location.z,
        )
        self.shot_direction = direction

        self.intercept_time = intercept_time
        self.elapsed = 0.0
        self.phase_elapsed = 0.0
        self.refresh_elapsed = 0.0
        self.jump_elapsed = 0.0

        self.release_frames = 0
        self.second_jump_frames = 0

        self.start_boost = car.boost
        self.left_ground = False

        if not self.arrive.start(
            car,
            self.target_location.flatten(),
            direction,
            intercept_time=intercept_time,
        ):
            self.phase = DoubleJumpShotPhase.ABORTED
            self.reason = "could not start arrival action"
            return False

        self.phase = DoubleJumpShotPhase.APPROACH
        self.reason = "approaching double-jump contact point"
        return True

    def set_intercept_time(self, intercept_time: float) -> None:
        if (
            self.phase == DoubleJumpShotPhase.APPROACH
            and intercept_time > 0
        ):
            self.intercept_time = intercept_time

    def _abort(self, reason: str) -> DoubleJumpShotOutput:
        self.phase = DoubleJumpShotPhase.ABORTED
        self.reason = reason

        return DoubleJumpShotOutput(
            phase=self.phase,
            active=False,
            interruptible=True,
            reason=reason,
        )

    def _time_remaining(self, current_time: float) -> float:
        return self.intercept_time - current_time

    def _jump_time(self, height: float) -> float:
        """Conservative double-jump timing estimate.

        The estimate includes the first jump hold, release window,
        and second jump impulse. It is intentionally conservative;
        the live trajectory/reachability solver will refine this.
        """
        height = max(0.0, height)
        if height <= 0:
            return 0.0

        # Approximate vertical reach using the Grid jump model:
        # first jump impulse + held acceleration + second jump impulse.
        first_jump = 291.67
        second_jump = 291.67
        gravity = 650.0
        hold = self.config.jump_hold_time
        release = self.config.release_frames / 120.0
        second_press = self.config.second_jump_frames / 120.0

        velocity_after_first = first_jump + (1458.33 - gravity) * hold
        height_after_first = (
            first_jump * hold
            + 0.5 * (1458.33 - gravity) * hold * hold
        )

        height_after_release = (
            height_after_first
            + velocity_after_first * release
            - 0.5 * gravity * release * release
        )

        velocity_before_second = velocity_after_first - gravity * release
        velocity_after_second = velocity_before_second + second_jump

        height_after_second = (
            height_after_release
            + velocity_after_second * second_press
            - 0.5 * gravity * second_press * second_press
        )

        if height <= height_after_first:
            return hold

        if height <= height_after_second:
            return hold + release + second_press

        remaining_height = height - height_after_second
        if velocity_after_second <= 0:
            return hold + release + second_press + 1.0

        # Approximate time after the second jump to reach the remaining height.
        discriminant = (
            velocity_after_second * velocity_after_second
            - 2.0 * gravity * remaining_height
        )
        if discriminant < 0:
            return hold + release + second_press + 1.0

        extra_time = (
            velocity_after_second - discriminant ** 0.5
        ) / gravity

        return hold + release + second_press + extra_time

    def _dodge_orient_controls(
        self,
        car: VehicleState,
    ) -> tuple[float, float]:
        if self.shot_direction is None:
            return 0.0, 0.0

        local = car.orientation.world_to_local(
            self.shot_direction
        ).normalized()

        pitch = clamp(-local.x, -1.0, 1.0)
        yaw = clamp(local.y, -1.0, 1.0)
        return pitch, yaw

    def _approach_output(self, output) -> DoubleJumpShotOutput:
        return DoubleJumpShotOutput(
            phase=self.phase,
            throttle=output.throttle,
            steer=output.steer,
            boost=output.boost,
            handbrake=getattr(output, "handbrake", False),
            active=True,
            interruptible=True,
            reason=self.reason,
        )

    def update(
        self,
        car: VehicleState,
        delta_time: float = 1.0 / 120.0,
        current_time: float | None = None,
        opponent_touched_ball: bool = False,
    ) -> DoubleJumpShotOutput:
        if not self.active:
            return DoubleJumpShotOutput(
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
            return self._abort("double-jump shot timed out")

        if opponent_touched_ball:
            return self._abort("opponent touched the ball")

        if time_remaining < -self.config.abort_after_intercept:
            return self._abort("intercept time passed")

        if self.phase == DoubleJumpShotPhase.APPROACH:
            if not car.on_ground:
                self.left_ground = True

            if self.left_ground and car.on_ground:
                return self._abort("car left ground and landed before jumping")

            if car.boost > self.start_boost + 0.5:
                return self._abort("boost increased; shot may have been replaced")

            if self.target_location is None:
                return self._abort("missing contact target")

            height = max(
                0.0,
                self.target_location.z - car.location.z,
            )

            if not (
                self.config.minimum_height
                <= height
                <= self.config.maximum_height
            ):
                return self._abort("target height no longer valid")

            jump_time = self._jump_time(height)
            arrival_output = self.arrive.update(car)
            arrival_eta = getattr(
                self.arrive,
                "eta",
                time_remaining,
            )

            if time_remaining < jump_time - self.config.timing_tolerance:
                return self._abort("not enough time for double jump")

            if arrival_eta > max(
                time_remaining * 1.05,
                time_remaining + 0.025,
            ):
                return self._abort("cannot reach double-jump contact point")

            if (
                arrival_eta < time_remaining - 0.25
                and self.refresh_elapsed > self.config.update_interval
            ):
                return self._abort("arriving too early for selected intercept")

            distance = (
                self.target_location - car.location
            ).flatten().magnitude

            # Party Cannon checks predicted double-jump position here.
            # Until the live vehicle predictor is connected, require both
            # close horizontal alignment and a viable jump window.
            if (
                distance < self.config.contact_tolerance
                and time_remaining <= jump_time + self.config.timing_tolerance
                and arrival_eta <= time_remaining + self.config.timing_tolerance
            ):
                self.phase = DoubleJumpShotPhase.FIRST_JUMP
                self.phase_elapsed = 0.0
                self.jump_elapsed = 0.0
                self.reason = "committed to double jump"

                return DoubleJumpShotOutput(
                    phase=self.phase,
                    throttle=1.0,
                    jump=True,
                    active=True,
                    interruptible=False,
                    reason=self.reason,
                )

            return self._approach_output(arrival_output)

        if self.phase == DoubleJumpShotPhase.FIRST_JUMP:
            self.jump_elapsed += dt

            if self.jump_elapsed >= self.config.jump_hold_time:
                self.phase = DoubleJumpShotPhase.RELEASE
                self.phase_elapsed = 0.0
                self.release_frames = 0

            return DoubleJumpShotOutput(
                phase=self.phase,
                throttle=1.0,
                jump=True,
                boost=car.boost > 0,
                active=True,
                interruptible=False,
                reason="holding first jump",
            )

        if self.phase == DoubleJumpShotPhase.RELEASE:
            self.release_frames += 1

            if self.release_frames >= self.config.release_frames:
                self.phase = DoubleJumpShotPhase.SECOND_JUMP
                self.phase_elapsed = 0.0
                self.second_jump_frames = 0

            return DoubleJumpShotOutput(
                phase=self.phase,
                jump=False,
                active=True,
                interruptible=False,
                reason="releasing jump before second jump",
            )

        if self.phase == DoubleJumpShotPhase.SECOND_JUMP:
            self.second_jump_frames += 1

            if self.second_jump_frames >= self.config.second_jump_frames:
                self.phase = DoubleJumpShotPhase.FLIGHT
                self.phase_elapsed = 0.0

            return DoubleJumpShotOutput(
                phase=self.phase,
                throttle=1.0,
                jump=True,
                active=True,
                interruptible=False,
                reason="performing second jump",
            )

        if self.phase == DoubleJumpShotPhase.FLIGHT:
            if self.target_location is None or self.shot_direction is None:
                return self._abort("missing aerial target")

            offset = self.target_location - car.location
            distance = offset.magnitude

            if distance < 55.0:
                self.phase = DoubleJumpShotPhase.COMPLETE
                self.reason = "double-jump contact reached"

                return DoubleJumpShotOutput(
                    phase=self.phase,
                    active=False,
                    interruptible=True,
                    reason=self.reason,
                )

            forward = car.orientation.forward.normalized()
            offset_direction = offset.normalized()
            alignment = offset_direction.dot(forward)

            boost = (
                car.boost > 0
                and alignment > self.config.boost_alignment
            )
            throttle = (
                1.0
                if alignment > self.config.throttle_alignment
                else 0.0
            )

            pitch, yaw = self._dodge_orient_controls(car)

            return DoubleJumpShotOutput(
                phase=self.phase,
                throttle=throttle,
                boost=boost,
                jump=False,
                pitch=pitch,
                yaw=yaw,
                active=True,
                interruptible=False,
                reason="correcting toward double-jump contact",
            )

        return self._abort("unexpected double-jump phase")

    def cancel(self, reason: str = "cancelled") -> None:
        if self.phase != DoubleJumpShotPhase.APPROACH:
            return

        self.phase = DoubleJumpShotPhase.ABORTED
        self.reason = reason
