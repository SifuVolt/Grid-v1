
"""Grid jump, dodge and flip mechanics.

Implements persistent jump sequences with explicit timing.

The action owns its jump button timing. It must be retained between frames;
creating a new Dodge object every update will restart the sequence.

Supports:
- Single jump.
- Double jump.
- Directional dodge.
- Frontflip and backflip.
- Diagonal dodge.
- Dodge cancellation.
- Half-flip sequence.

Controller axes use the RLBot convention:
pitch: -1 = nose up, +1 = nose down
yaw: -1 = left, +1 = right
roll: -1 = left, +1 = right
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import atan2, pi

from physics.math3d import Vec3, clamp
from physics.vehicle import VehicleState
from mechanics.drive import DriveOutput, DriveStatus


# ---------------------------------------------------------------------------
# Timing and physics constants
# ---------------------------------------------------------------------------

JUMP_HOLD_TIME = 0.20
DOUBLE_JUMP_RELEASE_TIME = 3.0 / 120.0
DOUBLE_JUMP_PRESS_TIME = 3.0 / 120.0

DODGE_DURATION = 0.65
DODGE_CANCEL_TIME = 0.12

HALF_FLIP_CANCEL_TIME = 0.16
HALF_FLIP_RECOVERY_TIME = 0.42

MIN_DODGE_SPEED = 120.0
LANDING_HEIGHT = 45.0

DEFAULT_DODGE_DIRECTION = Vec3(1.0, 0.0, 0.0)


class DodgeType(str, Enum):
    SINGLE_JUMP = "single_jump"
    DOUBLE_JUMP = "double_jump"
    FRONTFLIP = "frontflip"
    BACKFLIP = "backflip"
    DIRECTIONAL = "directional"
    DIAGONAL = "diagonal"
    HALF_FLIP = "half_flip"


class DodgePhase(str, Enum):
    IDLE = "idle"
    FIRST_JUMP = "first_jump"
    JUMP_RELEASE = "jump_release"
    SECOND_JUMP = "second_jump"
    DODGING = "dodging"
    CANCEL = "cancel"
    RECOVERING = "recovering"
    FINISHED = "finished"
    FAILED = "failed"


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class DodgeOutput:
    throttle: float = 0.0
    steer: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0

    jump: bool = False
    boost: bool = False
    handbrake: bool = False

    phase: DodgePhase = DodgePhase.IDLE
    finished: bool = False

    def to_drive_output(self) -> DriveOutput:
        return DriveOutput(
            throttle=self.throttle,
            steer=self.steer,
            pitch=self.pitch,
            yaw=self.yaw,
            roll=self.roll,
            jump=self.jump,
            boost=self.boost,
            handbrake=self.handbrake,
            status=(
                DriveStatus.ARRIVED
                if self.finished
                else DriveStatus.DRIVING
            ),
        )


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class DodgeConfig:
    dodge_type: DodgeType = DodgeType.DIRECTIONAL

    jump_hold_time: float = JUMP_HOLD_TIME
    release_time: float = DOUBLE_JUMP_RELEASE_TIME
    second_jump_time: float = DOUBLE_JUMP_PRESS_TIME

    dodge_duration: float = DODGE_DURATION
    cancel_time: float = DODGE_CANCEL_TIME

    direction: Vec3 = DEFAULT_DODGE_DIRECTION

    boost_during_jump: bool = False
    boost_during_dodge: bool = False

    cancel_dodge: bool = False
    maintain_throttle: bool = True

    # Half-flip uses a backward dodge followed by a pitch cancel.
    half_flip_cancel_time: float = HALF_FLIP_CANCEL_TIME
    half_flip_recovery_time: float = HALF_FLIP_RECOVERY_TIME


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _flat_angle_to_direction(car: VehicleState, direction: Vec3) -> float:
    direction = direction.flat_normalized()
    forward = car.forward.flat_normalized()

    if direction.length_squared() < 1e-6:
        return 0.0

    if forward.length_squared() < 1e-6:
        return 0.0

    return forward.signed_flat_angle_to(direction)


def _local_dodge_direction(
    car: VehicleState,
    direction: Vec3,
) -> tuple[float, float]:
    """Return local pitch and yaw input for a world-space dodge direction."""
    local = car.orientation.world_to_local(direction)

    flat_length = max(
        1e-6,
        (local.x * local.x + local.y * local.y) ** 0.5,
    )

    forward_component = local.x / flat_length
    side_component = local.y / flat_length

    # Rocket League's dodge pitch convention:
    # negative pitch = forward dodge, positive pitch = backward dodge.
    pitch = -forward_component
    yaw = side_component

    return (
        clamp(pitch, -1.0, 1.0),
        clamp(yaw, -1.0, 1.0),
    )


# ---------------------------------------------------------------------------
# Dodge action
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class Dodge:
    """Persistent jump and dodge sequence."""

    config: DodgeConfig

    phase: DodgePhase = DodgePhase.IDLE
    elapsed: float = 0.0
    phase_elapsed: float = 0.0
    dodge_elapsed: float = 0.0

    started: bool = False
    jump_was_pressed: bool = False
    initial_location: Vec3 | None = None

    def start(self, car: VehicleState) -> None:
        """Begin the jump sequence."""
        self.phase = DodgePhase.FIRST_JUMP
        self.elapsed = 0.0
        self.phase_elapsed = 0.0
        self.dodge_elapsed = 0.0
        self.started = True
        self.jump_was_pressed = False
        self.initial_location = car.location

    def reset(self) -> None:
        self.phase = DodgePhase.IDLE
        self.elapsed = 0.0
        self.phase_elapsed = 0.0
        self.dodge_elapsed = 0.0
        self.started = False
        self.jump_was_pressed = False
        self.initial_location = None

    def is_finished(self) -> bool:
        return self.phase in (
            DodgePhase.FINISHED,
            DodgePhase.FAILED,
        )

    def _change_phase(self, phase: DodgePhase) -> None:
        self.phase = phase
        self.phase_elapsed = 0.0

    def _output(
        self,
        *,
        throttle: float = 0.0,
        steer: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        roll: float = 0.0,
        jump: bool = False,
        boost: bool = False,
        handbrake: bool = False,
        finished: bool = False,
    ) -> DodgeOutput:
        return DodgeOutput(
            throttle=throttle,
            steer=steer,
            pitch=pitch,
            yaw=yaw,
            roll=roll,
            jump=jump,
            boost=boost,
            handbrake=handbrake,
            phase=self.phase,
            finished=finished,
        )

    def update(
        self,
        car: VehicleState,
        dt: float = 1.0 / 120.0,
        *,
        boost: bool | None = None,
        cancel: bool = False,
    ) -> DodgeOutput:
        """Advance the jump/dodge sequence by one frame."""
        dt = clamp(dt, 0.001, 0.1)

        if not self.started:
            self.start(car)

        if self.is_finished():
            return self._output(finished=True)

        self.elapsed += dt
        self.phase_elapsed += dt

        if boost is None:
            boost = False

        if cancel:
            self._change_phase(DodgePhase.FAILED)
            return self._output(finished=True)

        if car.demolished:
            self._change_phase(DodgePhase.FAILED)
            return self._output(finished=True)

        if self.phase == DodgePhase.FIRST_JUMP:
            return self._update_first_jump(car, boost)

        if self.phase == DodgePhase.JUMP_RELEASE:
            return self._update_jump_release()

        if self.phase == DodgePhase.SECOND_JUMP:
            return self._update_second_jump(car, boost)

        if self.phase == DodgePhase.DODGING:
            return self._update_dodge(car, boost)

        if self.phase == DodgePhase.CANCEL:
            return self._update_cancel(car, boost)

        if self.phase == DodgePhase.RECOVERING:
            return self._update_recovery(car, boost)

        self._change_phase(DodgePhase.FAILED)
        return self._output(finished=True)

    def _update_first_jump(
        self,
        car: VehicleState,
        boost: bool,
    ) -> DodgeOutput:
        hold_time = self.config.jump_hold_time

        if self.phase_elapsed < hold_time:
            self.jump_was_pressed = True

            return self._output(
                throttle=1.0 if self.config.maintain_throttle else 0.0,
                jump=True,
                boost=(
                    self.config.boost_during_jump
                    and boost
                ),
            )

        if self.config.dodge_type == DodgeType.SINGLE_JUMP:
            self._change_phase(DodgePhase.FINISHED)
            return self._output(finished=True)

        self._change_phase(DodgePhase.JUMP_RELEASE)
        return self._output(
            throttle=1.0 if self.config.maintain_throttle else 0.0,
            jump=False,
        )

    def _update_jump_release(self) -> DodgeOutput:
        if self.phase_elapsed < self.config.release_time:
            return self._output(
                throttle=1.0 if self.config.maintain_throttle else 0.0,
                jump=False,
            )

        if self.config.dodge_type == DodgeType.DOUBLE_JUMP:
            self._change_phase(DodgePhase.SECOND_JUMP)
        else:
            self._change_phase(DodgePhase.DODGING)

        return self._output(jump=False)

    def _update_second_jump(
        self,
        car: VehicleState,
        boost: bool,
    ) -> DodgeOutput:
        if self.phase_elapsed < self.config.second_jump_time:
            return self._output(
                throttle=1.0 if self.config.maintain_throttle else 0.0,
                jump=True,
                boost=(
                    self.config.boost_during_jump
                    and boost
                ),
            )

        self._change_phase(DodgePhase.FINISHED)
        return self._output(finished=True)

    def _update_dodge(
        self,
        car: VehicleState,
        boost: bool,
    ) -> DodgeOutput:
        self.dodge_elapsed += 1.0 / 120.0

        if self.dodge_elapsed >= self.config.dodge_duration:
            self._change_phase(DodgePhase.FINISHED)
            return self._output(finished=True)

        if self.config.dodge_type == DodgeType.HALF_FLIP:
            # Start with a backward dodge.
            pitch = 1.0
            yaw = 0.0

            if self.dodge_elapsed >= self.config.half_flip_cancel_time:
                self._change_phase(DodgePhase.CANCEL)

            return self._output(
                throttle=0.0,
                pitch=pitch,
                yaw=yaw,
                jump=True,
                boost=(
                    self.config.boost_during_dodge
                    and boost
                ),
            )

        if self.config.dodge_type == DodgeType.FRONTFLIP:
            pitch, yaw = -1.0, 0.0
        elif self.config.dodge_type == DodgeType.BACKFLIP:
            pitch, yaw = 1.0, 0.0
        else:
            pitch, yaw = _local_dodge_direction(
                car,
                self.config.direction,
            )

        if self.config.cancel_dodge:
            if self.dodge_elapsed >= self.config.cancel_time:
                self._change_phase(DodgePhase.CANCEL)

        return self._output(
            throttle=1.0 if self.config.maintain_throttle else 0.0,
            pitch=pitch,
            yaw=yaw,
            jump=True,
            boost=(
                self.config.boost_during_dodge
                and boost
            ),
        )

    def _update_cancel(
        self,
        car: VehicleState,
        boost: bool,
    ) -> DodgeOutput:
        if self.config.dodge_type == DodgeType.HALF_FLIP:
            # Cancel the backward flip and rotate the car back upright.
            pitch = -1.0
            roll = 1.0 if car.roll < 0.0 else -1.0

            if self.phase_elapsed >= self.config.half_flip_recovery_time:
                self._change_phase(DodgePhase.RECOVERING)

            return self._output(
                throttle=-1.0,
                pitch=pitch,
                roll=roll,
                jump=False,
                boost=False,
            )

        # Standard dodge cancel: pull pitch opposite to the dodge.
        pitch = 1.0

        if self.config.dodge_type == DodgeType.BACKFLIP:
            pitch = -1.0

        if self.phase_elapsed >= self.config.cancel_time:
            self._change_phase(DodgePhase.RECOVERING)

        return self._output(
            pitch=pitch,
            jump=False,
            boost=False,
        )

    def _update_recovery(
        self,
        car: VehicleState,
        boost: bool,
    ) -> DodgeOutput:
        if car.on_ground and car.location.z <= LANDING_HEIGHT + 35.0:
            self._change_phase(DodgePhase.FINISHED)
            return self._output(finished=True)

        if self.phase_elapsed >= 0.8:
            self._change_phase(DodgePhase.FINISHED)
            return self._output(finished=True)

        # Keep the car's nose approximately level while recovering.
        return self._output(
            throttle=0.0,
            pitch=0.0,
            yaw=0.0,
            roll=0.0,
            boost=False,
        )


# ---------------------------------------------------------------------------
# Constructors
# ---------------------------------------------------------------------------

def single_jump(
    jump_hold_time: float = JUMP_HOLD_TIME,
    boost: bool = False,
) -> Dodge:
    return Dodge(
        DodgeConfig(
            dodge_type=DodgeType.SINGLE_JUMP,
            jump_hold_time=jump_hold_time,
            boost_during_jump=boost,
        )
    )


def double_jump(
    jump_hold_time: float = JUMP_HOLD_TIME,
    boost: bool = False,
) -> Dodge:
    return Dodge(
        DodgeConfig(
            dodge_type=DodgeType.DOUBLE_JUMP,
            jump_hold_time=jump_hold_time,
            boost_during_jump=boost,
        )
    )


def frontflip(
    cancel: bool = False,
) -> Dodge:
    return Dodge(
        DodgeConfig(
            dodge_type=DodgeType.FRONTFLIP,
            cancel_dodge=cancel,
        )
    )


def backflip(
    cancel: bool = False,
) -> Dodge:
    return Dodge(
        DodgeConfig(
            dodge_type=DodgeType.BACKFLIP,
            cancel_dodge=cancel,
        )
    )


def directional_dodge(
    direction: Vec3,
    cancel: bool = False,
) -> Dodge:
    return Dodge(
        DodgeConfig(
            dodge_type=DodgeType.DIRECTIONAL,
            direction=direction,
            cancel_dodge=cancel,
        )
    )


def diagonal_dodge(
    direction: Vec3,
    cancel: bool = False,
) -> Dodge:
    return Dodge(
        DodgeConfig(
            dodge_type=DodgeType.DIAGONAL,
            direction=direction,
            cancel_dodge=cancel,
        )
    )


def half_flip() -> Dodge:
    return Dodge(
        DodgeConfig(
            dodge_type=DodgeType.HALF_FLIP,
            direction=Vec3(-1.0, 0.0, 0.0),
            cancel_dodge=True,
        )
    )
