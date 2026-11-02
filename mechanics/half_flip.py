
"""Grid half-flip mechanic.

Ported from Party Cannon's RedUtils HalfFlip.cs and AimAt logic.

Sequence:
1. Jump for 0.1 seconds if grounded.
2. Release jump for three frames.
3. Backflip.
4. Cancel the flip and rotate toward the car's velocity direction.
5. Finish after the recovery window.

The action is deliberately persistent. Emergency overrides should be
handled by the agent's action-priority system.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from math import atan2, copysign

from physics.math3d import Vec3, UP, clamp
from physics.vehicle import VehicleState


class HalfFlipPhase(Enum):
    IDLE = auto()
    JUMP = auto()
    RELEASE = auto()
    BACKFLIP = auto()
    CANCEL = auto()
    FINISHED = auto()
    ABORTED = auto()


@dataclass
class HalfFlipConfig:
    jump_hold: float = 0.10
    release_frames: int = 3
    backflip_duration: float = 0.20
    cancel_duration: float = 0.80
    total_duration: float = 1.25

    # Party Cannon AimAt-style PD controller.
    aim_gain: float = 35.0
    aim_rate_limit: float = 1.0

    # Don't try to half-flip when already moving very slowly.
    minimum_speed: float = 100.0

    # Allow a grounded half-flip or an airborne turn-around.
    allow_airborne: bool = True


@dataclass
class HalfFlipOutput:
    throttle: float = 0.0
    steer: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0

    jump: bool = False
    boost: bool = False
    handbrake: bool = False

    phase: HalfFlipPhase = HalfFlipPhase.IDLE
    active: bool = False
    finished: bool = False
    elapsed: float = 0.0
    reason: str = ""


class HalfFlip:
    """Persistent Party Cannon-style half-flip."""

    def __init__(self, config: HalfFlipConfig | None = None):
        self.config = config or HalfFlipConfig()
        self.reset()

    def reset(self) -> None:
        self.phase = HalfFlipPhase.IDLE
        self.active = False
        self.finished = False

        self.elapsed = 0.0
        self.release_ticks = 0
        self.started_grounded = False

        self.start_location: Vec3 | None = None
        self.last_output = HalfFlipOutput()
        self.reason = ""

    def start(self, car: VehicleState) -> bool:
        self.reset()

        if car.demolished:
            self.phase = HalfFlipPhase.ABORTED
            self.finished = True
            self.reason = "car demolished"
            return False

        if not car.on_ground and not self.config.allow_airborne:
            self.phase = HalfFlipPhase.ABORTED
            self.finished = True
            self.reason = "airborne half-flip disabled"
            return False

        if car.speed < self.config.minimum_speed:
            self.phase = HalfFlipPhase.ABORTED
            self.finished = True
            self.reason = "insufficient speed"
            return False

        self.started_grounded = bool(car.on_ground)
        self.start_location = car.location

        self.active = True
        self.finished = False

        self.phase = (
            HalfFlipPhase.JUMP
            if self.started_grounded
            else HalfFlipPhase.BACKFLIP
        )

        self.reason = "half-flip started"
        return True

    @staticmethod
    def _pd(angle: float, rate: float) -> float:
        """Party Cannon-style nonlinear proportional-derivative control."""

        value = 35.0 * (angle + rate)
        return clamp((value ** 3) / 10.0, -1.0, 1.0)

    def _aim_at_velocity(self, car: VehicleState) -> tuple[float, float, float, float]:
        """Aim the car toward its current horizontal velocity.

        This mirrors Party Cannon's:
            AimAt(Me.Location + Me.Velocity.Flatten())

        Its local-axis angular damping is retained, with Grid's Orientation
        and Vec3 types.
        """

        target_direction = car.velocity.flatten()

        if target_direction.length() < 1.0:
            target_direction = car.orientation.forward.flatten()

        if target_direction.length() < 1.0:
            target_direction = Vec3(1.0, 0.0, 0.0)

        local_target = car.orientation.world_to_local(
            target_direction.normalized()
        )

        local_up = car.orientation.world_to_local(UP)

        local_angular = car.orientation.world_to_local(
            car.angular_velocity
        )

        pitch_angle = atan2(local_target.z, local_target.x)
        yaw_angle = atan2(local_target.y, local_target.x)
        roll_angle = atan2(local_up.y, local_up.z)

        pitch = self._pd(
            pitch_angle,
            local_angular.y * 0.2,
        )

        yaw = self._pd(
            yaw_angle,
            -local_angular.z * 0.15,
        )

        roll = self._pd(
            roll_angle,
            local_angular.x * 0.25,
        )

        steer = self._pd(
            yaw_angle,
            -local_angular.z * 0.01,
        )

        return (
            clamp(steer, -1.0, 1.0),
            clamp(pitch, -1.0, 1.0),
            clamp(yaw, -1.0, 1.0),
            clamp(roll, -1.0, 1.0),
        )

    def update(
        self,
        car: VehicleState,
        dt: float = 1.0 / 120.0,
    ) -> HalfFlipOutput:
        dt = max(0.0, float(dt))

        if not self.active or self.finished:
            return self.last_output

        if car.demolished:
            self.active = False
            self.finished = True
            self.phase = HalfFlipPhase.ABORTED
            self.reason = "car demolished during half-flip"

            self.last_output = HalfFlipOutput(
                phase=self.phase,
                finished=True,
                reason=self.reason,
                elapsed=self.elapsed,
            )
            return self.last_output

        previous_elapsed = self.elapsed
        self.elapsed += dt

        output = HalfFlipOutput(
            phase=self.phase,
            active=True,
            elapsed=self.elapsed,
            reason="half-flip active",
        )

        jump_duration = (
            self.config.jump_hold
            if self.started_grounded
            else 0.0
        )

        release_duration = (
            self.config.release_frames * dt
        )

        backflip_start = jump_duration + release_duration
        cancel_start = (
            jump_duration + self.config.backflip_duration
        )

        # Phase 1: initial jump.
        if (
            self.started_grounded
            and self.elapsed <= self.config.jump_hold
        ):
            self.phase = HalfFlipPhase.JUMP
            output.jump = True

        # Phase 2: release jump for three frames.
        elif (
            self.started_grounded
            and self.release_ticks < self.config.release_frames
            and previous_elapsed < backflip_start
        ):
            self.phase = HalfFlipPhase.RELEASE
            output.jump = False
            self.release_ticks += 1

        # Phase 3: backward dodge.
        elif self.elapsed < cancel_start:
            self.phase = HalfFlipPhase.BACKFLIP

            # Party Cannon's HalfFlip.cs uses Pitch = 1 and Jump = true.
            output.pitch = 1.0
            output.jump = True

        # Phase 4: cancel the flip and face the direction of travel.
        elif self.elapsed < self.config.total_duration:
            self.phase = HalfFlipPhase.CANCEL

            (
                output.steer,
                output.pitch,
                output.yaw,
                output.roll,
            ) = self._aim_at_velocity(car)

            output.jump = False

        # Phase 5: finish.
        else:
            self.phase = HalfFlipPhase.FINISHED
            self.active = False
            self.finished = True
            self.reason = "half-flip completed"

            output.phase = self.phase
            output.active = False
            output.finished = True
            output.reason = self.reason

            self.last_output = output
            return output

        output.phase = self.phase
        output.active = True
        output.finished = False

        self.last_output = output
        return output


def create_half_flip(
    config: HalfFlipConfig | None = None,
) -> HalfFlip:
    return HalfFlip(config)


def half_flip(
    car: VehicleState,
    action: HalfFlip | None = None,
    dt: float = 1.0 / 120.0,
) -> HalfFlipOutput:
    """Convenience wrapper for a persistent half-flip action."""

    if action is None:
        action = HalfFlip()

    if not action.active and not action.finished:
        action.start(car)

    return action.update(car, dt)
