
"""Grid speedflip mechanic.

Based on Party Cannon's RedUtils SpeedFlip.cs.

Sequence:
1. Steer toward a slightly offset direction to align velocity.
2. Choose the dodge side based on which side of the target direction
   the car is travelling.
3. Jump for 0.1 seconds.
4. Release jump briefly.
5. Diagonal dodge.
6. Cancel the forward flip component with opposite pitch.
7. Drift-assisted landing correction.

The action is persistent and non-interruptible during execution.
Boost is configurable because Party Cannon leaves boost control to the
parent action.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from math import atan2

from physics.math3d import Vec3, UP, clamp
from physics.vehicle import VehicleState


class SpeedFlipPhase(Enum):
    IDLE = auto()
    ALIGN = auto()
    JUMP = auto()
    RELEASE = auto()
    DODGE = auto()
    CANCEL = auto()
    LANDING = auto()
    FINISHED = auto()
    ABORTED = auto()


@dataclass
class SpeedFlipConfig:
    jump_duration: float = 0.10
    release_duration: float = 0.02
    dodge_duration: float = 0.03
    cancel_end: float = 0.75
    landing_end: float = 0.90

    alignment_tolerance: float = 0.05
    alignment_timeout: float = 1.0
    minimum_speed: float = 250.0

    # Party Cannon deliberately lets a parent action decide when to boost.
    boost_during_flip: bool = False

    # Final landing correction.
    use_landing_drift: bool = True
    landing_steer: float = 0.35

    # Controller gains for AimAt-style alignment.
    aim_gain: float = 35.0
    angular_damping: float = 0.20

    # Avoid starting too close to the field boundary.
    field_length: float = 5120.0
    field_width: float = 4096.0
    wall_margin: float = 150.0


@dataclass
class SpeedFlipOutput:
    throttle: float = 0.0
    steer: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0

    jump: bool = False
    boost: bool = False
    handbrake: bool = False

    phase: SpeedFlipPhase = SpeedFlipPhase.IDLE
    active: bool = False
    finished: bool = False

    target_direction: Vec3 | None = None
    dodge_side: int = 0
    elapsed: float = 0.0
    reason: str = ""


class SpeedFlip:
    """Persistent Party Cannon-style speedflip."""

    def __init__(self, config: SpeedFlipConfig | None = None):
        self.config = config or SpeedFlipConfig()
        self.reset()

    def reset(self) -> None:
        self.phase = SpeedFlipPhase.IDLE
        self.active = False
        self.finished = False

        self.elapsed = 0.0
        self.alignment_elapsed = 0.0
        self.flip_elapsed = 0.0

        self.target_direction: Vec3 | None = None
        self.dodge_side = 0

        self.reason = ""
        self.last_output = SpeedFlipOutput()

    def _abort(self, reason: str) -> SpeedFlipOutput:
        self.phase = SpeedFlipPhase.ABORTED
        self.active = False
        self.finished = True
        self.reason = reason

        output = SpeedFlipOutput(
            phase=self.phase,
            active=False,
            finished=True,
            reason=reason,
            target_direction=self.target_direction,
            dodge_side=self.dodge_side,
            elapsed=self.elapsed,
        )
        self.last_output = output
        return output

    def _inside_field(self, car: VehicleState) -> bool:
        return (
            abs(car.location.x)
            <= self.config.field_width - self.config.wall_margin
            and abs(car.location.y)
            <= self.config.field_length - self.config.wall_margin
        )

    def _turn_radius(self, car: VehicleState) -> float:
        try:
            radius = float(car.turning_radius())
            if radius > 1.0:
                return radius
        except (AttributeError, TypeError, ValueError):
            pass

        # Conservative fallback if VehicleState has no turning-radius method.
        speed = max(0.0, car.flat_speed)
        return max(180.0, 900.0 - speed * 0.25)

    def _alignment_direction(self, car: VehicleState) -> Vec3:
        """Calculate Party Cannon's small pre-flip steering offset."""

        assert self.target_direction is not None

        speed_forward = car.velocity.dot(car.orientation.forward)
        turn_radius = self._turn_radius(car)

        offset_angle = clamp(
            0.06 * speed_forward / turn_radius,
            -0.30,
            0.30,
        )

        left = self.target_direction.rotate_around(UP, offset_angle)
        right = self.target_direction.rotate_around(UP, -offset_angle)

        velocity_direction = car.velocity.flat_normalized()

        if velocity_direction.length() < 0.01:
            velocity_direction = car.orientation.forward.flat_normalized()

        if velocity_direction.angle_to(left) < velocity_direction.angle_to(right):
            self.dodge_side = 1
            return left.flat_normalized()

        self.dodge_side = -1
        return right.flat_normalized()

    def _aim_at(
        self,
        car: VehicleState,
        target_direction: Vec3,
    ) -> tuple[float, float, float, float]:
        """PD orientation controller inspired by Party Cannon AimAt."""

        local_target = car.orientation.world_to_local(
            target_direction.normalized()
        )

        local_up = car.orientation.world_to_local(UP)
        local_angular = car.orientation.world_to_local(car.angular_velocity)

        pitch_angle = atan2(local_target.z, local_target.x)
        yaw_angle = atan2(local_target.y, local_target.x)
        roll_angle = atan2(local_up.y, local_up.z)

        gain = self.config.aim_gain
        damping = self.config.angular_damping

        def control(angle: float, rate: float) -> float:
            value = gain * (angle + damping * rate)
            return clamp((value ** 3) / 10.0, -1.0, 1.0)

        pitch = control(pitch_angle, local_angular.y)
        yaw = control(yaw_angle, -local_angular.z)
        roll = control(roll_angle, local_angular.x)

        steer = control(yaw_angle, -local_angular.z * 0.05)

        return steer, pitch, yaw, roll

    def start(
        self,
        car: VehicleState,
        direction: Vec3,
    ) -> bool:
        self.reset()

        if car.demolished:
            self._abort("car demolished")
            return False

        if not car.on_ground:
            self._abort("speedflip must start on the ground")
            return False

        if car.flat_speed < self.config.minimum_speed:
            self._abort("insufficient speed to speedflip")
            return False

        if not self._inside_field(car):
            self._abort("too close to field boundary")
            return False

        target = direction.flatten().normalized()

        if target.length() < 0.01:
            self._abort("invalid speedflip direction")
            return False

        self.target_direction = target
        self.phase = SpeedFlipPhase.ALIGN
        self.active = True
        self.finished = False

        self.reason = "aligning for speedflip"
        return True

    def update(
        self,
        car: VehicleState,
        dt: float = 1.0 / 120.0,
        boost_allowed: bool | None = None,
    ) -> SpeedFlipOutput:
        dt = max(0.0, float(dt))

        if not self.active or self.finished:
            return self.last_output

        if car.demolished:
            return self._abort("car demolished during speedflip")

        if self.target_direction is None:
            return self._abort("speedflip target direction missing")

        self.elapsed += dt

        output = SpeedFlipOutput(
            throttle=1.0,
            boost=(
                self.config.boost_during_flip
                if boost_allowed is None
                else bool(boost_allowed)
            ),
            phase=self.phase,
            active=True,
            target_direction=self.target_direction,
            dodge_side=self.dodge_side,
            elapsed=self.elapsed,
            reason="speedflip active",
        )

        if self.phase == SpeedFlipPhase.ALIGN:
            self.alignment_elapsed += dt

            if not car.on_ground:
                return self._abort(
                    "left ground before speedflip alignment"
                )

            if not self._inside_field(car):
                return self._abort(
                    "field boundary too close during alignment"
                )

            if self.alignment_elapsed > self.config.alignment_timeout:
                return self._abort("speedflip alignment timed out")

            aligned_direction = self._alignment_direction(car)

            angle = car.velocity.flat_normalized().angle_to(
                aligned_direction
            )

            # Determine the signed error as well. Angle magnitude alone
            # cannot tell which way the car must steer.
            signed_error = car.velocity.flat_normalized().signed_flat_angle_to(
                aligned_direction
            )

            if abs(signed_error) <= self.config.alignment_tolerance:
                self.phase = SpeedFlipPhase.JUMP
                self.flip_elapsed = 0.0
                output.phase = self.phase
                output.jump = True
                output.reason = "alignment complete; jump initiated"
            else:
                output.steer = clamp(signed_error * 3.0, -1.0, 1.0)
                output.jump = False
                output.reason = "aligning velocity with speedflip direction"

            self.last_output = output
            return output

        self.flip_elapsed += dt
        t = self.flip_elapsed

        jump_end = self.config.jump_duration
        release_end = jump_end + self.config.release_duration
        dodge_end = release_end + self.config.dodge_duration

        # Party Cannon: first jump for 0.1 seconds.
        if t < jump_end:
            self.phase = SpeedFlipPhase.JUMP
            output.jump = True
            output.reason = "initial jump"

        # Short release before the dodge.
        elif t < release_end:
            self.phase = SpeedFlipPhase.RELEASE
            output.jump = False
            output.reason = "release jump before dodge"

        # Diagonal dodge: forward pitch plus selected side roll.
        elif t < dodge_end:
            self.phase = SpeedFlipPhase.DODGE
            output.jump = True
            output.pitch = -1.0
            output.roll = self.dodge_side * 0.5
            output.reason = "diagonal dodge"

        # Cancel the forward component of the dodge.
        elif t < self.config.cancel_end:
            self.phase = SpeedFlipPhase.CANCEL
            output.pitch = 1.0
            output.roll = float(self.dodge_side)
            output.jump = False
            output.reason = "canceling flip"

        # Drift-assisted landing correction.
        elif t < self.config.landing_end:
            self.phase = SpeedFlipPhase.LANDING
            output.pitch = 1.0
            output.roll = float(self.dodge_side)
            output.yaw = float(self.dodge_side)

            output.handbrake = self.config.use_landing_drift
            output.steer = (
                self.config.landing_steer * self.dodge_side
            )
            output.jump = False
            output.reason = "landing correction"

        else:
            self.phase = SpeedFlipPhase.FINISHED
            self.active = False
            self.finished = True
            self.reason = "speedflip completed"

            output.phase = self.phase
            output.active = False
            output.finished = True
            output.jump = False
            output.boost = False
            output.reason = self.reason

        output.phase = self.phase
        output.active = self.active
        output.finished = self.finished
        output.dodge_side = self.dodge_side

        self.last_output = output
        return output


def create_speedflip(
    config: SpeedFlipConfig | None = None,
) -> SpeedFlip:
    return SpeedFlip(config)


def speedflip(
    car: VehicleState,
    direction: Vec3,
    action: SpeedFlip | None = None,
    dt: float = 1.0 / 120.0,
) -> SpeedFlipOutput:
    """Convenience wrapper; pass the same action instance every tick."""

    if action is None:
        action = SpeedFlip()

    if not action.active and not action.finished:
        if not action.start(car, direction):
            return action.last_output

    return action.update(car, dt)
