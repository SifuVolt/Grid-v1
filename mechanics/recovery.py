
"""Grid recovery mechanics.

Airborne car recovery:
- Rotate the car upright.
- Align the nose with the direction of travel when useful.
- Dampen unwanted angular velocity.
- Prepare the car for a controlled landing.
- Finish once the car has landed and stabilised.

Inspired by the recovery and wavedash principles used by Party Cannon
and other competitive Rocket League bots.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import pi
from typing import Optional

from physics.math3d import Vec3, UP, clamp
from physics.vehicle import VehicleState


class RecoveryPhase(Enum):
    IDLE = "idle"
    ORIENT = "orient"
    LANDING = "landing"
    STABILISE = "stabilise"
    FINISHED = "finished"


@dataclass
class RecoveryConfig:
    # Orientation control.
    upright_gain: float = 4.0
    forward_gain: float = 2.0
    angular_damping: float = 0.9

    # Avoid aggressive steering when already nearly upright.
    upright_deadzone: float = 0.035
    forward_deadzone: float = 0.08

    # Landing behaviour.
    landing_height: float = 180.0
    landing_speed_threshold: float = 250.0
    stable_angular_speed: float = 1.0
    stable_frames_required: int = 4

    # Safety.
    timeout: float = 3.5
    minimum_airborne_time: float = 0.08


@dataclass
class RecoveryOutput:
    throttle: float = 0.0
    steer: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0
    jump: bool = False
    boost: bool = False
    handbrake: bool = False

    phase: RecoveryPhase = RecoveryPhase.IDLE
    finished: bool = False
    active: bool = False


class Recovery:
    """Persistent airborne recovery controller."""

    def __init__(
        self,
        config: Optional[RecoveryConfig] = None,
    ):
        self.config = config or RecoveryConfig()

        self.phase = RecoveryPhase.IDLE
        self.elapsed = 0.0
        self.airborne_elapsed = 0.0
        self.stable_frames = 0

        self.started = False
        self.finished = False

    def start(self) -> None:
        self.phase = RecoveryPhase.ORIENT
        self.elapsed = 0.0
        self.airborne_elapsed = 0.0
        self.stable_frames = 0

        self.started = True
        self.finished = False

    def reset(self) -> None:
        self.phase = RecoveryPhase.IDLE
        self.elapsed = 0.0
        self.airborne_elapsed = 0.0
        self.stable_frames = 0

        self.started = False
        self.finished = False

    @staticmethod
    def _angular_velocity(car: VehicleState) -> Vec3:
        value = getattr(car, "angular_velocity", None)

        if value is None:
            return Vec3(0.0, 0.0, 0.0)

        if isinstance(value, Vec3):
            return value

        try:
            return Vec3.from_value(value)
        except (TypeError, ValueError, AttributeError):
            return Vec3(0.0, 0.0, 0.0)

    @staticmethod
    def _upright_error(car: VehicleState) -> Vec3:
        """World-space rotation axis needed to make the car upright."""
        current_up = car.orientation.up.normalized()
        alignment = current_up.dot(UP)

        # The cross product is zero at exactly 180 degrees.
        # Force a roll around the car's forward axis in that case.
        if alignment < -0.95:
            return car.orientation.forward.normalized() * pi

        return current_up.cross(UP)

    @staticmethod
    def _forward_error(car: VehicleState) -> Vec3:
        """World-space yaw correction toward the direction of travel."""

        flat_velocity = car.velocity.flatten()

        if flat_velocity.length() < 250.0:
            return Vec3(0.0, 0.0, 0.0)

        desired_forward = flat_velocity.normalized()
        current_forward = car.orientation.forward.flatten().normalized()

        if current_forward.length() < 0.001:
            return Vec3(0.0, 0.0, 0.0)

        return current_forward.cross(desired_forward)

    def _control_axes(
        self,
        car: VehicleState,
        error_world: Vec3,
        angular_velocity: Vec3,
        gain: float,
    ) -> tuple[float, float, float]:
        """Convert world-space orientation error into local controls."""

        orientation = car.orientation

        # Local angular velocity components.
        local_pitch_rate = angular_velocity.dot(orientation.right)
        local_yaw_rate = angular_velocity.dot(orientation.up)
        local_roll_rate = angular_velocity.dot(orientation.forward)

        pitch_error = error_world.dot(orientation.right)
        yaw_error = error_world.dot(orientation.up)
        roll_error = error_world.dot(orientation.forward)

        damping = self.config.angular_damping

        pitch = gain * pitch_error - damping * local_pitch_rate
        yaw = gain * yaw_error - damping * local_yaw_rate
        roll = gain * roll_error - damping * local_roll_rate

        return (
            clamp(pitch, -1.0, 1.0),
            clamp(yaw, -1.0, 1.0),
            clamp(roll, -1.0, 1.0),
        )

    def _orientation_controls(
        self,
        car: VehicleState,
    ) -> tuple[float, float, float]:
        angular_velocity = self._angular_velocity(car)

        upright_error = self._upright_error(car)

        if upright_error.length() > self.config.upright_deadzone:
            return self._control_axes(
                car,
                upright_error,
                angular_velocity,
                self.config.upright_gain,
            )

        forward_error = self._forward_error(car)

        if forward_error.length() > self.config.forward_deadzone:
            return self._control_axes(
                car,
                forward_error,
                angular_velocity,
                self.config.forward_gain,
            )

        # Even when aligned, damp remaining rotation.
        return self._control_axes(
            car,
            Vec3(0.0, 0.0, 0.0),
            angular_velocity,
            0.0,
        )

    def update(
        self,
        car: VehicleState,
        dt: float = 1.0 / 120.0,
    ) -> RecoveryOutput:
        dt = max(0.0, min(dt, 0.1))

        if not self.started:
            self.start()

        if self.finished:
            return RecoveryOutput(
                phase=RecoveryPhase.FINISHED,
                finished=True,
                active=False,
            )

        self.elapsed += dt

        if car.on_ground:
            self.phase = RecoveryPhase.STABILISE
            self.stable_frames += 1

            if (
                self.stable_frames >= self.config.stable_frames_required
                or self.elapsed >= self.config.timeout
            ):
                self.phase = RecoveryPhase.FINISHED
                self.finished = True

                return RecoveryOutput(
                    phase=self.phase,
                    finished=True,
                    active=False,
                )

            return RecoveryOutput(
                throttle=0.0,
                steer=0.0,
                phase=self.phase,
                active=True,
            )

        self.stable_frames = 0
        self.airborne_elapsed += dt

        if self.elapsed >= self.config.timeout:
            self.phase = RecoveryPhase.FINISHED
            self.finished = True

            return RecoveryOutput(
                phase=self.phase,
                finished=True,
                active=False,
            )

        # Give an aerial action a brief moment to release its controls.
        if self.airborne_elapsed < self.config.minimum_airborne_time:
            return RecoveryOutput(
                phase=self.phase,
                active=True,
            )

        speed = car.velocity.length()
        height = car.location.z

        # Once close to the ground, prioritise upright landing.
        if height <= self.config.landing_height:
            self.phase = RecoveryPhase.LANDING
        else:
            self.phase = RecoveryPhase.ORIENT

        pitch, yaw, roll = self._orientation_controls(car)

        # Do not waste boost during recovery. A separate aerial action
        # should decide whether additional boost is necessary.
        return RecoveryOutput(
            throttle=0.0,
            steer=0.0,
            pitch=pitch,
            yaw=yaw,
            roll=roll,
            jump=False,
            boost=False,
            handbrake=False,
            phase=self.phase,
            finished=False,
            active=True,
        )


def create_recovery(
    config: Optional[RecoveryConfig] = None,
) -> Recovery:
    return Recovery(config)


def recover_car(
    car: VehicleState,
    recovery: Optional[Recovery] = None,
    dt: float = 1.0 / 120.0,
) -> RecoveryOutput:
    """Convenience wrapper for a single recovery update."""

    if recovery is None:
        recovery = Recovery()
        recovery.start()

    return recovery.update(car, dt)
