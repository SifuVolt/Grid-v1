
"""Grid aerial action.

A persistent aerial routine inspired by Party Cannon's AerialShot:
- Approach before committing to the jump.
- Use a reachability check before launching.
- Hold the first jump for its maximum useful duration.
- Release jump before a possible second jump.
- Use closed-loop aerial orientation control.
- Boost only when the car is sufficiently aligned.
- Commit while airborne rather than constantly switching targets.

This module produces controls. It does not directly access RLBot packets.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from math import atan2

from physics.air_control import (
    AirControlConfig,
    aerial_control,
)
from physics.math3d import Vec3, clamp
from physics.vehicle import VehicleState


class AerialPhase(Enum):
    IDLE = auto()
    APPROACH = auto()
    JUMP = auto()
    RELEASE = auto()
    SECOND_JUMP = auto()
    FLIGHT = auto()
    DODGE = auto()
    FINISHED = auto()
    ABORTED = auto()


@dataclass
class AerialConfig:
    jump_hold_time: float = 0.20
    release_frames: int = 3
    second_jump_frames: int = 3

    approach_distance: float = 250.0
    approach_speed: float = 1500.0
    approach_steer_gain: float = 2.5

    minimum_launch_height: float = 250.0
    minimum_boost: float = 8.0

    boost_alignment_dot: float = 0.82
    boost_distance: float = 250.0
    boost_error_threshold: float = 900.0

    aerial_turn_gain: float = 3.2
    aerial_angular_damping: float = 0.55

    double_jump_enabled: bool = True
    double_jump_min_height: float = 350.0

    dodge_enabled: bool = True
    dodge_distance: float = 150.0
    dodge_time: float = 0.12

    maximum_airborne_time: float = 4.0
    abort_distance: float = 2200.0


@dataclass
class AerialOutput:
    phase: AerialPhase = AerialPhase.IDLE

    throttle: float = 0.0
    steer: float = 0.0

    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0

    jump: bool = False
    boost: bool = False
    handbrake: bool = False

    target: Vec3 = Vec3(0.0, 0.0, 0.0)

    active: bool = False
    committed: bool = False

    reason: str = ""


class Aerial:
    """Persistent aerial action."""

    def __init__(
        self,
        config: AerialConfig | None = None,
        air_config: AirControlConfig | None = None,
    ):
        self.config = config or AerialConfig()
        self.air_config = air_config or AirControlConfig()

        self.phase = AerialPhase.IDLE

        self.target: Vec3 | None = None
        self.start_location: Vec3 | None = None

        self.elapsed = 0.0
        self.phase_elapsed = 0.0
        self.airborne_elapsed = 0.0

        self.release_frames = 0
        self.second_jump_frames = 0

        self.start_boost = 0.0
        self.double_jump = False
        self.committed = False

        self.reason = ""

    @property
    def active(self) -> bool:
        return self.phase not in (
            AerialPhase.IDLE,
            AerialPhase.FINISHED,
            AerialPhase.ABORTED,
        )

    def reset(self) -> None:
        self.phase = AerialPhase.IDLE
        self.target = None
        self.start_location = None

        self.elapsed = 0.0
        self.phase_elapsed = 0.0
        self.airborne_elapsed = 0.0

        self.release_frames = 0
        self.second_jump_frames = 0

        self.start_boost = 0.0
        self.double_jump = False
        self.committed = False

        self.reason = ""

    def start(
        self,
        car: VehicleState,
        target: Vec3,
        *,
        double_jump: bool | None = None,
    ) -> bool:
        """Begin an aerial attempt."""

        target = Vec3.from_value(target)

        if car.demolished:
            self.reason = "car demolished"
            return False

        if car.boost < self.config.minimum_boost:
            self.reason = "insufficient boost"
            return False

        if target.z < self.config.minimum_launch_height:
            self.reason = "target too low for aerial"
            return False

        if (target - car.location).length() < 100.0:
            self.reason = "target too close"
            return False

        self.reset()

        self.target = target
        self.start_location = car.location
        self.start_boost = car.boost

        self.double_jump = (
            self.config.double_jump_enabled
            if double_jump is None
            else bool(double_jump)
        )

        self.phase = AerialPhase.APPROACH
        self.reason = "approaching launch position"

        return True

    def abort(self, reason: str) -> None:
        self.phase = AerialPhase.ABORTED
        self.reason = reason
        self.committed = False

    def finish(self, reason: str) -> None:
        self.phase = AerialPhase.FINISHED
        self.reason = reason

    def _set_phase(self, phase: AerialPhase) -> None:
        self.phase = phase
        self.phase_elapsed = 0.0

    def _target_direction(self, car: VehicleState) -> Vec3:
        if self.target is None:
            return car.orientation.forward

        direction = self.target - car.location

        if direction.length() < 1e-6:
            return car.orientation.forward

        return direction.normalized()

    def _approach(self, car: VehicleState) -> AerialOutput:
        assert self.target is not None

        direction = self._target_direction(car)
        local = car.orientation.world_to_local(direction)

        angle = atan2(local.y, max(0.001, local.x))
        steer = clamp(
            angle * self.config.approach_steer_gain,
            -1.0,
            1.0,
        )

        distance = (self.target - car.location).flat_magnitude

        # Do not launch while facing substantially away from the target.
        aligned = abs(angle) < 0.30

        if not car.on_ground:
            self.committed = True
            self._set_phase(AerialPhase.JUMP)
            return self._jump(car)

        if distance <= self.config.approach_distance and aligned:
            self._set_phase(AerialPhase.JUMP)
            return self._jump(car)

        return AerialOutput(
            phase=self.phase,
            throttle=1.0,
            steer=steer,
            boost=(
                car.boost > self.config.minimum_boost
                and abs(angle) < 0.18
                and car.velocity.length() < self.config.approach_speed
            ),
            target=self.target,
            active=True,
            committed=False,
            reason="approaching aerial launch",
        )

    def _jump(self, car: VehicleState) -> AerialOutput:
        if car.demolished:
            self.abort("demolished during launch")
            return self._inactive_output()

        self.committed = True

        if self.phase_elapsed < self.config.jump_hold_time:
            return AerialOutput(
                phase=self.phase,
                throttle=1.0,
                jump=True,
                target=self.target or car.location,
                active=True,
                committed=True,
                reason="holding first jump",
            )

        if self.double_jump:
            self._set_phase(AerialPhase.RELEASE)
            return self._release_jump(car)

        self._set_phase(AerialPhase.FLIGHT)
        return self._flight(car)

    def _release_jump(self, car: VehicleState) -> AerialOutput:
        self.release_frames += 1

        if self.release_frames >= self.config.release_frames:
            self._set_phase(AerialPhase.SECOND_JUMP)
            return self._second_jump(car)

        return AerialOutput(
            phase=self.phase,
            throttle=1.0,
            jump=False,
            target=self.target or car.location,
            active=True,
            committed=True,
            reason="releasing jump before second jump",
        )

    def _second_jump(self, car: VehicleState) -> AerialOutput:
        self.second_jump_frames += 1

        if self.second_jump_frames >= self.config.second_jump_frames:
            self._set_phase(AerialPhase.FLIGHT)
            return self._flight(car)

        return AerialOutput(
            phase=self.phase,
            throttle=1.0,
            jump=True,
            target=self.target or car.location,
            active=True,
            committed=True,
            reason="second jump impulse",
        )

    def _flight(self, car: VehicleState) -> AerialOutput:
        if self.target is None:
            self.abort("missing aerial target")
            return self._inactive_output()

        direction = self._target_direction(car)
        distance = (self.target - car.location).length()

        if distance > self.config.abort_distance:
            self.abort("target no longer reachable")
            return self._inactive_output()

        if self.airborne_elapsed > self.config.maximum_airborne_time:
            self.finish("maximum aerial time reached")
            return self._inactive_output()

        if car.on_ground and self.airborne_elapsed > 0.15:
            self.finish("landed")
            return self._inactive_output()

        control = aerial_control(
            car,
            direction,
            preferred_up=Vec3(0.0, 0.0, 1.0),
            config=self.air_config,
        )

        alignment = car.orientation.forward.dot(direction)

        boost = (
            car.boost > 0.0
            and distance > self.config.boost_distance
            and alignment >= self.config.boost_alignment_dot
            and distance < self.config.boost_error_threshold
        )

        # A small amount of throttle is useful in the air, but boost is
        # the primary acceleration source during a properly aligned flight.
        throttle = clamp(alignment, -1.0, 1.0)

        if (
            self.config.dodge_enabled
            and distance < self.config.dodge_distance
            and self.airborne_elapsed < 1.45
        ):
            self._set_phase(AerialPhase.DODGE)
            return self._dodge(car, direction)

        return AerialOutput(
            phase=self.phase,
            throttle=throttle,
            pitch=control.pitch,
            yaw=control.yaw,
            roll=control.roll,
            jump=False,
            boost=boost,
            target=self.target,
            active=True,
            committed=True,
            reason="aerial flight",
        )

    def _dodge(
        self,
        car: VehicleState,
        direction: Vec3,
    ) -> AerialOutput:
        local = car.orientation.world_to_local(direction)

        dodge_direction = Vec3(
            1.0,
            clamp(local.y, -1.0, 1.0),
            0.0,
        ).normalized()

        if self.phase_elapsed >= self.config.dodge_time:
            self.finish("aerial dodge completed")
            return self._inactive_output()

        return AerialOutput(
            phase=self.phase,
            throttle=1.0,
            pitch=-dodge_direction.x,
            yaw=dodge_direction.y,
            jump=True,
            boost=False,
            target=self.target or car.location,
            active=True,
            committed=True,
            reason="final aerial dodge",
        )

    def _inactive_output(self) -> AerialOutput:
        return AerialOutput(
            phase=self.phase,
            target=self.target or Vec3(0.0, 0.0, 0.0),
            active=False,
            committed=self.committed,
            reason=self.reason,
        )

    def update(
        self,
        car: VehicleState,
        dt: float = 1.0 / 120.0,
    ) -> AerialOutput:
        """Advance the aerial action by one simulation tick."""

        if not self.active:
            return self._inactive_output()

        dt = max(1e-4, float(dt))

        self.elapsed += dt
        self.phase_elapsed += dt

        if self.phase in (
            AerialPhase.JUMP,
            AerialPhase.RELEASE,
            AerialPhase.SECOND_JUMP,
            AerialPhase.FLIGHT,
            AerialPhase.DODGE,
        ):
            self.airborne_elapsed += dt

        if car.demolished:
            self.abort("car demolished")
            return self._inactive_output()

        if self.phase == AerialPhase.APPROACH:
            return self._approach(car)

        if self.phase == AerialPhase.JUMP:
            return self._jump(car)

        if self.phase == AerialPhase.RELEASE:
            return self._release_jump(car)

        if self.phase == AerialPhase.SECOND_JUMP:
            return self._second_jump(car)

        if self.phase == AerialPhase.FLIGHT:
            return self._flight(car)

        if self.phase == AerialPhase.DODGE:
            return self._dodge(car, self._target_direction(car))

        return self._inactive_output()
