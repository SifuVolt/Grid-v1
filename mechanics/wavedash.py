
"""Grid wavedash mechanic.

Based on Party Cannon's RedUtils/Actions/Wavedash.cs.

The dodge is triggered by landing proximity and downward velocity,
not by a fixed timer. The direction is converted into local car
coordinates before being sent to the controller.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

from physics.math3d import Vec3, clamp
from physics.vehicle import VehicleState


class WavedashPhase(Enum):
    IDLE = auto()
    JUMP = auto()
    FALLING = auto()
    DODGE = auto()
    LANDING = auto()
    FINISHED = auto()
    ABORTED = auto()


@dataclass
class WavedashConfig:
    jump_time: float = 0.05

    # Party Cannon's landing trigger.
    landing_height: float = 40.0
    minimum_downward_speed: float = 100.0

    # Safety and completion.
    timeout: float = 2.0
    minimum_speed: float = 100.0

    # Small upward landing aim offset.
    landing_aim_height: float = 0.2

    # Controller tuning.
    aim_gain: float = 25.0
    angular_damping: float = 0.2


@dataclass
class WavedashOutput:
    throttle: float = 0.0
    steer: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    roll: float = 0.0

    jump: bool = False
    boost: bool = False
    handbrake: bool = False

    phase: WavedashPhase = WavedashPhase.IDLE
    active: bool = False
    finished: bool = False

    dodge_direction: Vec3 | None = None
    elapsed: float = 0.0
    reason: str = ""


class Wavedash:
    """Persistent landing-triggered wavedash action."""

    def __init__(self, config: WavedashConfig | None = None):
        self.config = config or WavedashConfig()
        self.reset()

    def reset(self) -> None:
        self.phase = WavedashPhase.IDLE
        self.active = False
        self.finished = False

        self.elapsed = 0.0
        self.started_grounded = False
        self.has_left_ground = False
        self.dodge_started = False

        self.direction: Vec3 | None = None
        self.dodge_input: Vec3 | None = None

        self.reason = ""
        self.last_output = WavedashOutput()

    def _abort(self, reason: str) -> WavedashOutput:
        self.phase = WavedashPhase.ABORTED
        self.active = False
        self.finished = True
        self.reason = reason

        output = WavedashOutput(
            phase=self.phase,
            active=False,
            finished=True,
            elapsed=self.elapsed,
            reason=reason,
        )
        self.last_output = output
        return output

    def start(
        self,
        car: VehicleState,
        direction: Vec3 | None = None,
    ) -> bool:
        self.reset()

        if car.demolished:
            self._abort("car demolished")
            return False

        if car.speed < self.config.minimum_speed:
            self._abort("insufficient speed")
            return False

        chosen_direction = direction

        if chosen_direction is None or chosen_direction.flat_length() < 1.0:
            chosen_direction = car.velocity.flatten()

        if chosen_direction.flat_length() < 1.0:
            chosen_direction = car.orientation.forward.flatten()

        self.direction = chosen_direction.flat_normalized()

        self.started_grounded = bool(car.on_ground)
        self.has_left_ground = not car.on_ground

        self.active = True
        self.finished = False

        self.phase = (
            WavedashPhase.JUMP
            if self.started_grounded
            else WavedashPhase.FALLING
        )

        self.reason = "wavedash started"
        return True

    def _calculate_dodge_input(self, car: VehicleState) -> Vec3:
        """Convert world direction to Party Cannon-style local inputs."""

        assert self.direction is not None

        local = car.orientation.world_to_local(self.direction)

        # Party Cannon Wavedash.cs:
        # input = (local direction Y, -local direction X)
        return Vec3(
            clamp(local.y, -1.0, 1.0),
            clamp(-local.x, -1.0, 1.0),
            0.0,
        )

    def _aim_at_landing(self, car: VehicleState) -> tuple[float, float, float]:
        """Keep the nose aimed toward the landing direction."""

        assert self.direction is not None

        target = self.direction.flatten().normalized()

        local_target = car.orientation.world_to_local(target)
        local_angular = car.orientation.world_to_local(
            car.angular_velocity
        )

        pitch_error = local_target.z
        yaw_error = local_target.y

        pitch = clamp(
            self.config.aim_gain * pitch_error
            + self.config.angular_damping * local_angular.y,
            -1.0,
            1.0,
        )

        yaw = clamp(
            self.config.aim_gain * yaw_error
            - self.config.angular_damping * local_angular.z,
            -1.0,
            1.0,
        )

        roll = clamp(
            -self.config.angular_damping * local_angular.x,
            -1.0,
            1.0,
        )

        return pitch, yaw, roll

    def update(
        self,
        car: VehicleState,
        dt: float = 1.0 / 120.0,
    ) -> WavedashOutput:
        dt = max(0.0, float(dt))

        if not self.active or self.finished:
            return self.last_output

        if car.demolished:
            return self._abort("car demolished during wavedash")

        self.elapsed += dt

        if self.elapsed > self.config.timeout:
            return self._abort("wavedash timed out")

        if not car.on_ground:
            self.has_left_ground = True

        output = WavedashOutput(
            throttle=1.0,
            phase=self.phase,
            active=True,
            elapsed=self.elapsed,
            dodge_direction=self.direction,
            reason="wavedash active",
        )

        # If we have landed after leaving the ground, the action is complete.
        if car.on_ground and self.has_left_ground:
            self.phase = WavedashPhase.FINISHED
            self.active = False
            self.finished = True
            self.reason = "landed"

            output.phase = self.phase
            output.active = False
            output.finished = True
            output.reason = self.reason

            self.last_output = output
            return output

        # Initial jump, only when starting grounded.
        if (
            self.started_grounded
            and not self.has_left_ground
            and self.elapsed < self.config.jump_time
        ):
            self.phase = WavedashPhase.JUMP
            output.jump = True
            output.reason = "initial jump"

        # Close to the floor and descending: execute the dodge.
        elif (
            not car.on_ground
            and car.location.z < self.config.landing_height
            and car.velocity.z < -self.config.minimum_downward_speed
        ):
            self.phase = WavedashPhase.DODGE

            if self.dodge_input is None:
                self.dodge_input = self._calculate_dodge_input(car)

            output.yaw = self.dodge_input.x
            output.pitch = self.dodge_input.y
            output.jump = True

            self.dodge_started = True
            output.reason = "landing dodge"

        # Still airborne: orient toward the intended landing direction.
        elif not car.on_ground:
            self.phase = (
                WavedashPhase.LANDING
                if self.dodge_started
                else WavedashPhase.FALLING
            )

            (
                output.pitch,
                output.yaw,
                output.roll,
            ) = self._aim_at_landing(car)

            output.reason = "aligning for landing"

        else:
            # Grounded at the beginning of the action. Wait for the
            # initial jump to take effect instead of finishing immediately.
            self.phase = WavedashPhase.JUMP
            output.jump = self.started_grounded
            output.reason = "waiting for takeoff"

        output.phase = self.phase
        output.active = self.active
        output.finished = self.finished

        self.last_output = output
        return output


def create_wavedash(
    config: WavedashConfig | None = None,
) -> Wavedash:
    return Wavedash(config)


def wavedash(
    car: VehicleState,
    direction: Vec3 | None = None,
    action: Wavedash | None = None,
    dt: float = 1.0 / 120.0,
) -> WavedashOutput:
    """Convenience wrapper; reuse the same action instance every tick."""

    if action is None:
        action = Wavedash()

    if not action.active and not action.finished:
        if not action.start(car, direction):
            return action.last_output

    return action.update(car, dt)
