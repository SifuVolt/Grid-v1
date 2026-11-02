
"""Grid's persistent action routines and mechanic controllers."""
from __future__ import annotations
from dataclasses import dataclass, field
from math import atan2, pi
from .utils import *
from .tools import CarState, Intercept, get_rotation_basis
from physics.math3d import Vec3 as PhysicsVec3, Orientation
from physics.vehicle import VehicleState
from physics.steering import (
    drive_to_point as physics_drive_to_point,
    DriveMemory,
)
from physics.aerial import find_best_aerial_option
import math
_DRIVE_MEMORY = {}

@dataclass(slots=True)
class RoutineResult:
    controls: dict
    finished: bool = False
    interruptible: bool = True
    label: str = "idle"

def neutral():
    return {"throttle": 0.0, "steer": 0.0, "pitch": 0.0, "yaw": 0.0,
            "roll": 0.0, "jump": False, "boost": False, "handbrake": False}


def steer_to(car: CarState, target: Vec3, reverse: bool = False) -> float:
    vehicle = VehicleState(
        location=PhysicsVec3(
            car.position.x, car.position.y, car.position.z
        ),
        velocity=PhysicsVec3(
            car.velocity.x, car.velocity.y, car.velocity.z
        ),
        yaw=car.yaw,
        boost=car.boost,
        on_ground=car.on_ground,
        handbrake=False,
        angular_velocity_z=car.physics.angular_velocity.z,
        local_angular_velocity=Orientation.from_rotator(
            car.physics.rotation
        ).world_to_local(
            PhysicsVec3.from_rlbot(car.physics.angular_velocity)
        ),
        orientation=Orientation.from_rotator(car.physics.rotation),
    )

    physics_target = PhysicsVec3(
        target.x, target.y, target.z
    )

    return physics_drive_to_point(
        vehicle,
        physics_target,
        desired_speed=900.0 if reverse else 2200.0,
        allow_boost=False,
    ).steer



def drive_controller(
    car,
    target,
    desired_speed=2200,
    allow_boost=True,
    handbrake=True,
    action_key="default",
):
    vehicle = VehicleState(
        location=PhysicsVec3(
            car.position.x, car.position.y, car.position.z
        ),
        velocity=PhysicsVec3(
            car.velocity.x, car.velocity.y, car.velocity.z
        ),
        yaw=car.yaw,
        boost=car.boost,
        on_ground=car.on_ground,
        handbrake=False,
        angular_velocity_z=car.physics.angular_velocity.z,
        local_angular_velocity=Orientation.from_rotator(
            car.physics.rotation
        ).world_to_local(
            PhysicsVec3.from_rlbot(car.physics.angular_velocity)
        ),
        orientation=Orientation.from_rotator(car.physics.rotation),
    )

    physics_target = PhysicsVec3(
        target.x, target.y, target.z
    )

    memory = _DRIVE_MEMORY.setdefault(
        car.index,
        DriveMemory(),
    )

    command = physics_drive_to_point(
        vehicle,
        physics_target,
        desired_speed=desired_speed,
        allow_boost=allow_boost,
        allow_handbrake=handbrake,
        memory=memory,
        action_key=action_key,
    )

    controls = neutral()
    controls["throttle"] = command.throttle
    controls["steer"] = command.steer
    controls["boost"] = command.boost
    controls["handbrake"] = command.handbrake

    return controls



@dataclass
class JumpShotRoutine:
    """Two-stage jump timing controller for a predicted ground/low-ball contact."""
    target: Vec3
    started_at: float
    stage: str = "approach"
    stage_time: float = 0.0
    jump_hold: float = 0.085
    dodge_delay: float = 0.19
    dodge_direction: Vec3 = Vec3(1, 0, 0)
    completed: bool = False

    def tick(self, car: CarState, now: float) -> RoutineResult:
        elapsed = max(0.0, now-self.started_at)
        c = neutral()
        if self.stage == "approach":
            c = drive_controller(
                car, self.target, 1900,
                action_key="jump_shot_approach",
            )
            if elapsed >= 0.08:
                self.stage, self.stage_time = "jump", now
        elif self.stage == "jump":
            c["throttle"] = 1.0
            c["jump"] = now-self.stage_time < self.jump_hold
            c["boost"] = car.boost > 0
            if now-self.stage_time >= self.dodge_delay:
                self.stage, self.stage_time = "dodge", now
        elif self.stage == "dodge":
            c["throttle"] = 1.0
            c["jump"] = True
            c["pitch"] = -clamp(self.dodge_direction.x, -1, 1)
            c["yaw"] = clamp(self.dodge_direction.y, -1, 1)
            if now-self.stage_time > 0.12:
                self.stage, self.stage_time = "release", now
        else:
            c["jump"] = False
            self.completed = True
        return RoutineResult(c, self.completed, True, "jump_shot")

@dataclass
class AerialRoutine:
    """Approach, evaluate and execute a feasible aerial."""

    target: Vec3
    started_at: float
    intercept_time: float

    phase: str = "approach"
    phase_at: float = 0.0
    jump_option: object = None
    completed: bool = False

    approach_radius: float = 450.0
    jump_hold: float = 0.20
    boost_threshold: float = 1.0

    def _remaining(self, now):
        return max(0.0, self.intercept_time - (now - self.started_at))

    def _begin_aerial(self, car, now):
        forward, _, _ = get_rotation_basis(car.physics.rotation)
        remaining = self._remaining(now)

        option = find_best_aerial_option(
            position=car.position,
            velocity=car.velocity,
            forward=forward,
            target=self.target,
            time_available=remaining,
            boost=car.boost,
        )

        if option is None or not option.reachable:
            return False

        self.jump_option = option
        self.phase = "first_jump"
        self.phase_at = now
        return True

    def tick(self, car: CarState, now: float) -> RoutineResult:
        c = neutral()
        remaining = self._remaining(now)

        if self.completed:
            return RoutineResult(c, True, True, "aerial")

        if remaining <= 0.04:
            self.completed = True
            return RoutineResult(c, True, True, "aerial")

        # Phase 1: approach the horizontal launch position.
        if self.phase == "approach":
            if not car.on_ground:
                self.completed = True
                return RoutineResult(c, True, True, "aerial")

            launch_position = Vec3(
                self.target.x,
                self.target.y,
                0.0,
            )

            distance = car.position.flat_distance(launch_position)

            if distance > self.approach_radius:
                c = drive_controller(
                    car,
                    launch_position,
                    desired_speed=1500,
                    allow_boost=False,
                    handbrake=True,
                    action_key="aerial_approach",
                )

                # Don't spend the entire interception window driving.
                if remaining < 0.35:
                    self.completed = True

                return RoutineResult(
                    c,
                    self.completed,
                    True,
                    "aerial",
                )

            # Only evaluate once close enough to the launch position.
            if not self._begin_aerial(car, now):
                self.completed = True
                return RoutineResult(c, True, True, "aerial")

        # Phase 2: first jump, held for approximately 0.2 seconds.
        if self.phase == "first_jump":
            c["throttle"] = 1.0
            c["jump"] = now - self.phase_at < self.jump_hold

            if now - self.phase_at >= self.jump_hold:
                self.phase = "release"
                self.phase_at = now

        # Phase 3: release jump to create the second-press edge.
        elif self.phase == "release":
            c["jump"] = False

            if now - self.phase_at >= 3.0 / 120.0:
                if self.jump_option.method == "double_jump":
                    self.phase = "second_jump"
                else:
                    self.phase = "flight"

                self.phase_at = now

        # Phase 4: second jump only when the evaluator selected it.
        elif self.phase == "second_jump":
            c["jump"] = True
            c["throttle"] = 1.0

            if now - self.phase_at >= 3.0 / 120.0:
                self.phase = "flight"
                self.phase_at = now

        # Phase 5: orient toward the contact target and use boost.
        elif self.phase == "flight":
            if car.on_ground:
                self.completed = True
                return RoutineResult(c, True, True, "aerial")

            forward, right, up = get_rotation_basis(
                car.physics.rotation
            )

            offset = self.target - car.position
            distance = offset.magnitude()

            if distance < 160.0:
                self.completed = True
                return RoutineResult(c, True, True, "aerial")

            desired = offset.normalized()
            forward_error = forward.cross(desired)

            c["pitch"] = clamp(
                -forward_error.dot(right) * 3.0,
                -1.0,
                1.0,
            )

            c["yaw"] = clamp(
                forward_error.dot(up) * 3.0,
                -1.0,
                1.0,
            )

            # Keep roll corrections restrained.
            c["roll"] = clamp(
                -float(getattr(car.physics.rotation, "roll", 0.0)) * 1.2,
                -0.6,
                0.6,
            )

            alignment = forward.dot(desired)

            c["throttle"] = 1.0
            c["boost"] = (
                car.boost > self.boost_threshold
                and alignment > 0.55
                and remaining > 0.08
            )

            # Abandon if the contact window has expired.
            if remaining <= 0.08:
                self.completed = True

        return RoutineResult(
            c,
            self.completed,
            False,
            "aerial",
        )

@dataclass
class FlipRoutine:
    """Generic flip/wavedash pulse with explicit release windows."""
    started_at: float
    direction: Vec3 = Vec3(1, 0, 0)
    kind: str = "front_flip"
    phase: str = "jump"
    phase_at: float = 0.0
    completed: bool = False

    def tick(self, car: CarState, now: float) -> RoutineResult:
        c = neutral()
        if self.phase == "jump":
            c["jump"] = True
            c["throttle"] = 1.0
            if now-self.started_at > 0.07:
                self.phase, self.phase_at = "release", now
        elif self.phase == "release":
            c["jump"] = False
            if now-self.phase_at > 0.035:
                self.phase, self.phase_at = "dodge", now
        elif self.phase == "dodge":
            c["jump"] = True
            c["pitch"] = -clamp(self.direction.x, -1, 1)
            c["yaw"] = clamp(self.direction.y, -1, 1)
            c["throttle"] = 1.0
            if now-self.phase_at > 0.11:
                self.phase, self.phase_at = "recover", now
        else:
            c["jump"] = False
            c["throttle"] = 1.0
            self.completed = now-self.phase_at > 0.18
        return RoutineResult(c, self.completed, True, self.kind)

@dataclass
class RoutineManager:
    active: object = None
    last_started: float = -99.0
    cooldowns: dict = field(default_factory=dict)

    def start(self, routine, now: float, name: str):
        if self.active is not None:
            return False
        if now-self.cooldowns.get(name, -99.0) < 0.35:
            return False
        self.active = routine
        self.last_started = now
        return True

    def tick(self, car: CarState, now: float):
        if self.active is None:
            return None
        result = self.active.tick(car, now)
        if result.finished:
            self.cooldowns[result.label] = now
            self.active = None
        return result

    def cancel(self):
        self.active = None

class MechanicPlanner:
    """Conservative mechanic eligibility; tactical decisions remain outside routines."""
    @staticmethod
    def should_jump_shot(car: CarState, intercept: Intercept | None, ball_z: float) -> bool:
        return bool(intercept and intercept.reachable and intercept.mode == "ground"
                    and 0.22 < intercept.time < 0.78 and 120 < ball_z < 430
                    and car.on_ground and car.speed > 650)

    @staticmethod
    def should_aerial(car: CarState, intercept: Intercept | None) -> bool:
        return bool(intercept and intercept.reachable and intercept.mode == "aerial"
                    and intercept.time < 2.5 and car.boost > 17)

    @staticmethod
    def should_wave_dash(car: CarState, target: Vec3) -> bool:
        return bool(car.position.z > 45 and car.position.z < 155
                    and car.velocity.z < -80 and car.position.flat_distance(target) < 550
                    and car.flat_speed > 650)

    @staticmethod
    def should_half_flip(car: CarState, target: Vec3) -> bool:
        local = local_coordinates(target-car.position, car.yaw)
        return bool(local.x < -500 and car.flat_speed < 900 and car.on_ground)
