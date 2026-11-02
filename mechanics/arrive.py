
"""Grid arrival mechanic.

Drives to a target at a desired time and, optionally, arrives facing
a specified direction.

Inspired by Party Cannon Arrive.cs and Bumblebee Arrive.py.

Uses mechanics.drive for steering, braking, boost and turn control.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import hypot

from physics.math3d import Vec3, clamp
from mechanics.drive import Drive, DriveConfig, DriveOutput


class ArrivePhase(Enum):
    IDLE = "idle"
    APPROACH = "approach"
    ALIGN = "align"
    FINISHED = "finished"
    FAILED = "failed"
    INTERRUPTED = "interrupted"
    COMPLETE = "finished"
    ABORTED = "failed"

@dataclass
class ArriveConfig:
    allow_flipping: bool = False
    allow_boost: bool = True
    recovery_time: float = 0.0
    arrival_radius: float = 100.0
@dataclass
class ArriveOutput:
    throttle: float = 0.0
    steer: float = 0.0
    boost: bool = False
    handbrake: bool = False
    reverse: bool = False
    finished: bool = False
    failed: bool = False
    time_remaining: float = 0.0
    distance_remaining: float = 0.0
    target_speed: float = 0.0
    phase: ArrivePhase = ArrivePhase.IDLE
    reason: str = ""

    @classmethod
    def from_drive(
        cls,
        output: DriveOutput,
        phase: ArrivePhase,
        time_remaining: float,
    ) -> "ArriveOutput":
        return cls(
            throttle=output.throttle,
            steer=output.steer,
            boost=output.boost,
            handbrake=output.handbrake,
            reverse=output.reverse,
            finished=output.finished,
            time_remaining=time_remaining,
            distance_remaining=output.distance_remaining,
            target_speed=output.target_speed,
            phase=phase,
            reason=output.reason,
        )


def _flat_distance(a: Vec3, b: Vec3) -> float:
    return hypot(b.x - a.x, b.y - a.y)


def _flat_direction(vector: Vec3) -> Vec3:
    flat = Vec3(vector.x, vector.y, 0.0)
    if flat.magnitude < 0.001:
        return Vec3(0.0, 0.0, 0.0)
    return flat.normalized()


def _car_speed(car) -> float:
    velocity = getattr(car, "velocity", Vec3(0.0, 0.0, 0.0))
    return hypot(velocity.x, velocity.y)


def _car_time(car):
    value = getattr(car, "time", None)
    if value is None:
        return None
    return float(value)


class Arrive:
    """Persistent action for reaching a location at a desired time."""

    def __init__(
        self,
        target: Vec3 | ArriveConfig | None = None,
        arrival_time: float | None = None,
        direction: Vec3 | None = None,
        *,
        allow_flipping: bool = False,
        allow_boost: bool = True,
        recovery_time: float = 0.0,
        arrival_radius: float = 100.0,
    ):
        config = target if isinstance(target, ArriveConfig) else None

        if config is not None:
            target = None
            allow_flipping = config.allow_flipping
            allow_boost = config.allow_boost
            recovery_time = config.recovery_time
            arrival_radius = config.arrival_radius

        self.target = target
        self.direction = direction
        self.arrival_time = arrival_time

        self.allow_flipping = allow_flipping
        self.allow_boost = allow_boost
        self.recovery_time = max(0.0, recovery_time)
        self.arrival_radius = max(20.0, arrival_radius)

        self.phase = ArrivePhase.IDLE
        self.started = False
        self.finished = False
        self.failed = False
        self.interrupted = False

        self.elapsed = 0.0
        self.time_remaining = 0.0
        self.distance_remaining = 0.0

        self.shifted_target = None
        self.drive = None
        self.last_output = ArriveOutput()
        self._absolute_arrival_time = False
    @property

    def active(self) -> bool:
        return self.started and not self.is_finished()


    @property
    def eta(self) -> float:
        return max(0.0, self.time_remaining)


    def reset(self) -> None:
        self.phase = ArrivePhase.IDLE
        self.started = False
        self.finished = False
        self.failed = False
        self.interrupted = False
        self.elapsed = 0.0
        self.time_remaining = 0.0
        self.distance_remaining = 0.0
        self.shifted_target = None
        self.drive = None
        self.last_output = ArriveOutput()
        self._absolute_arrival_time = False


    def abort(self, reason: str = "arrival aborted") -> None:
        self.failed = True
        self.phase = ArrivePhase.FAILED

        if self.drive is not None:
            self.drive.interrupt()

        self.last_output = ArriveOutput(
            failed=True,
            phase=self.phase,
            reason=reason,
        )
    @property
    def interruptible(self) -> bool:
        if self.finished or self.failed or self.interrupted:
            return True

        if self.drive is None:
            return True

        return self.drive.is_finished()

    def start(
        self,
        car,
        target: Vec3 | None = None,
        arrival_time: float | None = None,
        direction: Vec3 | None = None,
        *,
        allow_boost: bool | None = None,
        intercept_time: float | None = None,
    ) -> ArriveOutput:
        """Start or restart the arrival action.

        arrival_time is a duration in seconds from now.
        """
        # Compatibility with shot routines that call:
        # start(car, target, direction, intercept_time=...)
        if isinstance(arrival_time, Vec3):
            if direction is None:
                direction = arrival_time
            arrival_time = intercept_time

        elif intercept_time is not None:
            arrival_time = intercept_time

        self._absolute_arrival_time = False
        if target is not None:
            self.target = target

        if arrival_time is not None:
            self.arrival_time = max(0.0, arrival_time)

        if direction is not None:
            self.direction = direction

        if allow_boost is not None:
            self.allow_boost = allow_boost

        if self.target is None:
            self.phase = ArrivePhase.FAILED
            self.failed = True
            self.last_output = ArriveOutput(
                failed=True,
                phase=self.phase,
                reason="no arrival target",
            )
            return self.last_output

        self.elapsed = 0.0
        self.finished = False
        self.failed = False
        self.interrupted = False
        self.started = True
        self.phase = ArrivePhase.APPROACH

        self.time_remaining = (
            self.arrival_time
            if self.arrival_time is not None
            else -1.0
        )

        self.shifted_target = self._calculate_shifted_target(car)

        self.drive = Drive(
            DriveConfig(
                arrival_radius=self.arrival_radius,
                target_speed=2300.0,
                allow_boost=self.allow_boost,
                allow_handbrake=True,
                allow_reverse=False,
            ),
            self.shifted_target,
        )

        drive_output = self.drive.start(car, self.shifted_target)
        self.last_output = ArriveOutput.from_drive(
            drive_output,
            self.phase,
            self.time_remaining,
        )
        self.last_output.reason = "arrival started: " + drive_output.reason
        return self.last_output

    def set_target(
        self,
        target: Vec3,
        direction: Vec3 | None = None,
    ) -> None:
        """Update the destination without restarting the action."""

        self.target = target

        if direction is not None:
            self.direction = direction

        if self.drive is not None:
            self.shifted_target = target
            self.drive.set_target(target, force=True)

    def interrupt(self) -> None:
        self.interrupted = True
        self.phase = ArrivePhase.INTERRUPTED

        if self.drive is not None:
            self.drive.interrupt()

        self.last_output = ArriveOutput(
            phase=self.phase,
            reason="arrival interrupted",
        )

    def is_finished(self) -> bool:
        return self.finished or self.failed or self.interrupted

    def _calculate_shifted_target(self, car) -> Vec3:
        """Move the approach point backward to allow final alignment."""

        assert self.target is not None

        if self.direction is None:
            return self.target

        direction = _flat_direction(self.direction)
        if direction.magnitude < 0.001:
            return self.target

        distance = _flat_distance(car.location, self.target)
        speed = _car_speed(car)

        # Bumblebee-style shift: use more room when far away,
        # but stop shifting when close enough to avoid overshooting.
        shift = min(
            distance * 0.56,
            clamp(speed, 1500.0, 2300.0) * 1.6,
        )

        # Party Cannon-style turn-radius safeguard.
        turn_radius = DriveTurnRadius(speed)

        if shift < turn_radius * 1.1:
            shift = 0.0
        else:
            shift = max(0.0, shift - self.recovery_time * speed)

        return self.target - direction * shift

    def _get_time_remaining(self, car, dt: float) -> float:
        if self.arrival_time is None:
            return max(
                0.05,
                self.distance_remaining / 1600.0,
            )

        car_time = _car_time(car)

        # If the car exposes game time, treat arrival_time as an
        # absolute game timestamp. Otherwise use a duration.
        if car_time is not None and self._absolute_arrival_time:
            return self.arrival_time - car_time

        return self.arrival_time - self.elapsed

    def update(
        self,
        car,
        dt: float = 1.0 / 120.0,
        *,
        target: Vec3 | None = None,
        direction: Vec3 | None = None,
        allow_boost: bool | None = None,
    ) -> ArriveOutput:
        dt = clamp(dt, 0.0, 0.1)

        if not self.started:
            return ArriveOutput(
                phase=ArrivePhase.IDLE,
                reason="arrival has not started",
            )

        if self.is_finished():
            return self.last_output

        if target is not None:
            self.set_target(target, direction)
        elif direction is not None:
            self.direction = direction

        if allow_boost is not None:
            self.allow_boost = allow_boost

        if self.target is None or self.drive is None:
            self.failed = True
            self.phase = ArrivePhase.FAILED
            self.last_output = ArriveOutput(
                failed=True,
                phase=self.phase,
                reason="arrival target or drive action missing",
            )
            return self.last_output

        self.elapsed += dt

        self.distance_remaining = _flat_distance(
            car.location,
            self.target,
        )

        self.time_remaining = self._get_time_remaining(car, dt)

        if self.time_remaining < -0.05:
            self.failed = True
            self.phase = ArrivePhase.FAILED
            self.last_output = ArriveOutput(
                failed=True,
                time_remaining=self.time_remaining,
                distance_remaining=self.distance_remaining,
                phase=self.phase,
                reason="arrival time expired",
            )
            return self.last_output

        if self.distance_remaining <= self.arrival_radius:
            self.finished = True
            self.phase = ArrivePhase.FINISHED
            self.last_output = ArriveOutput(
                finished=True,
                time_remaining=self.time_remaining,
                distance_remaining=self.distance_remaining,
                phase=self.phase,
                reason="arrival location reached",
            )
            return self.last_output

        # Recalculate the shifted target as the car moves.
        self.shifted_target = self._calculate_shifted_target(car)

        distance_to_drive_target = _flat_distance(
            car.location,
            self.shifted_target,
        )

        if self.arrival_time is None:
            desired_speed = 2300.0
        else:
            usable_time = max(0.05, self.time_remaining - self.recovery_time)
            desired_speed = clamp(
                distance_to_drive_target / usable_time,
                0.0,
                2300.0,
            )

        # If we need to arrive slowly, avoid rushing through the target.
        if (
            desired_speed < 800.0
            and distance_to_drive_target > 1000.0
        ):
            desired_speed = max(0.0, desired_speed)

        # When the car is nearly at the target, drive directly to it
        # instead of continuing to aim at the shifted approach point.
        if (
            self.direction is not None
            and distance_to_drive_target < 350.0
        ):
            self.phase = ArrivePhase.ALIGN
            drive_target = self.target
        else:
            self.phase = ArrivePhase.APPROACH
            drive_target = self.shifted_target

        drive_output = self.drive.update(
            car,
            drive_target,
            dt=dt,
            allow_boost=self.allow_boost,
            target_speed=desired_speed,
            force_target=False,
        )

        self.last_output = ArriveOutput.from_drive(
            drive_output,
            self.phase,
            self.time_remaining,
        )

        if drive_output.finished and self.distance_remaining <= self.arrival_radius:
            self.finished = True
            self.phase = ArrivePhase.FINISHED
            self.last_output.finished = True

        return self.last_output

    def run(self, car, dt: float = 1.0 / 120.0) -> ArriveOutput:
        return self.update(car, dt)


def DriveTurnRadius(speed: float) -> float:
    """Local import wrapper to keep the arrival module lightweight."""
    from physics.turning import turn_radius
    return turn_radius(speed)
