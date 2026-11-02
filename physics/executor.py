
"""Grid interception execution layer.

Connects interception planning to the ground-driving controller.
This module produces driving commands but does not yet send them
to RLBot or directly control the game.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from physics.math3d import Vec3, clamp
from physics.trajectory import Trajectory
from physics.reachability import ReachabilityCar
from physics.vehicle import VehicleState
from physics.interception import (
    InterceptionPlan,
    choose_interception,
)
from physics.steering import (
    DriveCommand,
    drive_to_point,
)


@dataclass(frozen=True)
class ExecutionResult:
    """Complete result of one interception execution update."""

    plan: Optional[InterceptionPlan]
    drive: Optional[DriveCommand]
    target: Optional[Vec3]
    status: str

    @property
    def has_plan(self) -> bool:
        return self.plan is not None

    @property
    def has_drive_command(self) -> bool:
        return self.drive is not None


class InterceptionExecutor:
    """Select and execute a predicted ball interception."""

    def __init__(
        self,
        minimum_time: float = 0.08,
        maximum_time: float = 4.5,
        contact_offset: float = 115.0,
        minimum_drive_speed: float = 900.0,
        maximum_drive_speed: float = 2300.0,
    ):
        self.minimum_time = minimum_time
        self.maximum_time = maximum_time
        self.contact_offset = contact_offset

        self.minimum_drive_speed = minimum_drive_speed
        self.maximum_drive_speed = maximum_drive_speed

        self.last_plan: Optional[InterceptionPlan] = None
        self.last_status = "NOT_STARTED"

    def reset(self) -> None:
        """Clear the current execution state."""
        self.last_plan = None
        self.last_status = "RESET"

    def update(
        self,
        reachability_car: ReachabilityCar,
        vehicle: VehicleState,
        trajectory: Trajectory,
        attack_target: Optional[Vec3] = None,
        allow_boost: bool = True,
        allow_handbrake: bool = True,
    ) -> ExecutionResult:
        """Find a contact and produce controls toward its approach point."""

        if trajectory is None or len(trajectory) == 0:
            self.last_plan = None
            self.last_status = "NO_TRAJECTORY"

            return ExecutionResult(
                plan=None,
                drive=None,
                target=None,
                status=self.last_status,
            )

        plan = choose_interception(
            car=reachability_car,
            trajectory=trajectory,
            attack_target=attack_target,
            minimum_time=self.minimum_time,
            maximum_time=self.maximum_time,
            contact_offset=self.contact_offset,
        )

        if plan is None:
            self.last_plan = None
            self.last_status = "NO_REACHABLE_INTERCEPTION"

            return ExecutionResult(
                plan=None,
                drive=None,
                target=None,
                status=self.last_status,
            )

        target = plan.approach_position

        # Estimate the speed needed to cover the approach distance
        # before the predicted contact time.
        distance = vehicle.location.flat().distance(target.flat())
        time_available = max(plan.contact_time, 0.1)

        required_speed = distance / time_available

        desired_speed = clamp(
            required_speed,
            self.minimum_drive_speed,
            self.maximum_drive_speed,
        )

        drive = drive_to_point(
            car=vehicle,
            target=target,
            desired_speed=desired_speed,
            allow_boost=allow_boost,
            allow_handbrake=allow_handbrake,
        )

        self.last_plan = plan
        self.last_status = "EXECUTING"

        return ExecutionResult(
            plan=plan,
            drive=drive,
            target=target,
            status=self.last_status,
        )


def describe_execution(result: ExecutionResult) -> str:
    """Return compact execution diagnostics for CMD."""

    if result.plan is None:
        return f"status={result.status}"

    plan = result.plan

    output = (
        f"status={result.status} "
        f"method={plan.method} "
        f"contact_time={plan.contact_time:.3f}s "
        f"score={plan.score:.3f} "
        f"target={result.target}"
    )

    if result.drive is not None:
        output += (
            f" | throttle={result.drive.throttle:.2f}"
            f" steer={result.drive.steer:.2f}"
            f" boost={result.drive.boost}"
            f" handbrake={result.drive.handbrake}"
            f" desired_speed={result.drive.target_speed:.1f}"
        )

    return output
