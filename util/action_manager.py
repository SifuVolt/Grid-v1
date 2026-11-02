
"""Grid persistent action manager.

Inspired by Party Cannon's RUBot.Action lifecycle and Drive subactions.

Responsibilities:
- Keep one persistent action active across game ticks.
- Prevent normal strategy from constantly replacing active mechanics.
- Respect action interruptibility.
- Allow higher-priority emergency actions to preempt normal actions.
- Detect completion and clear finished actions.
- Return the mechanic's original output without changing its controls.

The manager is deliberately independent of RLBot packet/controller types.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import Any, Callable


class ActionPriority(Enum):
    IDLE = 0
    DRIVE = 10
    BOOST = 20
    RECOVERY = 30
    MECHANIC = 40
    SHOT = 50
    KICKOFF = 60
    EMERGENCY = 100


class ActionStatus(Enum):
    IDLE = auto()
    STARTED = auto()
    RUNNING = auto()
    BLOCKED = auto()
    PREEMPTED = auto()
    FINISHED = auto()
    ABORTED = auto()


@dataclass
class ActionRequest:
    """An action plus the function used to update it."""

    action: Any
    update_fn: Callable[..., Any]
    name: str
    priority: ActionPriority = ActionPriority.DRIVE
    interruptible: bool = True

    # Optional identifier prevents restarting an equivalent action every tick.
    key: str | None = None


@dataclass
class ActionManagerOutput:
    """Result of one action-manager update."""

    action_output: Any = None
    active_action: str | None = None
    status: ActionStatus = ActionStatus.IDLE

    changed: bool = False
    finished: bool = False
    preempted: bool = False
    blocked: bool = False

    reason: str = ""


class ActionManager:
    """Runs one persistent action and handles safe action transitions."""

    def __init__(self) -> None:
        self.current: ActionRequest | None = None
        self.last_output = ActionManagerOutput()

        self.previous_name: str | None = None
        self.previous_key: str | None = None

        self.started_count = 0
        self.finished_count = 0
        self.preempted_count = 0

    @property
    def active(self) -> bool:
        return self.current is not None

    @property
    def action(self) -> Any | None:
        return self.current.action if self.current else None

    @property
    def name(self) -> str | None:
        return self.current.name if self.current else None

    @property
    def priority(self) -> ActionPriority:
        if self.current is None:
            return ActionPriority.IDLE
        return self.current.priority

    @staticmethod
    def _is_finished(action: Any, output: Any) -> bool:
        """Check both the returned output and the action instance."""

        if bool(getattr(output, "finished", False)):
            return True

        if bool(getattr(action, "finished", False)):
            return True

        phase = getattr(action, "phase", None)
        phase_name = getattr(phase, "name", "")

        return phase_name in ("FINISHED", "ABORTED")

    @staticmethod
    def _is_aborted(action: Any, output: Any) -> bool:
        if getattr(output, "aborted", False):
            return True

        phase = getattr(action, "phase", None)
        phase_name = getattr(phase, "name", "")

        return phase_name == "ABORTED"

    @staticmethod
    def _request_is_same(
        current: ActionRequest,
        requested: ActionRequest,
    ) -> bool:
        if current.action is requested.action:
            return True

        if (
            current.key is not None
            and requested.key is not None
            and current.key == requested.key
        ):
            return True

        return False

    def can_replace(self, request: ActionRequest) -> bool:
        """Determine whether a new request may replace the current action."""

        if self.current is None:
            return True

        if self._request_is_same(self.current, request):
            return True

        if request.priority.value <= self.current.priority.value:
            return False

        if not self.current.interruptible:
            return False

        return True

    def start(self, request: ActionRequest) -> bool:
        """Start or replace the active action."""

        if self.current is not None:
            if self._request_is_same(self.current, request):
                return True

            if not self.can_replace(request):
                return False

            self.preempted_count += 1

        self.current = request
        self.previous_name = request.name
        self.previous_key = request.key
        self.started_count += 1

        return True

    def clear(self, reason: str = "cleared") -> None:
        """Clear the active action."""

        self.current = None
        self.last_output = ActionManagerOutput(
            status=ActionStatus.IDLE,
            changed=True,
            reason=reason,
        )

    def cancel(
        self,
        reason: str = "cancelled",
        force: bool = False,
    ) -> bool:
        """Cancel an action.

        Normal cancellation respects interruptibility.
        force=True is reserved for demolition, kickoff resets, or other
        explicit lifecycle resets.
        """

        if self.current is None:
            return True

        if not force and not self.current.interruptible:
            return False

        self.clear(reason)
        return True

    def update(
        self,
        car: Any,
        dt: float,
        emergency: ActionRequest | None = None,
    ) -> ActionManagerOutput:
        """Update the current action once.

        update_fn must accept either:
            update_fn(car, dt)
        or:
            update_fn(car)

        Emergency requests use the same interface and are permitted to
        replace a committed action when their priority is higher.
        """

        changed = False
        preempted = False

        if emergency is not None:
            if self.current is None:
                changed = self.start(emergency)

            elif not self._request_is_same(self.current, emergency):
                if emergency.priority.value > self.current.priority.value:
                    # Emergency actions are allowed to interrupt committed
                    # mechanics. The caller must reserve this for genuine
                    # emergencies such as a goal-line save.
                    self.preempted_count += 1
                    self.current = emergency
                    self.previous_name = emergency.name
                    self.previous_key = emergency.key
                    self.started_count += 1

                    changed = True
                    preempted = True

        if self.current is None:
            result = ActionManagerOutput(
                status=ActionStatus.IDLE,
                reason="no active action",
            )
            self.last_output = result
            return result

        request = self.current

        try:
            output = request.update_fn(car, dt)
        except TypeError:
            # Support simple one-argument update methods.
            # If the callback itself raises TypeError internally, preserve
            # that original error rather than silently masking it.
            import inspect

            try:
                signature = inspect.signature(request.update_fn)
                signature.bind(car, dt)
            except TypeError:
                output = request.update_fn(car)
            except (ValueError, AttributeError):
                raise
            else:
                raise

        if self._is_finished(request.action, output):
            aborted = self._is_aborted(request.action, output)

            self.current = None
            self.finished_count += 1

            result = ActionManagerOutput(
                action_output=output,
                active_action=None,
                status=(
                    ActionStatus.ABORTED
                    if aborted
                    else ActionStatus.FINISHED
                ),
                changed=True,
                finished=True,
                preempted=preempted,
                reason=(
                    getattr(output, "reason", "")
                    or ("action aborted" if aborted else "action finished")
                ),
            )

        else:
            result = ActionManagerOutput(
                action_output=output,
                active_action=request.name,
                status=(
                    ActionStatus.STARTED
                    if changed
                    else ActionStatus.RUNNING
                ),
                changed=changed,
                preempted=preempted,
                reason=getattr(output, "reason", "action running"),
            )

        self.last_output = result
        return result

    def reset(self) -> None:
        """Reset all manager state."""

        self.current = None
        self.previous_name = None
        self.previous_key = None

        self.started_count = 0
        self.finished_count = 0
        self.preempted_count = 0

        self.last_output = ActionManagerOutput(
            status=ActionStatus.IDLE,
            reason="manager reset",
        )


def make_request(
    action: Any,
    update_fn: Callable[..., Any],
    name: str,
    priority: ActionPriority = ActionPriority.DRIVE,
    interruptible: bool | None = None,
    key: str | None = None,
) -> ActionRequest:
    """Create a request, defaulting to the action's own interruptibility."""

    if interruptible is None:
        interruptible = bool(
            getattr(action, "interruptible", True)
        )

    return ActionRequest(
        action=action,
        update_fn=update_fn,
        name=name,
        priority=priority,
        interruptible=interruptible,
        key=key,
    )
