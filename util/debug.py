
"""Grid debugging and CMD diagnostics.

Provides rate-limited console output for live bot testing.

All output uses flush=True so messages appear immediately in the
RLBot console or the bot's CMD window.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from time import monotonic
from typing import Any

from physics.math3d import Vec3


DEFAULT_STATUS_INTERVAL = 1.0
DEFAULT_EVENT_INTERVAL = 0.20


def _format_number(value: Any, decimals: int = 2) -> str:
    try:
        return f"{float(value):.{decimals}f}"
    except (TypeError, ValueError, OverflowError):
        return "?"


def _format_vector(value: Vec3 | Any) -> str:
    try:
        return (
            f"({_format_number(value.x, 0)}, "
            f"{_format_number(value.y, 0)}, "
            f"{_format_number(value.z, 0)})"
        )
    except AttributeError:
        return "(?, ?, ?)"


def _safe_text(value: Any) -> str:
    if value is None:
        return "-"
    return str(value).replace("\n", " ").replace("\r", " ")


@dataclass(slots=True)
class DebugConfig:
    enabled: bool = True
    status_interval: float = DEFAULT_STATUS_INTERVAL
    event_interval: float = DEFAULT_EVENT_INTERVAL
    print_player_state: bool = True
    print_ball_state: bool = True
    print_teamplay: bool = True
    print_interception: bool = True
    print_mechanics: bool = True
    print_warnings: bool = True


@dataclass(slots=True)
class Debug:
    """Rate-limited Grid debug output."""

    config: DebugConfig = field(default_factory=DebugConfig)

    _last_status: float = 0.0
    _last_events: dict[str, float] = field(default_factory=dict)
    _last_signatures: dict[str, str] = field(default_factory=dict)

    def _now(self) -> float:
        return monotonic()

    def _print(self, category: str, message: str) -> None:
        if not self.config.enabled:
            return

        print(f"[GRID][{category}] {message}", flush=True)

    def event(
        self,
        key: str,
        message: str,
        interval: float | None = None,
    ) -> None:
        """Print a repeated event at most once per interval."""
        if not self.config.enabled:
            return

        now = self._now()
        interval = (
            self.config.event_interval
            if interval is None
            else max(0.0, interval)
        )

        previous = self._last_events.get(key, 0.0)

        if now - previous < interval:
            return

        self._last_events[key] = now
        self._print("EVENT", message)

    def changed(
        self,
        key: str,
        signature: Any,
        message: str,
    ) -> bool:
        """Print only when a value changes."""
        if not self.config.enabled:
            return False

        signature = str(signature)

        if self._last_signatures.get(key) == signature:
            return False

        self._last_signatures[key] = signature
        self._print("CHANGE", message)
        return True

    def status(self, message: str, force: bool = False) -> None:
        """Print a general status message periodically."""
        if not self.config.enabled:
            return

        now = self._now()

        if (
            not force
            and now - self._last_status < self.config.status_interval
        ):
            return

        self._last_status = now
        self._print("STATUS", message)

    def warning(self, message: str) -> None:
        if self.config.print_warnings:
            self._print("WARNING", message)

    def error(self, message: str) -> None:
        self._print("ERROR", message)

    def exception(self, context: str, error: BaseException) -> None:
        self._print(
            "ERROR",
            f"{context}: {type(error).__name__}: {error}",
        )

    def player(self, player: Any, label: str = "CAR") -> None:
        if not self.config.print_player_state:
            return

        car = getattr(player, "car", player)

        self._print(
            label,
            (
                f"index={getattr(player, 'index', '?')} "
                f"team={getattr(player, 'team', '?')} "
                f"pos={_format_vector(getattr(car, 'location', None))} "
                f"vel={_format_vector(getattr(car, 'velocity', None))} "
                f"speed={_format_number(getattr(car, 'speed', 0), 0)} "
                f"boost={_format_number(getattr(car, 'boost', 0), 0)} "
                f"ground={getattr(car, 'on_ground', '?')} "
                f"demolished={getattr(car, 'demolished', '?')}"
            ),
        )

    def ball(self, ball: Any) -> None:
        if not self.config.print_ball_state:
            return

        self._print(
            "BALL",
            (
                f"pos={_format_vector(getattr(ball, 'location', None))} "
                f"vel={_format_vector(getattr(ball, 'velocity', None))} "
                f"speed={_format_number(getattr(ball, 'speed', 0), 0)}"
            ),
        )

    def teamplay(self, decision: Any) -> None:
        if not self.config.print_teamplay:
            return

        role = getattr(decision, "role", "unknown")
        challenger = getattr(decision, "challenger_index", None)
        target = getattr(decision, "target", None)
        reason = getattr(decision, "reason", "")

        self.changed(
            "teamplay",
            (
                role,
                challenger,
                reason,
            ),
            (
                f"role={_safe_text(role)} "
                f"challenger={_safe_text(challenger)} "
                f"target={_format_vector(target)} "
                f"reason={_safe_text(reason)}"
            ),
        )

    def interception(self, result: Any) -> None:
        if not self.config.print_interception:
            return

        best = getattr(result, "best", result)

        if best is None:
            self.changed(
                "interception",
                "none",
                "No reachable interception found",
            )
            return

        self.changed(
            "interception",
            (
                getattr(best, "time", None),
                getattr(best, "car_target", None),
                getattr(best, "aerial", None),
            ),
            (
                f"time={_format_number(getattr(best, 'time', 0))} "
                f"target={_format_vector(getattr(best, 'car_target', None))} "
                f"margin={_format_number(getattr(best, 'arrival_margin', 0))} "
                f"score={_format_number(getattr(best, 'score', 0))} "
                f"aerial={getattr(best, 'aerial', False)}"
            ),
        )

    def mechanic(
        self,
        name: str,
        state: str = "active",
        target: Vec3 | None = None,
        detail: str = "",
    ) -> None:
        if not self.config.print_mechanics:
            return

        signature = (name, state, target, detail)

        self.changed(
            "mechanic",
            signature,
            (
                f"name={_safe_text(name)} "
                f"state={_safe_text(state)} "
                f"target={_format_vector(target)} "
                f"detail={_safe_text(detail)}"
            ),
        )

    def frame(self, state: Any, decision: Any = None) -> None:
        """Print a compact periodic snapshot."""
        if not self.config.enabled:
            return

        now = self._now()

        if now - self._last_status < self.config.status_interval:
            return

        self._last_status = now

        me = getattr(state, "me", None)
        ball = getattr(state, "ball", None)

        parts = [
            f"frame={getattr(state, 'frame', '?')}",
            f"time={_format_number(getattr(state, 'game_time', 0))}",
        ]

        if me is not None:
            car = getattr(me, "car", me)
            parts.extend([
                f"car={_format_vector(getattr(car, 'location', None))}",
                f"speed={_format_number(getattr(car, 'speed', 0), 0)}",
                f"boost={_format_number(getattr(car, 'boost', 0), 0)}",
            ])

        if ball is not None:
            parts.append(
                f"ball={_format_vector(getattr(ball, 'location', None))}"
            )

        if decision is not None:
            parts.append(
                f"role={_safe_text(getattr(decision, 'role', '?'))}"
            )
            parts.append(
                f"challenger={_safe_text(getattr(decision, 'challenger_index', None))}"
            )

        self._print("FRAME", " ".join(parts))

    def reset(self) -> None:
        """Reset rate-limit and change-detection state."""
        self._last_status = 0.0
        self._last_events.clear()
        self._last_signatures.clear()


# ---------------------------------------------------------------------------
# Convenience functions
# ---------------------------------------------------------------------------

_default_debug = Debug()


def get_debug() -> Debug:
    return _default_debug


def debug_print(message: str) -> None:
    _default_debug._print("INFO", message)


def debug_warning(message: str) -> None:
    _default_debug.warning(message)


def debug_error(message: str) -> None:
    _default_debug.error(message)


def debug_event(key: str, message: str) -> None:
    _default_debug.event(key, message)


def debug_status(message: str, force: bool = False) -> None:
    _default_debug.status(message, force)
