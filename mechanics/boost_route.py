
"""Grid boost route planner.

Inspired by Party Cannon's GetBoost.cs.

Chooses boost pads using:
- Estimated travel time.
- Pad availability on arrival.
- Boost amount gained.
- Detour from the strategic destination.
- Whether collecting boost conflicts with the current role.

The planner returns a ranked route. BoostManager remains responsible
for executing the selected collection action.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import inf
from typing import Any, Iterable

from physics.math3d import Vec3
from physics.reachability import estimate_reachability
from physics.vehicle import VehicleState


@dataclass
class BoostRouteConfig:
    max_eta: float = 5.0
    minimum_eta: float = 0.05

    stop_boost: float = 90.0
    minimum_gain: float = 8.0

    # Strategic detour penalty.
    detour_weight: float = 0.65

    # Reward for large pads, while still allowing small pads to be useful.
    large_pad_bonus: float = 0.45

    # Prefer pads that become active before arrival.
    respawn_tolerance: float = 0.15

    # Avoid collecting boost when the ball is dangerously close.
    danger_distance: float = 1800.0

    # Do not divert a challenger from a nearby ball.
    challenger_ball_distance: float = 1500.0

    # Route scoring weights.
    eta_weight: float = 1.0
    gain_weight: float = 0.025
    strategic_weight: float = 1.0


@dataclass
class BoostRouteCandidate:
    pad_index: int
    location: Vec3
    is_large: bool

    eta: float
    time_until_active: float
    expected_gain: float

    strategic_detour: float
    strategic_eta: float

    score: float
    valid: bool
    reason: str = ""


@dataclass
class BoostRoute:
    candidates: list[BoostRouteCandidate] = field(default_factory=list)
    best: BoostRouteCandidate | None = None

    valid: bool = False
    reason: str = ""

    @property
    def pad_index(self) -> int | None:
        return self.best.pad_index if self.best else None

    @property
    def target(self) -> Vec3 | None:
        return self.best.location if self.best else None

    @property
    def eta(self) -> float:
        return self.best.eta if self.best else inf


def _pad_value(pad: Any, *names: str, default: Any = None) -> Any:
    for name in names:
        if hasattr(pad, name):
            return getattr(pad, name)
    return default


def _pad_location(pad: Any) -> Vec3 | None:
    value = _pad_value(pad, "location", "position")

    if value is None:
        return None

    try:
        return Vec3.from_value(value)
    except (TypeError, ValueError, AttributeError):
        return None


def _pad_index(pad: Any, fallback: int) -> int:
    return int(_pad_value(pad, "index", default=fallback))


def _pad_is_large(pad: Any) -> bool:
    return bool(
        _pad_value(
            pad,
            "is_large",
            "is_full_boost",
            "large",
            default=False,
        )
    )


def _pad_is_active(pad: Any) -> bool:
    return bool(_pad_value(pad, "is_active", "active", default=True))


def _pad_respawn_time(pad: Any) -> float:
    value = _pad_value(
        pad,
        "time_until_active",
        "inactive_time",
        "respawn_time",
        default=0.0,
    )

    try:
        return max(0.0, float(value))
    except (TypeError, ValueError):
        return 0.0


def _pad_gain(pad: Any) -> float:
    explicit = _pad_value(pad, "boost_amount", "amount")

    if explicit is not None:
        try:
            return float(explicit)
        except (TypeError, ValueError):
            pass

    return 100.0 if _pad_is_large(pad) else 12.0


def _get_pads(source: Any) -> list[Any]:
    """Accept a tracker, iterable, or object exposing a pads collection."""

    if source is None:
        return []

    if hasattr(source, "pads"):
        pads = source.pads
        if isinstance(pads, dict):
            return list(pads.values())
        return list(pads)

    if isinstance(source, dict):
        return list(source.values())

    return list(source)


def _arrival_time(car: VehicleState, target: Vec3) -> float:
    result = estimate_reachability(car, target)

    if not bool(getattr(result, "reachable", False)):
        return inf

    try:
        return max(0.0, float(result.time))
    except (AttributeError, TypeError, ValueError):
        return inf


def _strategic_detour(
    car: VehicleState,
    pad_location: Vec3,
    strategic_target: Vec3 | None,
) -> tuple[float, float]:
    """Compare direct travel against travel through the boost pad."""

    if strategic_target is None:
        return 0.0, 0.0

    direct_eta = _arrival_time(car, strategic_target)
    pad_eta = _arrival_time(car, pad_location)

    if pad_eta == inf:
        return inf, direct_eta

    # Approximate the onward leg using current speed. This is a route
    # comparison, not a replacement for a full simulated trajectory.
    onward_distance = pad_location.flat_distance(strategic_target)
    onward_speed = max(900.0, car.flat_speed)

    onward_eta = onward_distance / onward_speed

    if direct_eta == inf:
        return onward_eta, direct_eta

    return max(0.0, pad_eta + onward_eta - direct_eta), direct_eta


def score_boost_route(
    car: VehicleState,
    pad: Any,
    pad_index: int,
    config: BoostRouteConfig,
    strategic_target: Vec3 | None = None,
) -> BoostRouteCandidate:
    location = _pad_location(pad)
    large = _pad_is_large(pad)
    active = _pad_is_active(pad)
    respawn = 0.0 if active else _pad_respawn_time(pad)
    gain = _pad_gain(pad)

    invalid = lambda reason: BoostRouteCandidate(
        pad_index=pad_index,
        location=location or car.location,
        is_large=large,
        eta=inf,
        time_until_active=respawn,
        expected_gain=0.0,
        strategic_detour=inf,
        strategic_eta=inf,
        score=-inf,
        valid=False,
        reason=reason,
    )

    if location is None:
        return invalid("pad has no valid location")

    if car.boost >= config.stop_boost:
        return invalid("boost already sufficient")

    if gain < config.minimum_gain:
        return invalid("boost gain too small")

    eta = _arrival_time(car, location)

    if eta == inf:
        return invalid("pad is unreachable")

    if eta < config.minimum_eta:
        eta = config.minimum_eta

    if eta > config.max_eta:
        return invalid("pad ETA exceeds route limit")

    # Party Cannon's key availability test:
    # an inactive pad is still a candidate if it respawns before arrival.
    if not active and respawn > eta + config.respawn_tolerance:
        return invalid("pad will not be active on arrival")

    detour, strategic_eta = _strategic_detour(
        car,
        location,
        strategic_target,
    )

    if detour == inf:
        return invalid("strategic route unavailable")

    gain_score = min(gain, 100.0 - car.boost)

    score = (
        -config.eta_weight * eta
        -config.detour_weight * detour
        +config.gain_weight * gain_score
        +(
            config.large_pad_bonus
            if large
            else 0.0
        )
    )

    return BoostRouteCandidate(
        pad_index=pad_index,
        location=location,
        is_large=large,
        eta=eta,
        time_until_active=respawn,
        expected_gain=gain_score,
        strategic_detour=detour,
        strategic_eta=strategic_eta,
        score=score,
        valid=True,
        reason="reachable and available on arrival",
    )


def find_boost_route(
    car: VehicleState,
    pads_or_tracker: Any,
    strategic_target: Vec3 | None = None,
    config: BoostRouteConfig | None = None,
    *,
    role: str = "",
    ball_location: Vec3 | None = None,
    emergency: bool = False,
) -> BoostRoute:
    """Rank available boost pads and return the best route.

    Cover players may collect pads while supporting their team.
    Challengers should not divert from a nearby ball.
    Emergency situations suppress all boost routes.
    """

    config = config or BoostRouteConfig()

    if emergency:
        return BoostRoute(
            reason="emergency overrides boost collection",
        )

    if car.demolished:
        return BoostRoute(reason="car demolished")

    if car.boost >= config.stop_boost:
        return BoostRoute(reason="boost already sufficient")

    if (
        ball_location is not None
        and role.lower() in ("challenge", "challenger")
        and car.location.flat_distance(ball_location)
        < config.challenger_ball_distance
    ):
        return BoostRoute(
            reason="challenger must stay with nearby ball",
        )

    pads = _get_pads(pads_or_tracker)

    candidates: list[BoostRouteCandidate] = []

    for i, pad in enumerate(pads):
        candidate = score_boost_route(
            car,
            pad,
            _pad_index(pad, i),
            config,
            strategic_target,
        )

        if candidate.valid:
            candidates.append(candidate)

    candidates.sort(key=lambda candidate: candidate.score, reverse=True)

    if not candidates:
        return BoostRoute(
            candidates=[],
            reason="no suitable boost route",
        )

    return BoostRoute(
        candidates=candidates,
        best=candidates[0],
        valid=True,
        reason="best boost route selected",
    )


def next_boost_pad(
    car: VehicleState,
    tracker: Any,
    strategic_target: Vec3 | None = None,
    config: BoostRouteConfig | None = None,
) -> BoostRouteCandidate | None:
    """Convenience function returning only the best candidate."""

    result = find_boost_route(
        car,
        tracker,
        strategic_target,
        config,
    )
    return result.best


def rank_boost_pads(
    car: VehicleState,
    pads: Iterable[Any],
    strategic_target: Vec3 | None = None,
    config: BoostRouteConfig | None = None,
) -> list[BoostRouteCandidate]:
    """Return every valid candidate, best first."""

    return find_boost_route(
        car,
        pads,
        strategic_target,
        config,
    ).candidates
