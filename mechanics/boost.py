
"""Grid boost-pad tracking, selection and collection.

Inspired by:
- Party Cannon GetBoost: ETA-based selection, pad respawn checks and
  persistent driving to the selected pad.
- Noob Black BoostPadTracker: initialize pad locations from FieldInfo and
  update active state/timers from each GamePacket.

Grid additions:
- Small and large pads are tracked separately.
- Route-aware scoring avoids unnecessary detours.
- Cover players are restricted from abandoning defensive positions.
- Emergency situations pre-empt boost collection.
- Pad selection is committed briefly to prevent target oscillation.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from math import inf
from typing import Any, Iterable

from physics.math3d import Vec3, clamp
from physics.vehicle import VehicleState
from physics.reachability import estimate_reachability
from mechanics.drive import Drive, DriveOutput


SMALL_PAD_RESPAWN = 4.0
LARGE_PAD_RESPAWN = 10.0

SMALL_PAD_BOOST = 12.0
LARGE_PAD_BOOST = 100.0


class BoostPhase(Enum):
    IDLE = auto()
    COLLECTING = auto()
    ARRIVED = auto()
    FINISHED = auto()
    ABORTED = auto()


@dataclass
class BoostPad:
    index: int
    location: Vec3
    is_large: bool
    is_active: bool = True
    inactive_time: float = 0.0

    @property
    def respawn_duration(self) -> float:
        return LARGE_PAD_RESPAWN if self.is_large else SMALL_PAD_RESPAWN

    @property
    def time_until_active(self) -> float:
        if self.is_active:
            return 0.0
        return max(0.0, self.respawn_duration - self.inactive_time)

    @property
    def boost_amount(self) -> float:
        return LARGE_PAD_BOOST if self.is_large else SMALL_PAD_BOOST


@dataclass
class BoostConfig:
    # Stop collecting when boost reaches this amount.
    stop_boost: float = 90.0

    # Do not start a collection run above this amount.
    start_boost_below: float = 65.0

    # Minimum expected boost gain for a collection run.
    minimum_gain: float = 8.0

    # Candidate ETA limits.
    maximum_eta: float = 5.0
    minimum_eta: float = 0.05

    # A pad may be slightly inactive on arrival due to ETA uncertainty.
    respawn_tolerance: float = 0.15

    # Route preference.
    route_detour_weight: float = 0.002
    large_pad_preference: float = 0.25
    eta_weight: float = 1.0
    gain_weight: float = 0.8

    # Selection stability.
    commitment_time: float = 0.45
    refresh_interval: float = 0.25
    replacement_score_margin: float = 0.35

    # Defensive restrictions.
    cover_max_detour: float = 350.0
    cover_max_eta: float = 1.8
    cover_minimum_boost: float = 35.0

    # Do not collect if the car is already in a dangerous position.
    own_goal_guard_distance: float = 1800.0

    # Drive configuration.
    drive_to_small_pads: bool = True
    drive_to_large_pads: bool = True


@dataclass
class BoostCandidate:
    valid: bool = False
    pad_index: int = -1
    location: Vec3 | None = None
    is_large: bool = False

    eta: float = inf
    time_until_active: float = inf
    arrival_margin: float = -inf

    distance: float = inf
    route_detour: float = inf
    expected_gain: float = 0.0
    projected_boost: float = 0.0

    score: float = inf
    reason: str = ""


@dataclass
class BoostOutput:
    phase: BoostPhase = BoostPhase.IDLE
    active: bool = False
    finished: bool = False

    throttle: float = 0.0
    steer: float = 0.0
    boost: bool = False
    handbrake: bool = False

    target: Vec3 | None = None
    pad_index: int = -1
    pad_is_large: bool = False
    eta: float = 0.0
    expected_gain: float = 0.0
    reason: str = ""


def _get(obj: Any, *names: str, default: Any = None) -> Any:
    for name in names:
        try:
            value = getattr(obj, name)
        except (AttributeError, TypeError):
            continue

        if value is not None:
            return value

    return default


def _as_vec3(value: Any) -> Vec3:
    if isinstance(value, Vec3):
        return value

    try:
        return Vec3.from_value(value)
    except (TypeError, ValueError, AttributeError):
        return Vec3()


def _is_large_pad(pad: Any) -> bool:
    return bool(
        _get(
            pad,
            "is_large",
            "is_full_boost",
            "large",
            default=False,
        )
    )


class BoostPadTracker:
    """Maintains pad locations and their current active/respawn state.

    Initialize once with RLBot FieldInfo. Then call update_from_packet
    every frame, as Noob Black does.
    """

    def __init__(self):
        self.pads: list[BoostPad] = []
        self.small_pads: list[BoostPad] = []
        self.large_pads: list[BoostPad] = []
        self.initialized = False

    def initialize(self, field_info: Any) -> int:
        field_pads = _get(
            field_info,
            "boost_pads",
            "boostPads",
            default=[],
        )

        self.pads = []

        for index, raw_pad in enumerate(field_pads):
            location = _as_vec3(_get(raw_pad, "location"))

            pad = BoostPad(
                index=index,
                location=location,
                is_large=_is_large_pad(raw_pad),
                is_active=True,
                inactive_time=0.0,
            )
            self.pads.append(pad)

        self._rebuild_groups()
        self.initialized = bool(self.pads)
        return len(self.pads)

    def initialize_from_pads(self, pads: Iterable[Any]) -> int:
        """Initialize directly from an iterable of pad-like objects."""

        self.pads = []

        for index, raw_pad in enumerate(pads):
            if isinstance(raw_pad, BoostPad):
                pad = raw_pad
                pad.index = index
            else:
                pad = BoostPad(
                    index=index,
                    location=_as_vec3(
                        _get(raw_pad, "location", "position")
                    ),
                    is_large=_is_large_pad(raw_pad),
                    is_active=bool(
                        _get(raw_pad, "is_active", "active", default=True)
                    ),
                    inactive_time=float(
                        _get(
                            raw_pad,
                            "inactive_time",
                            "timer",
                            default=0.0,
                        )
                    ),
                )

            self.pads.append(pad)

        self._rebuild_groups()
        self.initialized = bool(self.pads)
        return len(self.pads)

    def _rebuild_groups(self) -> None:
        self.small_pads = [
            pad for pad in self.pads if not pad.is_large
        ]
        self.large_pads = [
            pad for pad in self.pads if pad.is_large
        ]

    def update_from_packet(self, packet: Any) -> None:
        """Update active state and inactive timers from a GamePacket."""

        packet_pads = _get(
            packet,
            "boost_pads",
            "boostPads",
            default=[],
        )

        if not self.initialized:
            return

        for index, packet_pad in enumerate(packet_pads):
            if index >= len(self.pads):
                break

            pad = self.pads[index]
            pad.is_active = bool(
                _get(packet_pad, "is_active", "active", default=pad.is_active)
            )

            timer = _get(
                packet_pad,
                "timer",
                "inactive_time",
                default=None,
            )

            if timer is not None:
                pad.inactive_time = max(0.0, float(timer))
            elif pad.is_active:
                pad.inactive_time = 0.0

    def get_pad(self, index: int) -> BoostPad | None:
        if 0 <= index < len(self.pads):
            return self.pads[index]
        return None

    def get_small_pads(self) -> list[BoostPad]:
        return self.small_pads

    def get_large_pads(self) -> list[BoostPad]:
        return self.large_pads

    def active_pads(self) -> list[BoostPad]:
        return [pad for pad in self.pads if pad.is_active]

    def available_at(self, pad: BoostPad, arrival_time: float) -> bool:
        return (
            pad.is_active
            or pad.time_until_active
            <= arrival_time
        )


def _estimate_eta(
    car: VehicleState,
    target: Vec3,
) -> float:
    try:
        result = estimate_reachability(car, target)

        if not getattr(result, "reachable", False):
            return inf

        return float(getattr(result, "time", inf))

    except (AttributeError, TypeError, ValueError, OverflowError):
        return inf


def _route_detour(
    car: VehicleState,
    pad_location: Vec3,
    strategic_target: Vec3 | None,
) -> float:
    """Extra flat distance caused by visiting the pad first."""

    if strategic_target is None:
        return 0.0

    direct = car.location.flat_distance(strategic_target)

    via_pad = (
        car.location.flat_distance(pad_location)
        + pad_location.flat_distance(strategic_target)
    )

    return max(0.0, via_pad - direct)


def _expected_gain(
    car_boost: float,
    pad: BoostPad,
) -> tuple[float, float]:
    projected = min(100.0, car_boost + pad.boost_amount)
    return max(0.0, projected - car_boost), projected


def score_boost_pad(
    car: VehicleState,
    pad: BoostPad,
    config: BoostConfig,
    strategic_target: Vec3 | None = None,
    covering: bool = False,
    ball_in_own_half: bool = False,
    emergency: bool = False,
) -> BoostCandidate:
    """Evaluate one pad using ETA, respawn timing, route cost and role."""

    if emergency:
        return BoostCandidate(reason="emergency overrides boost")

    if car.demolished:
        return BoostCandidate(reason="car demolished")

    if car.boost >= config.stop_boost:
        return BoostCandidate(reason="boost already sufficient")

    if pad.is_large and not config.drive_to_large_pads:
        return BoostCandidate(reason="large pads disabled")

    if not pad.is_large and not config.drive_to_small_pads:
        return BoostCandidate(reason="small pads disabled")

    eta = _estimate_eta(car, pad.location)

    if eta == inf or eta < config.minimum_eta:
        return BoostCandidate(reason="pad unreachable or too close")

    if eta > config.maximum_eta:
        return BoostCandidate(reason="pad too far away")

    time_until_active = pad.time_until_active

    if time_until_active > eta + config.respawn_tolerance:
        return BoostCandidate(reason="pad will not respawn in time")

    gain, projected_boost = _expected_gain(car.boost, pad)

    if gain < config.minimum_gain:
        return BoostCandidate(reason="insufficient boost gain")

    detour = _route_detour(
        car,
        pad.location,
        strategic_target,
    )

    if covering and ball_in_own_half:
        if car.boost >= config.cover_minimum_boost:
            return BoostCandidate(
                reason="cover role: retain defensive position"
            )

        if detour > config.cover_max_detour:
            return BoostCandidate(
                reason="cover role: pad is too far off route"
            )

        if eta > config.cover_max_eta:
            return BoostCandidate(
                reason="cover role: pad arrival is too slow"
            )

    arrival_margin = eta - time_until_active

    # Lower scores are better. Large pads are useful, but not worth
    # unlimited detours or abandoning a good position.
    score = (
        eta * config.eta_weight
        + detour * config.route_detour_weight
        - gain * config.gain_weight / 100.0
        - (config.large_pad_preference if pad.is_large else 0.0)
    )

    return BoostCandidate(
        valid=True,
        pad_index=pad.index,
        location=pad.location,
        is_large=pad.is_large,
        eta=eta,
        time_until_active=time_until_active,
        arrival_margin=arrival_margin,
        distance=car.location.flat_distance(pad.location),
        route_detour=detour,
        expected_gain=gain,
        projected_boost=projected_boost,
        score=score,
        reason="reachable boost pad",
    )


def find_best_boost(
    car: VehicleState,
    tracker: BoostPadTracker,
    config: BoostConfig | None = None,
    strategic_target: Vec3 | None = None,
    covering: bool = False,
    ball_in_own_half: bool = False,
    emergency: bool = False,
    large_only: bool = False,
) -> BoostCandidate:
    """Choose the best currently reachable boost pad."""

    config = config or BoostConfig()

    if car.boost >= config.start_boost_below:
        return BoostCandidate(reason="boost above collection threshold")

    pads = (
        tracker.get_large_pads()
        if large_only
        else tracker.pads
    )

    best = BoostCandidate()
    best_score = inf

    for pad in pads:
        candidate = score_boost_pad(
            car,
            pad,
            config,
            strategic_target=strategic_target,
            covering=covering,
            ball_in_own_half=ball_in_own_half,
            emergency=emergency,
        )

        if candidate.valid and candidate.score < best_score:
            best = candidate
            best_score = candidate.score

    if not best.valid:
        best.reason = "no suitable boost pad"

    return best


class BoostManager:
    """Persistent boost-collection action.

    Call start() to select a pad, then update() each tick. The action keeps
    its chosen pad briefly and only replaces it when a clearly better
    candidate appears.
    """

    def __init__(
        self,
        tracker: BoostPadTracker | None = None,
        config: BoostConfig | None = None,
    ):
        self.tracker = tracker or BoostPadTracker()
        self.config = config or BoostConfig()

        self.drive = Drive()

        self.phase = BoostPhase.IDLE
        self.active = False
        self.finished = False

        self.candidate = BoostCandidate()
        self.target: Vec3 | None = None

        self.elapsed = 0.0
        self.commitment_remaining = 0.0
        self.time_since_refresh = 0.0

        self.reason = ""
        self._last_output = BoostOutput()

    def reset(self) -> None:
        self.drive = Drive()

        self.phase = BoostPhase.IDLE
        self.active = False
        self.finished = False

        self.candidate = BoostCandidate()
        self.target = None

        self.elapsed = 0.0
        self.commitment_remaining = 0.0
        self.time_since_refresh = 0.0

        self.reason = ""
        self._last_output = BoostOutput()

    def start(
        self,
        car: VehicleState,
        strategic_target: Vec3 | None = None,
        covering: bool = False,
        ball_in_own_half: bool = False,
        emergency: bool = False,
        large_only: bool = False,
    ) -> bool:
        self.reset()

        if not self.tracker.initialized:
            self.reason = "boost tracker not initialized"
            self.phase = BoostPhase.ABORTED
            self.finished = True
            return False

        if car.boost >= self.config.start_boost_below:
            self.reason = "boost above collection threshold"
            self.phase = BoostPhase.ABORTED
            self.finished = True
            return False

        self.candidate = find_best_boost(
            car,
            self.tracker,
            self.config,
            strategic_target=strategic_target,
            covering=covering,
            ball_in_own_half=ball_in_own_half,
            emergency=emergency,
            large_only=large_only,
        )

        if not self.candidate.valid or self.candidate.location is None:
            self.reason = self.candidate.reason
            self.phase = BoostPhase.ABORTED
            self.finished = True
            return False

        self.target = self.candidate.location
        self.phase = BoostPhase.COLLECTING
        self.active = True
        self.finished = False

        self.commitment_remaining = self.config.commitment_time
        self.reason = "boost pad selected"
        return True

    def _finish(self, reason: str, phase: BoostPhase) -> BoostOutput:
        self.phase = phase
        self.active = False
        self.finished = True
        self.reason = reason

        output = BoostOutput(
            phase=phase,
            active=False,
            finished=True,
            target=self.target,
            pad_index=self.candidate.pad_index,
            pad_is_large=self.candidate.is_large,
            eta=self.candidate.eta,
            expected_gain=self.candidate.expected_gain,
            reason=reason,
        )

        self._last_output = output
        return output

    def _refresh(
        self,
        car: VehicleState,
        strategic_target: Vec3 | None,
        covering: bool,
        ball_in_own_half: bool,
    ) -> None:
        if self.commitment_remaining > 0.0:
            return

        new_candidate = find_best_boost(
            car,
            self.tracker,
            self.config,
            strategic_target=strategic_target,
            covering=covering,
            ball_in_own_half=ball_in_own_half,
        )

        if not new_candidate.valid or new_candidate.location is None:
            return

        if (
            new_candidate.pad_index != self.candidate.pad_index
            and new_candidate.score
            >= self.candidate.score - self.config.replacement_score_margin
        ):
            return

        self.candidate = new_candidate
        self.target = new_candidate.location
        self.drive = Drive()
        self.commitment_remaining = self.config.commitment_time

    def update(
        self,
        car: VehicleState,
        dt: float = 1.0 / 120.0,
        strategic_target: Vec3 | None = None,
        covering: bool = False,
        ball_in_own_half: bool = False,
        emergency: bool = False,
    ) -> BoostOutput:
        dt = max(0.0, float(dt))

        if not self.active or self.finished:
            return self._last_output

        if emergency:
            return self._finish(
                "emergency overrides boost collection",
                BoostPhase.ABORTED,
            )

        if car.demolished:
            return self._finish(
                "car demolished",
                BoostPhase.ABORTED,
            )

        self.elapsed += dt
        self.time_since_refresh += dt
        self.commitment_remaining = max(
            0.0,
            self.commitment_remaining - dt,
        )

        if car.boost >= self.config.stop_boost:
            return self._finish(
                "boost level sufficient",
                BoostPhase.FINISHED,
            )

        if self.target is None:
            return self._finish(
                "boost target missing",
                BoostPhase.ABORTED,
            )

        pad = self.tracker.get_pad(self.candidate.pad_index)

        if pad is None:
            return self._finish(
                "selected pad no longer exists",
                BoostPhase.ABORTED,
            )

        eta = _estimate_eta(car, pad.location)

        # Party Cannon's key respawn check: if the pad is inactive and
        # will not respawn before arrival, stop pursuing it.
        if (
            not pad.is_active
            and pad.time_until_active
            > eta + self.config.respawn_tolerance
        ):
            return self._finish(
                "pad will not be active on arrival",
                BoostPhase.ABORTED,
            )

        if self.time_since_refresh >= self.config.refresh_interval:
            self._refresh(
                car,
                strategic_target,
                covering,
                ball_in_own_half,
            )
            self.time_since_refresh = 0.0

            pad = self.tracker.get_pad(self.candidate.pad_index)
            if pad is None or self.target is None:
                return self._finish(
                    "boost target lost",
                    BoostPhase.ABORTED,
                )

        distance = car.location.flat_distance(self.target)

        # A pad pickup is normally detected through the boost increase.
        # The small radius is only a fallback if the pickup packet is delayed.
        if distance <= 95.0 and pad.is_active:
            return self._finish(
                "boost pad reached",
                BoostPhase.ARRIVED,
            )

        drive_output: DriveOutput = self.drive.update(
            car,
            self.target,
            dt,
        )

        output = BoostOutput(
            phase=BoostPhase.COLLECTING,
            active=True,
            finished=False,
            throttle=drive_output.throttle,
            steer=drive_output.steer,
            boost=drive_output.boost,
            handbrake=drive_output.handbrake,
            target=self.target,
            pad_index=self.candidate.pad_index,
            pad_is_large=self.candidate.is_large,
            eta=eta,
            expected_gain=self.candidate.expected_gain,
            reason="driving to boost pad",
        )

        self._last_output = output
        return output


def create_boost_manager(
    tracker: BoostPadTracker | None = None,
    config: BoostConfig | None = None,
) -> BoostManager:
    return BoostManager(tracker, config)
