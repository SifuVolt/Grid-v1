
"""RLBot v5 game-packet conversion for Grid.

Converts incoming game data into Grid's internal state objects.

This module deliberately avoids importing RLBot packet classes directly.
That keeps the physics and mechanics modules independent of the RLBot
runtime and makes packet conversion easier to test.

Primary output:
- GameState
- VehicleState for every player
- BallState
- Trajectory
- BoostPadState for each known boost pad

Packet fields are read defensively because optional fields and generated
packet wrappers can differ between RLBot interface versions.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from math import isfinite
from typing import Any, Iterable

from physics.math3d import Vec3, ZERO
from physics.vehicle import VehicleState, make_vehicle_state
from physics.trajectory import (
    BallState,
    Trajectory,
    BALL_RADIUS,
)
from physics.field import (
    SMALL_BOOST_PADS,
    LARGE_BOOST_PADS,
)


# ---------------------------------------------------------------------------
# Packet-reading helpers
# ---------------------------------------------------------------------------

def read_member(obj: Any, name: str, default=None):
    """Read an attribute or dictionary key safely."""
    if obj is None:
        return default

    if isinstance(obj, dict):
        return obj.get(name, default)

    return getattr(obj, name, default)


def read_number(
    obj: Any,
    name: str,
    default: float = 0.0,
) -> float:
    value = read_member(obj, name, default)

    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return default

    return value if isfinite(value) else default


def read_bool(obj: Any, name: str, default: bool = False) -> bool:
    value = read_member(obj, name, default)

    if value is None:
        return default

    return bool(value)


def read_sequence(obj: Any, name: str) -> list:
    value = read_member(obj, name, None)

    if value is None:
        return []

    try:
        return list(value)
    except (TypeError, ValueError):
        return []


def read_vector(obj: Any, name: str, default: Vec3 = ZERO) -> Vec3:
    value = read_member(obj, name, None)

    if value is None:
        return default

    if isinstance(value, Vec3):
        return value

    if isinstance(value, dict):
        return Vec3(
            read_number(value, "x"),
            read_number(value, "y"),
            read_number(value, "z"),
        )

    if all(hasattr(value, component) for component in ("x", "y", "z")):
        return Vec3(
            read_number(value, "x"),
            read_number(value, "y"),
            read_number(value, "z"),
        )

    try:
        return Vec3(
            float(value[0]),
            float(value[1]),
            float(value[2]),
        )
    except (TypeError, ValueError, IndexError, KeyError, OverflowError):
        return default


def read_rotation(physics: Any) -> tuple[float, float, float]:
    """Read (pitch, yaw, roll) in radians."""
    rotation = read_member(physics, "rotation", None)

    if rotation is None:
        return 0.0, 0.0, 0.0

    return (
        read_number(rotation, "pitch"),
        read_number(rotation, "yaw"),
        read_number(rotation, "roll"),
    )


def read_game_time(packet: Any) -> float:
    """Read elapsed game time from available packet metadata."""
    match_info = read_member(packet, "match_info", None)

    candidates = (
        (match_info, "seconds_elapsed"),
        (match_info, "game_seconds"),
        (match_info, "elapsed_seconds"),
        (packet, "game_seconds"),
        (packet, "seconds_elapsed"),
    )

    for obj, name in candidates:
        value = read_member(obj, name, None)

        if value is None:
            continue

        try:
            result = float(value)
        except (TypeError, ValueError, OverflowError):
            continue

        if isfinite(result):
            return result

    return 0.0


# ---------------------------------------------------------------------------
# Internal state objects
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class BoostPadState:
    """Current state of one boost pad."""

    index: int
    location: Vec3
    active: bool = True
    timer: float = 0.0
    is_large: bool = False

    @property
    def available(self) -> bool:
        return self.active or self.timer <= 0.0


@dataclass(slots=True)
class PlayerState:
    """A player and their converted vehicle state."""

    index: int
    name: str
    team: int
    car: VehicleState
    is_local: bool = False

    @property
    def location(self) -> Vec3:
        return self.car.location

    @property
    def velocity(self) -> Vec3:
        return self.car.velocity

    @property
    def boost(self) -> float:
        return self.car.boost

    @property
    def demolished(self) -> bool:
        return self.car.demolished

    @property
    def on_ground(self) -> bool:
        return self.car.on_ground


@dataclass(slots=True)
class GameState:
    """Complete internal game snapshot for one bot frame."""

    frame: int
    game_time: float
    player_index: int
    team: int

    me: PlayerState
    teammates: list[PlayerState]
    opponents: list[PlayerState]

    ball: BallState
    trajectory: Trajectory

    boost_pads: list[BoostPadState]

    is_kickoff: bool = False
    is_round_active: bool = True
    is_match_ended: bool = False
    match_phase: str = "unknown"

    @property
    def players(self) -> list[PlayerState]:
        return [self.me, *self.teammates, *self.opponents]

    @property
    def all_teammates(self) -> list[PlayerState]:
        return [self.me, *self.teammates]

    @property
    def all_opponents(self) -> list[PlayerState]:
        return self.opponents

    @property
    def alive_teammates(self) -> list[PlayerState]:
        return [
            player
            for player in self.all_teammates
            if not player.demolished
        ]

    @property
    def alive_opponents(self) -> list[PlayerState]:
        return [
            player
            for player in self.opponents
            if not player.demolished
        ]

    @property
    def active_boost_pads(self) -> list[BoostPadState]:
        return [
            pad
            for pad in self.boost_pads
            if pad.active
        ]

    def teammate_by_index(self, index: int) -> PlayerState | None:
        for player in self.teammates:
            if player.index == index:
                return player
        return None

    def opponent_by_index(self, index: int) -> PlayerState | None:
        for player in self.opponents:
            if player.index == index:
                return player
        return None

    @classmethod
    def from_packet(
        cls,
        packet: Any,
        player_index: int,
        ball_prediction: Any = None,
        frame: int = 0,
    ) -> GameState:
        """Convert an RLBot packet into a Grid GameState.

        player_index must be the index of this bot in packet.players.

        ball_prediction may be supplied separately if the RLBot interface
        exposes prediction independently of the game packet.
        """
        raw_players = read_sequence(packet, "players")

        if not raw_players:
            raise ValueError(
                "Game packet contains no players. "
                "Check that the RLBot v5 packet is being passed."
            )

        if not 0 <= player_index < len(raw_players):
            raise IndexError(
                f"Player index {player_index} is outside "
                f"the packet player list ({len(raw_players)} players)."
            )

        converted_players: list[PlayerState] = []

        for index, raw_player in enumerate(raw_players):
            converted_players.append(
                player_from_packet(
                    raw_player,
                    index=index,
                    is_local=(index == player_index),
                )
            )

        me = converted_players[player_index]
        team = me.team

        teammates = [
            player
            for player in converted_players
            if player.index != player_index and player.team == team
        ]

        opponents = [
            player
            for player in converted_players
            if player.team != team
        ]

        raw_balls = read_sequence(packet, "balls")

        if not raw_balls:
            raise ValueError(
                "Game packet contains no balls. "
                "Grid currently expects a Soccar ball."
            )

        ball = BallState.from_packet_ball(raw_balls[0])

        game_time = read_game_time(packet)

        trajectory = Trajectory.from_ball_or_prediction(
            ball=ball,
            prediction=ball_prediction,
            current_game_seconds=game_time,
        )

        boost_pads = parse_boost_pads(packet)

        match_info = read_member(packet, "match_info", None)

        is_round_active = read_bool(
            match_info,
            "is_round_active",
            True,
        )

        is_match_ended = read_bool(
            match_info,
            "is_match_ended",
            False,
        )

        match_phase = str(
            read_member(match_info, "match_phase", "unknown")
        )

        is_kickoff = detect_kickoff(
            ball,
            converted_players,
            is_round_active,
        )

        return cls(
            frame=frame,
            game_time=game_time,
            player_index=player_index,
            team=team,
            me=me,
            teammates=teammates,
            opponents=opponents,
            ball=ball,
            trajectory=trajectory,
            boost_pads=boost_pads,
            is_kickoff=is_kickoff,
            is_round_active=is_round_active,
            is_match_ended=is_match_ended,
            match_phase=match_phase,
        )


# ---------------------------------------------------------------------------
# Player conversion
# ---------------------------------------------------------------------------

def player_from_packet(
    raw_player: Any,
    index: int,
    is_local: bool = False,
) -> PlayerState:
    """Convert one RLBot player packet into PlayerState."""
    physics = read_member(raw_player, "physics", None)

    if physics is None:
        physics = raw_player

    location = read_vector(physics, "location")
    velocity = read_vector(physics, "velocity")
    angular_velocity = read_vector(physics, "angular_velocity")

    pitch, yaw, roll = read_rotation(physics)

    boost = read_number(
        raw_player,
        "boost",
        read_number(raw_player, "boost_amount", 0.0),
    )

    team = int(read_number(raw_player, "team", 0))

    # In RLBot player data, has_wheel_contact is the ground-contact flag.
    # on_ground is accepted as a fallback for alternate wrappers.
    wheel_contact = read_member(
        raw_player,
        "has_wheel_contact",
        None,
    )

    if wheel_contact is None:
        wheel_contact = read_member(
            raw_player,
            "on_ground",
            True,
        )

    demolished = read_bool(
        raw_player,
        "is_demolished",
        read_bool(raw_player, "demolished", False),
    )

    supersonic = read_bool(raw_player, "is_supersonic", False)
    is_bot = read_bool(raw_player, "is_bot", False)

    car = make_vehicle_state(
        location=location,
        velocity=velocity,
        rotation=(pitch, yaw, roll),
        boost=boost,
        on_ground=bool(wheel_contact),
        team=team,
        index=index,
        angular_velocity=angular_velocity,
        demolished=demolished,
    )

    car.has_wheel_contact = bool(wheel_contact)
    car.supersonic = supersonic
    car.is_bot = is_bot

    name = str(read_member(raw_player, "name", f"Player {index}"))

    return PlayerState(
        index=index,
        name=name,
        team=team,
        car=car,
        is_local=is_local,
    )


# ---------------------------------------------------------------------------
# Boost pad conversion
# ---------------------------------------------------------------------------

def parse_boost_pads(packet: Any) -> list[BoostPadState]:
    """Convert packet boost-pad states using known Soccar pad positions.

    RLBot commonly supplies boost-pad activity and timer arrays separately
    from their static map locations. If those arrays are absent, pads are
    returned as active so the strategic layer can still use their locations.

    Pad-location ordering must match the standard Soccar boost-pad ordering
    used by the running RLBot interface.
    """
    raw_pads = read_sequence(packet, "boost_pads")

    if not raw_pads:
        return []

    pads: list[BoostPadState] = []

    for index, raw_pad in enumerate(raw_pads):
        if index >= len(SMALL_BOOST_PADS):
            break

        location = SMALL_BOOST_PADS[index]
        active = True
        timer = 0.0

        if isinstance(raw_pad, bool):
            active = raw_pad
        else:
            active = read_bool(
                raw_pad,
                "is_active",
                read_bool(raw_pad, "active", True),
            )

            timer = read_number(
                raw_pad,
                "timer",
                read_number(raw_pad, "respawn_timer", 0.0),
            )

        is_large = any(
            location.distance(large_location) < 1.0
            for large_location in LARGE_BOOST_PADS
        )

        pads.append(
            BoostPadState(
                index=index,
                location=location,
                active=active,
                timer=timer,
                is_large=is_large,
            )
        )

    return pads


# ---------------------------------------------------------------------------
# Match-state helpers
# ---------------------------------------------------------------------------

def detect_kickoff(
    ball: BallState,
    players: Iterable[PlayerState],
    is_round_active: bool = True,
) -> bool:
    """Detect a kickoff-like state from the ball and player positions.

    This is a fallback heuristic. The agent can later use explicit
    match-phase and kickoff events from the RLBot v5 interface.
    """
    if not is_round_active:
        return False

    ball_near_center = (
        abs(ball.location.x) < 20.0
        and abs(ball.location.y) < 20.0
        and abs(ball.location.z - BALL_RADIUS) < 35.0
        and ball.velocity.length() < 30.0
    )

    if not ball_near_center:
        return False

    alive_players = [
        player
        for player in players
        if not player.demolished
    ]

    if not alive_players:
        return False

    # At kickoff, players are generally spread around the centre circle.
    # Requiring multiple cars away from the ball helps avoid treating every
    # stationary centre-ball moment as a kickoff.
    cars_away = sum(
        player.location.flat_distance(ball.location) > 1500.0
        for player in alive_players
    )

    return cars_away >= min(2, len(alive_players))


def get_local_player(
    packet: Any,
    player_index: int,
) -> PlayerState:
    raw_players = read_sequence(packet, "players")

    if not 0 <= player_index < len(raw_players):
        raise IndexError(
            f"Invalid player index {player_index}; "
            f"packet has {len(raw_players)} players."
        )

    return player_from_packet(
        raw_players[player_index],
        index=player_index,
        is_local=True,
    )


def get_ball(packet: Any) -> BallState:
    raw_balls = read_sequence(packet, "balls")

    if not raw_balls:
        raise ValueError("No ball found in packet.")

    return BallState.from_packet_ball(raw_balls[0])
