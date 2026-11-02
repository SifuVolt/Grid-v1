
"""Grid kickoff mechanics.

Handles kickoff responsibility, positioning and approach.

The kickoff action is persistent and should be retained by the agent
between frames. Once the approach starts, it remains committed until
contact, timeout or an explicit emergency interruption.

The actual speedflip sequence is kept separate so it can be added
without making the basic kickoff dependent on an unfinished mechanic.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import inf

from physics.math3d import Vec3, clamp
from physics.field import (
    goal_for_team,
    enemy_goal_for_team,
)
from physics.reachability import estimate_reachability
from physics.vehicle import VehicleState

from mechanics.drive import Drive, DriveConfig, DriveOutput, DriveStatus
from util.game_state import GameState, PlayerState


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

KICKOFF_TIMEOUT = 5.0
KICKOFF_CONTACT_RADIUS = 170.0
KICKOFF_COMMITMENT_TIME = 0.30

KICKOFF_MAX_SPEED = 2300.0
KICKOFF_BOOST_THRESHOLD = 5.0

KICKOFF_TARGET_OFFSET = 90.0
KICKOFF_APPROACH_OFFSET = 450.0

LEFT_GOES_TOLERANCE = 100.0
TEAMMATE_ARRIVAL_ADVANTAGE = 0.18


class KickoffPhase(str, Enum):
    IDLE = "idle"
    WAITING = "waiting"
    APPROACH = "approach"
    CONTACT = "contact"
    COMPLETE = "complete"
    ABORTED = "aborted"


# ---------------------------------------------------------------------------
# Kickoff responsibility
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class KickoffAssignment:
    should_take: bool
    taker_index: int | None
    reason: str
    my_arrival_time: float = inf
    teammate_arrival_time: float = inf


def _kickoff_side(player: PlayerState) -> float:
    """Return the player's lateral kickoff coordinate."""
    return player.location.x


def _kickoff_arrival(player: PlayerState, ball: Vec3) -> float:
    if player.demolished:
        return inf

    result = estimate_reachability(
        player.car,
        ball,
        use_boost=True,
        target_radius=KICKOFF_CONTACT_RADIUS,
    )

    return result.time if result.reachable else inf


def choose_kickoff_taker(state: GameState) -> KickoffAssignment:
    """Choose the kickoff taker using arrival time and left-goes.

    The fastest reachable teammate takes the kickoff. If the arrival
    estimates are effectively tied, the left-most car takes it.
    """
    players = [
        player
        for player in state.all_teammates
        if not player.demolished
    ]

    if not players:
        return KickoffAssignment(
            should_take=False,
            taker_index=None,
            reason="No living teammates",
        )

    arrivals = [
        (player, _kickoff_arrival(player, state.ball.location))
        for player in players
    ]

    arrivals.sort(key=lambda item: item[1])

    fastest_player, fastest_time = arrivals[0]

    if fastest_time == inf:
        return KickoffAssignment(
            should_take=False,
            taker_index=None,
            reason="No reachable kickoff player",
        )

    tied = [
        (player, arrival)
        for player, arrival in arrivals
        if arrival <= fastest_time + TEAMMATE_ARRIVAL_ADVANTAGE
    ]

    if len(tied) > 1:
        # Standard left-goes convention: the player with the more
        # negative X coordinate is the left-most player.
        tied.sort(key=lambda item: item[0].location.x)
        taker, taker_time = tied[0]
        reason = "Left-goes tie-break"
    else:
        taker, taker_time = fastest_player, fastest_time
        reason = "Fastest arrival"

    my_time = next(
        (
            arrival
            for player, arrival in arrivals
            if player.index == state.me.index
        ),
        inf,
    )

    teammate_time = min(
        (
            arrival
            for player, arrival in arrivals
            if player.index != state.me.index
        ),
        default=inf,
    )

    return KickoffAssignment(
        should_take=taker.index == state.me.index,
        taker_index=taker.index,
        reason=reason,
        my_arrival_time=my_time,
        teammate_arrival_time=teammate_time,
    )


# ---------------------------------------------------------------------------
# Kickoff geometry
# ---------------------------------------------------------------------------

def kickoff_contact_target(
    state: GameState,
    car: VehicleState,
) -> Vec3:
    """Return a slightly offset target for controlled kickoff contact."""
    ball = state.ball.location

    enemy_goal = enemy_goal_for_team(state.team)
    attack_direction = ball.flat_direction_to(enemy_goal.center)

    if attack_direction.length_squared() < 1e-6:
        attack_direction = Vec3(
            0.0,
            1.0 if state.team == 0 else -1.0,
            0.0,
        )

    lateral = Vec3(
        -attack_direction.y,
        attack_direction.x,
        0.0,
    )

    # Offset away from the centre of the ball based on the car's spawn
    # side. This gives the car a slightly angled approach to the 50/50.
    side = -1.0 if car.location.x < 0.0 else 1.0

    return Vec3(
        ball.x + lateral.x * KICKOFF_TARGET_OFFSET * side,
        ball.y + lateral.y * KICKOFF_TARGET_OFFSET * side,
        ball.z,
    )


def kickoff_approach_target(
    state: GameState,
    car: VehicleState,
) -> Vec3:
    """Choose a point behind the ball along the kickoff approach."""
    ball = state.ball.location

    direction = car.location.flat_direction_to(ball)

    if direction.length_squared() < 1e-6:
        direction = Vec3(
            0.0,
            1.0 if state.team == 0 else -1.0,
            0.0,
        )

    target = ball - direction * KICKOFF_APPROACH_OFFSET

    return Vec3(
        clamp(target.x, -3800.0, 3800.0),
        clamp(target.y, -4800.0, 4800.0),
        17.0,
    )


# ---------------------------------------------------------------------------
# Kickoff action
# ---------------------------------------------------------------------------

@dataclass(slots=True)
class Kickoff:
    """Persistent kickoff action for the controlled car."""

    phase: KickoffPhase = KickoffPhase.IDLE

    elapsed: float = 0.0
    phase_elapsed: float = 0.0

    taker_index: int | None = None
    committed: bool = False

    drive: Drive | None = None

    last_target: Vec3 | None = None

    def start(
        self,
        state: GameState,
        assignment: KickoffAssignment | None = None,
    ) -> None:
        """Begin kickoff preparation."""
        if assignment is None:
            assignment = choose_kickoff_taker(state)

        self.taker_index = assignment.taker_index
        self.elapsed = 0.0
        self.phase_elapsed = 0.0
        self.committed = True

        if assignment.should_take:
            self.phase = KickoffPhase.APPROACH
            self.drive = Drive(
                config=DriveConfig(
                    arrival_radius=KICKOFF_CONTACT_RADIUS,
                    target_lock_time=0.10,
                    max_speed=KICKOFF_MAX_SPEED,
                    boost_threshold=KICKOFF_BOOST_THRESHOLD,
                    allow_boost=True,
                    allow_handbrake=False,
                    allow_reverse=False,
                    stop_on_arrival=False,
                )
            )
        else:
            self.phase = KickoffPhase.WAITING
            self.drive = None

    def reset(self) -> None:
        self.phase = KickoffPhase.IDLE
        self.elapsed = 0.0
        self.phase_elapsed = 0.0
        self.taker_index = None
        self.committed = False
        self.drive = None
        self.last_target = None

    def is_finished(self) -> bool:
        return self.phase in (
            KickoffPhase.COMPLETE,
            KickoffPhase.ABORTED,
        )

    def interrupt(self) -> None:
        """Explicitly interrupt kickoff, for example for a critical save."""
        if self.drive is not None:
            self.drive.interrupt()

        self.phase = KickoffPhase.ABORTED
        self.committed = False

    def update(
        self,
        state: GameState,
        dt: float = 1.0 / 120.0,
        *,
        emergency: bool = False,
    ) -> DriveOutput:
        """Advance the kickoff action."""
        dt = clamp(dt, 0.001, 0.1)

        if self.phase == KickoffPhase.IDLE:
            self.start(state)

        if self.is_finished():
            return DriveOutput.neutral(
                DriveStatus.ARRIVED
                if self.phase == KickoffPhase.COMPLETE
                else DriveStatus.INTERRUPTED
            )

        if emergency:
            self.interrupt()
            return DriveOutput.neutral(DriveStatus.INTERRUPTED)

        self.elapsed += dt
        self.phase_elapsed += dt

        if self.elapsed > KICKOFF_TIMEOUT:
            self.phase = KickoffPhase.COMPLETE
            return DriveOutput.neutral(DriveStatus.ARRIVED)

        if not state.is_kickoff:
            self.phase = KickoffPhase.COMPLETE
            return DriveOutput.neutral(DriveStatus.ARRIVED)

        if self.phase == KickoffPhase.WAITING:
            return self._update_waiting(state)

        if self.phase == KickoffPhase.APPROACH:
            return self._update_approach(state, dt)

        if self.phase == KickoffPhase.CONTACT:
            return self._update_contact(state)

        return DriveOutput.neutral()

    def _update_waiting(
        self,
        state: GameState,
    ) -> DriveOutput:
        """Non-takers hold a useful cheat/support position."""
        ball = state.ball.location
        car = state.me.car

        direction = car.location.flat_direction_to(ball)

        if direction.length_squared() < 1e-6:
            direction = Vec3(
                0.0,
                1.0 if state.team == 0 else -1.0,
                0.0,
            )

        target = ball - direction * 1000.0

        self.last_target = target

        if self.drive is None:
            self.drive = Drive(
                config=DriveConfig(
                    arrival_radius=180.0,
                    target_lock_time=0.15,
                    max_speed=1500.0,
                    allow_boost=False,
                    allow_handbrake=False,
                    allow_reverse=False,
                )
            )

        return self.drive.update(
            car,
            target,
            allow_boost=False,
            max_speed=1500.0,
        )

    def _update_approach(
        self,
        state: GameState,
        dt: float,
    ) -> DriveOutput:
        """Drive toward the kickoff ball."""
        car = state.me.car
        ball = state.ball.location

        distance = car.location.flat_distance(ball)

        if distance <= KICKOFF_CONTACT_RADIUS:
            self.phase = KickoffPhase.CONTACT
            self.phase_elapsed = 0.0
            return self._update_contact(state)

        target = kickoff_contact_target(state, car)
        self.last_target = target

        if self.drive is None:
            self.drive = Drive(
                config=DriveConfig(
                    arrival_radius=KICKOFF_CONTACT_RADIUS,
                    target_lock_time=0.08,
                    max_speed=KICKOFF_MAX_SPEED,
                    boost_threshold=KICKOFF_BOOST_THRESHOLD,
                    allow_boost=True,
                    allow_handbrake=False,
                    allow_reverse=False,
                    stop_on_arrival=False,
                )
            )

        # Keep boost available for the straight approach. The drive
        # controller still suppresses boost when badly misaligned.
        return self.drive.update(
            car,
            target,
            dt,
            allow_boost=True,
            max_speed=KICKOFF_MAX_SPEED,
            force_target=False,
        )

    def _update_contact(
        self,
        state: GameState,
    ) -> DriveOutput:
        """Maintain forward pressure through the initial 50/50."""
        car = state.me.car
        ball = state.ball.location

        if car.location.flat_distance(ball) > 550.0:
            self.phase = KickoffPhase.COMPLETE
            return DriveOutput.neutral(DriveStatus.ARRIVED)

        direction = car.location.flat_direction_to(ball)

        if direction.length_squared() < 1e-6:
            direction = car.forward.flat_normalized()

        target = ball + direction * 300.0

        if self.drive is None:
            self.drive = Drive(
                config=DriveConfig(
                    arrival_radius=80.0,
                    target_lock_time=0.05,
                    max_speed=KICKOFF_MAX_SPEED,
                    allow_boost=True,
                    allow_handbrake=False,
                    allow_reverse=False,
                    stop_on_arrival=False,
                )
            )

        return self.drive.update(
            car,
            target,
            allow_boost=True,
            max_speed=KICKOFF_MAX_SPEED,
            force_target=True,
        )


# ---------------------------------------------------------------------------
# Convenience helpers
# ---------------------------------------------------------------------------

def should_take_kickoff(state: GameState) -> bool:
    return choose_kickoff_taker(state).should_take


def create_kickoff(
    state: GameState,
) -> Kickoff:
    action = Kickoff()
    action.start(state)
    return action
