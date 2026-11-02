
"""Team roles, rotations and challenge decisions for Grid.

The teamplay layer decides which car should challenge, support or defend.

Decisions are based on:
- Estimated interception times.
- Car position relative to the ball and own goal.
- Teammate availability.
- Boost and demolition state.
- Defensive urgency.
- Role stability and challenge commitment.

This module does not issue controller inputs. It returns a strategic
decision for the agent and mechanics layers to execute.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import inf

from physics.math3d import Vec3, clamp
from physics.field import (
    goal_for_team,
    enemy_goal_for_team,
    in_own_half,
)
from physics.reachability import estimate_reachability

from util.game_state import GameState, PlayerState


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

CHALLENGE_ADVANTAGE = 0.18
CHALLENGE_DISADVANTAGE = 0.30
ROLE_SWITCH_MARGIN = 0.12

LOW_BOOST_THRESHOLD = 25.0
CRITICAL_BOOST_THRESHOLD = 10.0

DEFENSIVE_DEPTH = 1800.0
EMERGENCY_DEFENSIVE_DEPTH = 3500.0

SUPPORT_DISTANCE = 1800.0
COVER_DISTANCE = 2800.0

MIN_CHALLENGE_TIME = 0.12
MAX_CHALLENGE_TIME = 4.0

OWN_GOAL_DANGER_DISTANCE = 2400.0


# ---------------------------------------------------------------------------
# Types
# ---------------------------------------------------------------------------

class TeamRole(str, Enum):
    CHALLENGE = "challenge"
    SUPPORT = "support"
    COVER = "cover"
    RECOVER = "recover"
    KICKOFF = "kickoff"


@dataclass(slots=True)
class PlayerAssessment:
    """A player's current tactical assessment."""

    player: PlayerState
    arrival_time: float
    reachable: bool
    distance_to_ball: float
    distance_to_own_goal: float
    boost: float
    defensive_depth: float
    challenge_score: float


@dataclass(slots=True)
class TeamDecision:
    """Strategic decision for the controlled car."""

    role: TeamRole
    challenger_index: int | None
    target: Vec3
    reason: str

    should_challenge: bool = False
    should_support: bool = False
    should_cover: bool = False
    emergency: bool = False

    my_arrival_time: float = inf
    challenger_arrival_time: float = inf
    teammate_challenger_index: int | None = None

    @property
    def is_challenger(self) -> bool:
        return self.should_challenge

    @property
    def is_defending(self) -> bool:
        return self.should_cover or self.emergency


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _flat_distance(a: Vec3, b: Vec3) -> float:
    return a.flat_distance(b)


def _goal_distance(player: PlayerState, team: int) -> float:
    goal = goal_for_team(team)
    return player.location.flat_distance(goal.center)


def _defensive_depth(
    player: PlayerState,
    team: int,
) -> float:
    """Positive depth means the player is on their own side of the field."""
    own_goal = goal_for_team(team)

    if team == 0:
        return own_goal.center.y - player.location.y

    return player.location.y - own_goal.center.y


def _ball_defensive_depth(
    ball_position: Vec3,
    team: int,
) -> float:
    own_goal = goal_for_team(team)

    if team == 0:
        return own_goal.center.y - ball_position.y

    return ball_position.y - own_goal.center.y


def _attack_direction(team: int) -> Vec3:
    goal = enemy_goal_for_team(team)
    return Vec3(0.0, 1.0 if team == 0 else -1.0, 0.0)


def _support_target(
    state: GameState,
    player: PlayerState,
) -> Vec3:
    """Position behind and slightly to the side of the active challenge."""
    ball = state.ball.location
    attack = _attack_direction(state.team)

    behind = ball - attack * SUPPORT_DISTANCE

    side = -1.0 if player.location.x >= ball.x else 1.0

    return Vec3(
        clamp(behind.x + side * 450.0, -3500.0, 3500.0),
        behind.y,
        17.0,
    )


def _cover_target(
    state: GameState,
    player: PlayerState,
) -> Vec3:
    """Keep the cover player between the ball and their own goal."""
    ball = state.ball.location
    own_goal = goal_for_team(state.team)

    direction = ball.flat_direction_to(own_goal.center)

    if direction.length_squared() < 1e-6:
        direction = Vec3(0.0, -1.0 if state.team == 0 else 1.0, 0.0)

    target = ball + direction * COVER_DISTANCE

    return Vec3(
        clamp(target.x, -3800.0, 3800.0),
        clamp(target.y, -4700.0, 4700.0),
        17.0,
    )


def _recovery_target(
    state: GameState,
    player: PlayerState,
) -> Vec3:
    """Choose a safe ground position while the car recovers."""
    own_goal = goal_for_team(state.team)
    ball = state.ball.location

    if _ball_defensive_depth(ball, state.team) > 0.0:
        target = ball + (own_goal.center - ball).flat_normalized() * 1000.0
    else:
        target = ball - _attack_direction(state.team) * 1000.0

    return Vec3(
        clamp(target.x, -3900.0, 3900.0),
        clamp(target.y, -4900.0, 4900.0),
        17.0,
    )


# ---------------------------------------------------------------------------
# Arrival-time assessment
# ---------------------------------------------------------------------------

def assess_player(
    state: GameState,
    player: PlayerState,
) -> PlayerAssessment:
    """Estimate a player's ability to reach the current ball position."""
    ball = state.ball.location

    distance = _flat_distance(player.location, ball)
    own_goal_distance = _goal_distance(player, state.team)
    depth = _defensive_depth(player, state.team)

    if player.demolished:
        return PlayerAssessment(
            player=player,
            arrival_time=inf,
            reachable=False,
            distance_to_ball=distance,
            distance_to_own_goal=own_goal_distance,
            boost=player.boost,
            defensive_depth=depth,
            challenge_score=-inf,
        )

    result = estimate_reachability(
        player.car,
        ball,
        use_boost=True,
        target_radius=125.0,
    )

    arrival = result.time if result.reachable else inf

    # Reward a good arrival time, but avoid making distance the only
    # factor. The car's defensive position and boost reserve matter too.
    time_score = (
        1.0 / max(arrival, 0.15)
        if result.reachable
        else 0.0
    )

    boost_factor = 0.85 + 0.15 * clamp(player.boost / 100.0, 0.0, 1.0)

    # A player already deep in their own half receives a small penalty
    # when the ball is safely upfield, preserving defensive structure.
    ball_depth = _ball_defensive_depth(ball, state.team)

    position_factor = 1.0

    if depth > DEFENSIVE_DEPTH and ball_depth < -500.0:
        position_factor *= 0.82

    challenge_score = time_score * boost_factor * position_factor

    return PlayerAssessment(
        player=player,
        arrival_time=arrival,
        reachable=result.reachable,
        distance_to_ball=distance,
        distance_to_own_goal=own_goal_distance,
        boost=player.boost,
        defensive_depth=depth,
        challenge_score=challenge_score,
    )


def assess_team(
    state: GameState,
) -> list[PlayerAssessment]:
    """Assess all living players on the controlled car's team."""
    assessments = [
        assess_player(state, player)
        for player in state.all_teammates
    ]

    return sorted(
        assessments,
        key=lambda assessment: assessment.arrival_time,
    )


# ---------------------------------------------------------------------------
# Defensive urgency
# ---------------------------------------------------------------------------

def defensive_emergency(state: GameState) -> bool:
    """Return True when the ball is close enough to the own goal to demand cover."""
    ball = state.ball.location
    own_goal = goal_for_team(state.team)

    goal_distance = ball.flat_distance(own_goal.center)
    depth = _ball_defensive_depth(ball, state.team)

    moving_toward_goal = (
        ball.velocity.dot(
            own_goal.center.flat_direction_to(ball).normalized()
        ) < -150.0
    )

    deep_and_low = (
        depth > EMERGENCY_DEFENSIVE_DEPTH
        and ball.z < 500.0
    )

    close_and_threatening = (
        goal_distance < OWN_GOAL_DANGER_DISTANCE
        and ball.z < 700.0
        and moving_toward_goal
    )

    return deep_and_low or close_and_threatening


# ---------------------------------------------------------------------------
# Main team decision
# ---------------------------------------------------------------------------

def choose_team_role(
    state: GameState,
    previous_role: TeamRole | None = None,
    previous_challenger_index: int | None = None,
) -> TeamDecision:
    """Choose the controlled car's role for the current frame.

    previous_role and previous_challenger_index can be supplied by the
    agent to reduce role oscillation between consecutive frames.
    """
    me = state.me

    if me.demolished:
        return TeamDecision(
            role=TeamRole.RECOVER,
            challenger_index=None,
            target=Vec3(0.0, 0.0, 17.0),
            reason="Demolished",
        )

    if state.is_kickoff:
        return TeamDecision(
            role=TeamRole.KICKOFF,
            challenger_index=me.index,
            target=state.ball.location,
            reason="Kickoff formation",
            should_challenge=True,
            my_arrival_time=0.0,
            challenger_arrival_time=0.0,
        )

    if not me.on_ground and me.location.z > 350.0:
        return TeamDecision(
            role=TeamRole.RECOVER,
            challenger_index=None,
            target=_recovery_target(state, me),
            reason="Airborne recovery",
        )

    assessments = assess_team(state)

    my_assessment = next(
        (
            assessment
            for assessment in assessments
            if assessment.player.index == me.index
        ),
        None,
    )

    if my_assessment is None:
        return TeamDecision(
            role=TeamRole.COVER,
            challenger_index=None,
            target=_cover_target(state, me),
            reason="Missing self assessment",
            should_cover=True,
        )

    living_teammates = [
        assessment
        for assessment in assessments
        if assessment.reachable
    ]

    if not living_teammates:
        return TeamDecision(
            role=TeamRole.COVER,
            challenger_index=None,
            target=_cover_target(state, me),
            reason="No reachable teammate",
            should_cover=True,
            my_arrival_time=my_assessment.arrival_time,
        )

    # A car with a meaningful arrival advantage is the challenger.
    challenger = living_teammates[0]

    # Keep the current challenger if the new candidate is only marginally
    # faster. This prevents role changes caused by tiny ETA fluctuations.
    if previous_challenger_index is not None:
        previous = next(
            (
                assessment
                for assessment in living_teammates
                if assessment.player.index == previous_challenger_index
            ),
            None,
        )

        if previous is not None:
            if (
                previous.arrival_time
                <= challenger.arrival_time + ROLE_SWITCH_MARGIN
            ):
                challenger = previous

    # If the ball is a serious threat, the closest reachable car may
    # need to challenge regardless of normal rotation preferences.
    emergency = defensive_emergency(state)

    if emergency:
        challenger = living_teammates[0]

    is_challenger = challenger.player.index == me.index

    teammate_challenger = (
        challenger.player.index
        if challenger.player.index != me.index
        else None
    )

    # If the teammate is only slightly faster, retain the challenge when
    # our car is already moving into the play. This avoids indecisive swaps.
    if (
        is_challenger
        and previous_role == TeamRole.CHALLENGE
        and previous_challenger_index == me.index
    ):
        next_player = next(
            (
                assessment
                for assessment in living_teammates
                if assessment.player.index != me.index
            ),
            None,
        )

        if (
            next_player is not None
            and next_player.arrival_time
            >= my_assessment.arrival_time - CHALLENGE_ADVANTAGE
        ):
            challenger = my_assessment
            is_challenger = True
            teammate_challenger = None

    # The challenger attacks the ball. Teammates support or cover based
    # on their position relative to the own goal and the current ball.
    if is_challenger:
        return TeamDecision(
            role=TeamRole.CHALLENGE,
            challenger_index=me.index,
            target=state.ball.location,
            reason=(
                "Emergency challenge"
                if emergency
                else "Best team arrival time"
            ),
            should_challenge=True,
            emergency=emergency,
            my_arrival_time=my_assessment.arrival_time,
            challenger_arrival_time=challenger.arrival_time,
        )

    ball_depth = _ball_defensive_depth(
        state.ball.location,
        state.team,
    )

    my_depth = my_assessment.defensive_depth

    # The deepest teammate remains cover when the ball is upfield.
    if ball_depth < -500.0 and my_depth > DEFENSIVE_DEPTH:
        return TeamDecision(
            role=TeamRole.COVER,
            challenger_index=challenger.player.index,
            target=_cover_target(state, me),
            reason="Deepest player protects own half",
            should_cover=True,
            emergency=emergency,
            my_arrival_time=my_assessment.arrival_time,
            challenger_arrival_time=challenger.arrival_time,
            teammate_challenger_index=teammate_challenger,
        )

    # When the ball is in the defensive half, the non-challenger should
    # support the play rather than abandoning it to collect boost.
    if ball_depth > 0.0:
        return TeamDecision(
            role=TeamRole.SUPPORT,
            challenger_index=challenger.player.index,
            target=_support_target(state, me),
            reason="Support defensive-half challenge",
            should_support=True,
            emergency=emergency,
            my_arrival_time=my_assessment.arrival_time,
            challenger_arrival_time=challenger.arrival_time,
            teammate_challenger_index=teammate_challenger,
        )

    # In the attacking half, the non-challenger supports from behind.
    return TeamDecision(
        role=TeamRole.SUPPORT,
        challenger_index=challenger.player.index,
        target=_support_target(state, me),
        reason="Second man support",
        should_support=True,
        emergency=emergency,
        my_arrival_time=my_assessment.arrival_time,
        challenger_arrival_time=challenger.arrival_time,
        teammate_challenger_index=teammate_challenger,
    )


def should_challenge(
    state: GameState,
    previous_challenger_index: int | None = None,
) -> bool:
    """Convenience function for the agent."""
    decision = choose_team_role(
        state,
        previous_challenger_index=previous_challenger_index,
    )

    return decision.should_challenge


def get_team_decision(
    state: GameState,
    previous_role: TeamRole | None = None,
    previous_challenger_index: int | None = None,
) -> TeamDecision:
    """Alias used by the agent layer."""
    return choose_team_role(
        state,
        previous_role=previous_role,
        previous_challenger_index=previous_challenger_index,
    )
