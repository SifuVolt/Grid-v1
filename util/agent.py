"""Grid RLBot v5 agent.

This is the integration pass for Grid's existing mechanics. The agent now
routes the live game loop through persistent kickoff, save, shot, aerial,
recovery, teamplay and boost systems instead of only calling ordinary Drive.

The mechanics remain separate modules; this file owns their lifecycle and
priority ordering.
"""

from __future__ import annotations

from inspect import signature
from typing import Any

from rlbot.flat import ControllerState, GamePacket
from rlbot.managers import Bot

from physics.math3d import Vec3
from physics.vehicle import VehicleState
from physics.trajectory import BallState, Trajectory
from physics.field import in_own_half

from mechanics.drive import Drive
from mechanics.saves import Save, SaveOutput
from mechanics.boost import BoostConfig, BoostManager, BoostPadTracker
from mechanics.kickoff import Kickoff
from mechanics.recovery import Recovery
from mechanics.ground_shot import GroundShot
from mechanics.jump_shot import JumpShot
from mechanics.double_jump_shot import DoubleJumpShot
from mechanics.aerial import Aerial
from mechanics.shot_selection import ShotType, find_shot

from util.action_manager import (
    ActionManager,
    ActionPriority,
    make_request,
)
from util.debug import Debug, DebugConfig
from util.game_state import GameState
from util.teamplay import TeamRole, get_team_decision


SHOT_SCAN_INTERVAL = 0.22
SHOT_COOLDOWN = 0.30


class GridAgent(Bot):
    """Grid's RLBot v5 entry point and mechanic coordinator."""

    def __init__(self, default_agent_id: str | None = None):
        super().__init__(default_agent_id)
        self.debug = Debug(DebugConfig())
        self._reset_match_systems()
        self.initialized = False

    def _reset_match_systems(self) -> None:
        self.drive = Drive()
        self.save = Save()
        self.kickoff = Kickoff()
        self.recovery = Recovery()

        self.boost_tracker = BoostPadTracker()
        self.boost = BoostManager(
            self.boost_tracker,
            BoostConfig(
                start_boost_below=35.0,
                stop_boost=85.0,
                maximum_eta=4.5,
            ),
        )

        self.actions = ActionManager()
        self.ground_shot = GroundShot()
        self.jump_shot = JumpShot()
        self.double_jump_shot = DoubleJumpShot()
        self.aerial = Aerial()

        self.frame = 0
        self.last_game_time: float | None = None
        self.current_game_time = 0.0
        self.current_trajectory: Trajectory | None = None
        self.current_state: GameState | None = None

        self.last_action = "initialising"
        self.last_target: Vec3 | None = None

        self.save_active = False
        self.kickoff_active = False
        self.recovery_active = False
        self.boost_active = False

        self.previous_role: TeamRole | None = None
        self.previous_challenger_index: int | None = None

        self.shot_scan_elapsed = SHOT_SCAN_INTERVAL
        self.shot_cooldown = 0.0
        self.last_shot_reason = "not scanned"

    def initialize(self) -> None:
        """Initialize match-dependent systems."""
        self._reset_match_systems()

        try:
            count = self.boost_tracker.initialize(self.field_info)
            print(
                f"[GRID][BOOST] Initialized {count} boost pads "
                f"({len(self.boost_tracker.small_pads)} small, "
                f"{len(self.boost_tracker.large_pads)} large)",
                flush=True,
            )
        except Exception as exc:
            self.debug.exception("Boost pad initialization failed", exc)

        self.initialized = True
        self.debug.event("agent", "Grid RLBot v5 initialized", force=True)
        print(
            f"[GRID] Initialized | index={self.index} | team={self.team}",
            flush=True,
        )

    @staticmethod
    def _value(obj: Any, *names: str, default: Any = None) -> Any:
        if obj is None:
            return default
        for name in names:
            try:
                value = getattr(obj, name)
            except (AttributeError, TypeError):
                continue
            if value is not None:
                return value
        return default

    @staticmethod
    def _vector(value: Any) -> Vec3:
        if isinstance(value, Vec3):
            return value
        try:
            return Vec3.from_value(value)
        except (TypeError, ValueError, AttributeError):
            return Vec3()

    @staticmethod
    def _enum_name(value: Any) -> str:
        if value is None:
            return ""
        return str(getattr(value, "name", value)).lower()

    def _on_ground(self, player: Any) -> bool:
        air_state = self._value(player, "air_state")
        state_name = self._enum_name(air_state)
        if "ground" in state_name:
            return True
        if "air" in state_name:
            return False
        wheel_contact = self._value(
            player, "has_wheel_contact", "wheel_contact", default=None
        )
        return bool(wheel_contact) if wheel_contact is not None else False

    def _make_vehicle(self, player: Any) -> VehicleState:
        physics = self._value(player, "physics")
        if physics is None:
            raise ValueError("Player packet has no physics data")

        location = self._vector(self._value(physics, "location"))
        velocity = self._vector(self._value(physics, "velocity"))
        angular_velocity = self._vector(
            self._value(physics, "angular_velocity")
        )
        rotation = self._value(physics, "rotation")

        return VehicleState(
            location=location,
            velocity=velocity,
            pitch=float(self._value(rotation, "pitch", default=0.0)),
            yaw=float(self._value(rotation, "yaw", default=0.0)),
            roll=float(self._value(rotation, "roll", default=0.0)),
            boost=float(
                self._value(player, "boost", "boost_amount", default=0.0)
            ),
            on_ground=self._on_ground(player),
            demolished=bool(
                self._value(
                    player, "is_demolished", "demolished", default=False
                )
            ),
            angular_velocity=angular_velocity,
            team=int(self.team),
            index=int(self.index),
        )

    def _make_ball(self, packet: GamePacket) -> BallState:
        if not packet.balls:
            raise ValueError("Packet contains no balls")
        return BallState.from_packet_ball(packet.balls[0])

    def _make_trajectory(
        self, ball: BallState, packet: GamePacket
    ) -> Trajectory:
        prediction = self._value(self, "ball_prediction")
        if prediction is not None:
            try:
                trajectory = Trajectory.from_ball_or_prediction(
                    ball=ball,
                    prediction=prediction,
                    current_game_seconds=self.current_game_time,
                )
                if trajectory is not None and len(trajectory) > 0:
                    return trajectory
            except (
                AttributeError, TypeError, ValueError, IndexError
            ) as exc:
                self.debug.warning(f"Ball prediction unavailable: {exc}")
        return Trajectory.ballistic(ball, horizon=3.0)

    def _build_game_state(self, packet: GamePacket) -> GameState:
        return GameState.from_packet(
            packet,
            player_index=self.index,
            ball_prediction=self._value(self, "ball_prediction"),
            frame=self.frame,
        )

    def _dt(self, game_time: float) -> float:
        if self.last_game_time is None:
            dt = 1.0 / 120.0
        else:
            dt = game_time - self.last_game_time
        self.last_game_time = game_time
        return max(1.0 / 240.0, min(1.0 / 30.0, dt))

    def _controller(
        self,
        throttle: float = 0.0,
        steer: float = 0.0,
        pitch: float = 0.0,
        yaw: float = 0.0,
        roll: float = 0.0,
        jump: bool = False,
        boost: bool = False,
        handbrake: bool = False,
    ) -> ControllerState:
        controls = ControllerState()
        controls.throttle = max(-1.0, min(1.0, float(throttle)))
        controls.steer = max(-1.0, min(1.0, float(steer)))
        controls.pitch = max(-1.0, min(1.0, float(pitch)))
        controls.yaw = max(-1.0, min(1.0, float(yaw)))
        controls.roll = max(-1.0, min(1.0, float(roll)))
        controls.jump = bool(jump)
        controls.boost = bool(boost)
        controls.handbrake = bool(handbrake)
        return controls

    def _controls_from_output(self, output: Any) -> ControllerState:
        """Map any Grid mechanic output to the RLBot v5 controller."""
        if output is None:
            return self._controller()
        return self._controller(
            throttle=getattr(output, "throttle", 0.0),
            steer=getattr(output, "steer", 0.0),
            pitch=getattr(output, "pitch", 0.0),
            yaw=getattr(output, "yaw", 0.0),
            roll=getattr(output, "roll", 0.0),
            jump=getattr(output, "jump", False),
            boost=getattr(output, "boost", False),
            handbrake=getattr(output, "handbrake", False),
        )

    def _get_team_decision(self, state: GameState):
        try:
            decision = get_team_decision(
                state,
                previous_role=self.previous_role,
                previous_challenger_index=self.previous_challenger_index,
            )
            self.previous_role = decision.role
            self.previous_challenger_index = decision.challenger_index
            return decision
        except Exception as exc:
            self.debug.exception("Teamplay decision failed", exc)
            return None

    @staticmethod
    def _decision_target(decision: Any) -> Vec3 | None:
        if decision is None:
            return None
        for name in ("target", "target_location", "support_target", "position"):
            value = getattr(decision, name, None)
            if value is not None:
                try:
                    return Vec3.from_value(value)
                except (TypeError, ValueError, AttributeError):
                    pass
        return None

    @staticmethod
    def _decision_role(decision: Any) -> str:
        if decision is None:
            return ""
        return GridAgent._enum_name(
            getattr(decision, "role", getattr(decision, "my_role", ""))
        )

    def _save_controls(
        self, car: VehicleState, trajectory: Trajectory, dt: float
    ) -> ControllerState | None:
        if not self.save_active:
            if not self.save.start(car, trajectory, team=int(self.team)):
                return None
            self.save_active = True
            # A genuine goal threat is allowed to interrupt a committed shot.
            self.actions.cancel("emergency save pre-emption", force=True)
            self.boost.reset()
            self.boost_active = False
            self.debug.event("save", "Emergency save started", force=True)

        output: SaveOutput = self.save.update(
            car, trajectory=trajectory, dt=dt
        )
        if output.finished:
            self.save_active = False
            self.debug.event(
                "save", f"Save action ended: {output.reason}", force=True
            )
        elif not output.active:
            self.save_active = False
            return None

        self.last_action = "save"
        self.last_target = output.target
        return self._controls_from_output(output)

    def _kickoff_controls(
        self, state: GameState, dt: float
    ) -> ControllerState | None:
        if state.is_kickoff and not self.kickoff_active:
            self.actions.cancel("kickoff reset", force=True)
            self.boost.reset()
            self.boost_active = False
            self.kickoff.start(state)
            self.kickoff_active = True
            self.debug.event("kickoff", "Kickoff routine started", force=True)

        if not self.kickoff_active:
            return None

        output = self.kickoff.update(state, dt)
        self.last_action = "kickoff"
        self.last_target = self.kickoff.last_target

        if self.kickoff.is_finished():
            self.kickoff_active = False
            self.kickoff.reset()
            return None

        return self._controls_from_output(output)

    def _recovery_controls(
        self, car: VehicleState, dt: float
    ) -> ControllerState | None:
        if not self.recovery_active and car.on_ground:
            return None

        if not self.recovery_active:
            self.recovery = Recovery()
            self.recovery_active = True
            self.recovery.start()

        output = self.recovery.update(car, dt)
        if output.finished:
            self.recovery_active = False
            self.recovery = Recovery()
            return None

        self.last_action = "recovery"
        return self._controls_from_output(output)

    def _boost_controls(
        self,
        car: VehicleState,
        strategic_target: Vec3,
        decision: Any,
        ball: BallState,
        dt: float,
    ) -> ControllerState | None:
        role = self._decision_role(decision)
        covering = "cover" in role

        try:
            ball_own_half = in_own_half(ball.location, int(self.team))
        except (TypeError, ValueError, AttributeError):
            ball_own_half = False

        ball_distance = car.location.flat_distance(ball.location)
        is_challenger = bool(getattr(decision, "should_challenge", False))

        # A challenger close enough to make a play must not keep following
        # an old pad route. This also breaks stale boost commitments.
        if is_challenger and ball_distance < 1800.0 and self.boost.active:
            self.boost.reset()
            self.boost_active = False

        if self.boost.active:
            output = self.boost.update(
                car,
                dt=dt,
                strategic_target=strategic_target,
                covering=covering,
                ball_in_own_half=ball_own_half,
                emergency=False,
            )
            if output.active:
                self.boost_active = True
                self.last_action = "boost"
                self.last_target = output.target
                return self._controls_from_output(output)
            self.boost_active = False

        if car.boost >= self.boost.config.start_boost_below:
            return None
        if ball_distance < 1500.0 and not covering:
            return None

        if not self.boost.start(
            car,
            strategic_target=strategic_target,
            covering=covering,
            ball_in_own_half=ball_own_half,
        ):
            return None

        output = self.boost.update(
            car,
            dt=dt,
            strategic_target=strategic_target,
            covering=covering,
            ball_in_own_half=ball_own_half,
        )
        if not output.active:
            return None

        self.boost_active = True
        self.last_action = "boost"
        self.last_target = output.target
        self.debug.event(
            "boost",
            f"Collecting pad {output.pad_index} "
            f"({'large' if output.pad_is_large else 'small'})",
        )
        return self._controls_from_output(output)

    def _start_selected_shot(
        self, car: VehicleState, trajectory: Trajectory, team: int
    ) -> bool:
        """Select a mechanic once per scan interval and register it persistently."""
        try:
            plan = find_shot(car, trajectory, team)
        except Exception as exc:
            self.last_shot_reason = f"shot selection error: {exc}"
            self.debug.exception("Shot selection failed", exc)
            return False

        candidate = plan.best
        if not plan.valid or candidate is None:
            self.last_shot_reason = plan.reason
            return False

        target = candidate.contact_position
        direction = candidate.shot_direction
        intercept_at = self.current_game_time + candidate.contact_time

        if candidate.shot_type == ShotType.GROUND:
            action = self.ground_shot
            if not action.start(car, trajectory, team):
                self.last_shot_reason = action.reason
                return False
            updater = lambda c, step: action.update(
                c, self.current_trajectory, step
            )
            name = "ground_shot"

        elif candidate.shot_type == ShotType.JUMP:
            action = self.jump_shot
            if not action.start(car, target, direction, intercept_at):
                self.last_shot_reason = action.reason
                return False
            updater = lambda c, step: action.update(
                c, step, current_time=self.current_game_time
            )
            name = "jump_shot"

        elif candidate.shot_type == ShotType.DOUBLE_JUMP:
            action = self.double_jump_shot
            if not action.start(car, target, direction, intercept_at):
                self.last_shot_reason = action.reason
                return False
            updater = lambda c, step: action.update(
                c, step, current_time=self.current_game_time
            )
            name = "double_jump_shot"

        elif candidate.shot_type == ShotType.AERIAL:
            action = self.aerial
            if not action.start(car, target):
                self.last_shot_reason = action.reason
                return False
            updater = lambda c, step: action.update(c, step)
            name = "aerial"

        else:
            self.last_shot_reason = "selector returned no mechanic"
            return False

        request = make_request(
            action=action,
            update_fn=updater,
            name=name,
            priority=ActionPriority.SHOT,
            interruptible=getattr(action, "interruptible", True),
            key=name,
        )
        if not self.actions.start(request):
            self.last_shot_reason = "active action is committed"
            return False

        self.last_shot_reason = candidate.reason
        self.last_action = name
        self.last_target = target
        self.debug.event(
            "shot",
            f"Started {name}: t={candidate.contact_time:.2f}s "
            f"margin={candidate.arrival_margin:.2f}s",
        )
        return True

    def _mechanic_controls(
        self, car: VehicleState, dt: float, decision: Any
    ) -> ControllerState | None:
        if self.actions.active:
            # A non-committed shot should yield if teamplay has reassigned
            # the challenge. A committed jump/aerial is allowed to finish.
            current = self.actions.action
            if (
                not bool(getattr(current, "interruptible", True))
                or bool(getattr(decision, "should_challenge", False))
            ):
                result = self.actions.update(car, dt)
                output = result.action_output
                if output is not None and bool(getattr(output, "active", True)):
                    self.last_action = result.active_action or self.last_action
                    self.last_target = getattr(output, "target", self.last_target)
                    return self._controls_from_output(output)
            else:
                self.actions.cancel("teamplay yielded challenge")

        return None

    def get_output(self, packet: GamePacket) -> ControllerState:
        """Called by RLBot for every game packet."""
        self.frame += 1

        try:
            if not self.initialized:
                self.initialize()

            if not packet.players or len(packet.players) <= self.index:
                return self._controller()
            if not packet.balls:
                return self._controller()

            try:
                self.boost_tracker.update_from_packet(packet)
            except Exception as exc:
                self.debug.exception("Boost pad update failed", exc)

            player = packet.players[self.index]
            car = self._make_vehicle(player)
            ball = self._make_ball(packet)

            match_info = self._value(packet, "match_info")
            self.current_game_time = float(
                self._value(match_info, "seconds_elapsed", default=0.0)
            )
            dt = self._dt(self.current_game_time)

            if car.demolished:
                self.actions.cancel("demolished", force=True)
                self.boost.reset()
                self.boost_active = False
                self.save_active = False
                self.kickoff_active = False
                self.recovery_active = False
                self.last_action = "demolished"
                return self._controller()

            trajectory = self._make_trajectory(ball, packet)
            self.current_trajectory = trajectory

            try:
                self.current_state = self._build_game_state(packet)
            except Exception as exc:
                self.current_state = None
                self.debug.exception("GameState conversion failed", exc)

            # Priority 1: an actual goal threat interrupts every ordinary action.
            save_controls = self._save_controls(car, trajectory, dt)
            if save_controls is not None:
                return save_controls

            # Priority 2: kickoff has its own role assignment and 50/50 routine.
            if self.current_state is not None:
                kickoff_controls = self._kickoff_controls(
                    self.current_state, dt
                )
                if kickoff_controls is not None:
                    return kickoff_controls

            decision = (
                self._get_team_decision(self.current_state)
                if self.current_state is not None
                else None
            )

            team_target = self._decision_target(decision)
            predicted_ball = trajectory.position_at(0.35)
            strategic_target = (
                team_target
                if team_target is not None
                else (predicted_ball if predicted_ball is not None else ball.location)
            )

            # Priority 3: persistent shot/aerial mechanics. Selection is
            # rate-limited; the active action itself is updated every packet.
            self.shot_cooldown = max(0.0, self.shot_cooldown - dt)
            self.shot_scan_elapsed += dt

            mechanic_controls = self._mechanic_controls(car, dt, decision)
            if mechanic_controls is not None:
                return mechanic_controls

            is_challenger = bool(
                getattr(decision, "should_challenge", False)
            )
            if (
                is_challenger
                and not self.actions.active
                and self.shot_cooldown <= 0.0
                and self.shot_scan_elapsed >= SHOT_SCAN_INTERVAL
                and car.on_ground
            ):
                self.shot_scan_elapsed = 0.0
                if self._start_selected_shot(
                    car, trajectory, int(self.team)
                ):
                    mechanic_controls = self._mechanic_controls(
                        car, dt, decision
                    )
                    if mechanic_controls is not None:
                        return mechanic_controls
                else:
                    self.shot_cooldown = 0.08

            # Priority 4: recover airborne cars when no committed shot owns
            # the controls. Recovery never competes with an active aerial.
            if self.recovery_active or (
                not car.on_ground and not self.actions.active
            ):
                recovery_controls = self._recovery_controls(car, dt)
                if recovery_controls is not None:
                    return recovery_controls
            elif car.on_ground and self.recovery_active:
                recovery_controls = self._recovery_controls(car, dt)
                if recovery_controls is not None:
                    return recovery_controls

            # Priority 5: boost routing is lower priority than a challenge,
            # save, kickoff, shot or recovery.
            if not self.actions.active:
                boost_controls = self._boost_controls(
                    car, strategic_target, decision, ball, dt
                )
                if boost_controls is not None:
                    return boost_controls

            # Priority 6: ordinary teamplay/ball drive.
            drive_output = self.drive.update(
                car, strategic_target, dt
            )
            self.last_action = (
                "teamplay_drive" if team_target is not None else "ball_drive"
            )
            self.last_target = strategic_target

            if self.frame % 120 == 0:
                role = self._decision_role(decision) or "unknown"
                print(
                    f"[GRID] action={self.last_action} role={role} "
                    f"speed={car.speed:.0f} boost={car.boost:.0f} "
                    f"shot={self.last_shot_reason}",
                    flush=True,
                )

            return self._controls_from_output(drive_output)

        except Exception as exc:
            self.debug.exception("Grid agent tick failed", exc)
            print(
                f"[GRID][ERROR] frame={self.frame}: {exc}",
                flush=True,
            )
            return self._controller()


if __name__ == "__main__":
    GridAgent("grid-ai/grid/v1").run()
