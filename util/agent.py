
"""Grid RLBot v5 agent.

Coordinates:
- Packet conversion and internal game state
- Teamplay decisions
- Emergency saves
- Persistent boost collection
- Ordinary driving

Mechanics are kept in their own modules. This agent selects and runs them.
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
from mechanics.boost import (
    BoostConfig,
    BoostManager,
    BoostPadTracker,
)

from util.debug import Debug, DebugConfig
from util.game_state import GameState
from util.teamplay import get_team_decision


class GridAgent(Bot):
    """Grid's RLBot v5 entry point."""

    def __init__(self, default_agent_id: str | None = None):
        super().__init__(default_agent_id)

        self.debug = Debug(DebugConfig())

        self.drive = Drive()
        self.save = Save()

        self.boost_tracker = BoostPadTracker()
        self.boost = BoostManager(
            self.boost_tracker,
            BoostConfig(
                start_boost_below=35.0,
                stop_boost=85.0,
                maximum_eta=4.5,
            ),
        )

        self.frame = 0
        self.last_game_time: float | None = None

        self.last_action = "initialising"
        self.last_target: Vec3 | None = None

        self.current_state: GameState | None = None
        self.current_trajectory: Trajectory | None = None

        self.save_active = False
        self.boost_active = False
        self.initialized = False

    def initialize(self) -> None:
        """Initialize match-dependent systems."""

        self.drive = Drive()
        self.save = Save()

        self.boost_tracker = BoostPadTracker()
        self.boost = BoostManager(
            self.boost_tracker,
            BoostConfig(
                start_boost_below=35.0,
                stop_boost=85.0,
                maximum_eta=4.5,
            ),
        )

        self.frame = 0
        self.last_game_time = None

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

        self.debug.event(
            "agent",
            "Grid RLBot v5 initialized",
            force=True,
        )

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

        name = getattr(value, "name", None)
        if name:
            return str(name).lower()

        return str(value).lower()

    def _on_ground(self, player: Any) -> bool:
        air_state = self._value(player, "air_state")
        state_name = self._enum_name(air_state)

        if "ground" in state_name:
            return True

        if "air" in state_name:
            return False

        wheel_contact = self._value(
            player,
            "has_wheel_contact",
            "wheel_contact",
            default=None,
        )

        return bool(wheel_contact) if wheel_contact is not None else False

    def _make_vehicle(self, player: Any) -> VehicleState:
        physics = self._value(player, "physics")

        if physics is None:
            raise ValueError("Player packet has no physics data")

        location = self._vector(
            self._value(physics, "location")
        )
        velocity = self._vector(
            self._value(physics, "velocity")
        )
        angular_velocity = self._vector(
            self._value(physics, "angular_velocity")
        )

        rotation = self._value(physics, "rotation")

        pitch = float(self._value(rotation, "pitch", default=0.0))
        yaw = float(self._value(rotation, "yaw", default=0.0))
        roll = float(self._value(rotation, "roll", default=0.0))

        boost = float(
            self._value(
                player,
                "boost",
                "boost_amount",
                default=0.0,
            )
        )

        demolished = bool(
            self._value(
                player,
                "is_demolished",
                "demolished",
                default=False,
            )
        )

        return VehicleState(
            location=location,
            velocity=velocity,
            pitch=pitch,
            yaw=yaw,
            roll=roll,
            boost=boost,
            on_ground=self._on_ground(player),
            demolished=demolished,
            angular_velocity=angular_velocity,
            team=int(self.team),
            index=int(self.index),
        )

    def _make_ball(self, packet: GamePacket) -> BallState:
        if not packet.balls:
            raise ValueError("Packet contains no balls")

        return BallState.from_packet_ball(packet.balls[0])

    def _make_trajectory(
        self,
        ball: BallState,
        packet: GamePacket,
    ) -> Trajectory:
        prediction = self._value(self, "ball_prediction")
        match_info = self._value(packet, "match_info")

        game_time = float(
            self._value(
                match_info,
                "seconds_elapsed",
                default=0.0,
            )
        )

        if prediction is not None:
            try:
                trajectory = Trajectory.from_ball_or_prediction(
                    ball=ball,
                    prediction=prediction,
                    current_game_seconds=game_time,
                )

                if trajectory is not None and len(trajectory) > 0:
                    return trajectory

            except (
                AttributeError,
                TypeError,
                ValueError,
                IndexError,
            ) as exc:
                self.debug.warning(
                    f"Ball prediction unavailable: {exc}"
                )

        return Trajectory.ballistic(ball, horizon=3.0)

    def _build_game_state(
        self,
        packet: GamePacket,
    ) -> GameState:
        return GameState.from_packet(
            packet,
            player_index=self.index,
            ball_prediction=self._value(self, "ball_prediction"),
            frame=self.frame,
        )

    def _get_team_decision(
        self,
        game_state: GameState,
    ) -> Any | None:
        """Call teamplay using the parameters supported by its signature."""

        try:
            parameters = signature(get_team_decision).parameters

            available = {
                "game_state": game_state,
                "state": game_state,
                "index": self.index,
                "player_index": self.index,
                "trajectory": self.current_trajectory,
                "frame": self.frame,
            }

            kwargs = {
                name: available[name]
                for name, parameter in parameters.items()
                if name in available
                and parameter.kind
                in (
                    parameter.POSITIONAL_OR_KEYWORD,
                    parameter.KEYWORD_ONLY,
                )
            }

            required_missing = [
                name
                for name, parameter in parameters.items()
                if parameter.default is parameter.empty
                and parameter.kind
                in (
                    parameter.POSITIONAL_ONLY,
                    parameter.POSITIONAL_OR_KEYWORD,
                    parameter.KEYWORD_ONLY,
                )
                and name not in kwargs
            ]

            if required_missing:
                self.debug.warning(
                    "Teamplay not called; unsupported parameters: "
                    + ", ".join(required_missing)
                )
                return None

            return get_team_decision(**kwargs)

        except Exception as exc:
            self.debug.exception("Teamplay decision failed", exc)
            return None

    @staticmethod
    def _decision_target(decision: Any) -> Vec3 | None:
        if decision is None:
            return None

        for name in (
            "target",
            "target_location",
            "support_target",
            "position",
        ):
            value = getattr(decision, name, None)

            if value is None:
                continue

            try:
                return Vec3.from_value(value)
            except (TypeError, ValueError, AttributeError):
                continue

        return None

    @staticmethod
    def _decision_role(decision: Any) -> str:
        if decision is None:
            return ""

        for name in ("role", "my_role", "assigned_role"):
            value = getattr(decision, name, None)

            if value is not None:
                return GridAgent._enum_name(value)

        return ""

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

    def _dt(self, game_time: float) -> float:
        if self.last_game_time is None:
            dt = 1.0 / 120.0
        else:
            dt = game_time - self.last_game_time

        self.last_game_time = game_time

        return max(1.0 / 240.0, min(1.0 / 30.0, dt))

    def _save_controls(
        self,
        car: VehicleState,
        trajectory: Trajectory,
        dt: float,
    ) -> ControllerState | None:
        """Emergency saves pre-empt boost and ordinary driving."""

        if not self.save_active:
            started = self.save.start(
                car,
                trajectory,
                team=int(self.team),
            )

            if not started:
                return None

            self.save_active = True

            self.debug.event(
                "save",
                "Emergency save started",
                force=True,
            )

        output: SaveOutput = self.save.update(
            car,
            trajectory=trajectory,
            dt=dt,
        )

        if output.finished:
            self.save_active = False
            self.debug.event(
                "save",
                f"Save action ended: {output.reason}",
                force=True,
            )

        if not output.active and not output.finished:
            self.save_active = False
            return None

        self.last_action = "save"
        self.last_target = output.target

        return self._controller(
            throttle=output.throttle,
            steer=output.steer,
            pitch=output.pitch,
            yaw=output.yaw,
            roll=output.roll,
            jump=output.jump,
            boost=output.boost,
            handbrake=output.handbrake,
        )

    def _boost_controls(
        self,
        car: VehicleState,
        strategic_target: Vec3,
        decision: Any,
        ball: BallState,
        dt: float,
    ) -> ControllerState | None:
        """Start or continue a boost collection action when appropriate."""

        role = self._decision_role(decision)
        covering = "cover" in role

        try:
            ball_own_half = in_own_half(
                ball.location,
                int(self.team),
            )
        except (TypeError, ValueError, AttributeError):
            ball_own_half = False

        # An active boost run is allowed to continue, but emergencies and
        # cover-role restrictions are checked every tick.
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

                return self._controller(
                    throttle=output.throttle,
                    steer=output.steer,
                    boost=output.boost,
                    handbrake=output.handbrake,
                )

            self.boost_active = False

            self.debug.event(
                "boost",
                f"Boost run ended: {output.reason}",
            )

        # Don't divert a challenger from a nearby ball merely to collect pads.
        ball_distance = car.location.flat_distance(ball.location)

        if car.boost >= self.boost.config.start_boost_below:
            return None

        if not covering and ball_distance < 1500.0:
            return None

        started = self.boost.start(
            car,
            strategic_target=strategic_target,
            covering=covering,
            ball_in_own_half=ball_own_half,
        )

        if not started:
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

        return self._controller(
            throttle=output.throttle,
            steer=output.steer,
            boost=output.boost,
            handbrake=output.handbrake,
        )

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

            # Pad status must be updated before selecting or continuing a pad.
            try:
                self.boost_tracker.update_from_packet(packet)
            except Exception as exc:
                self.debug.exception(
                    "Boost pad update failed",
                    exc,
                )

            player = packet.players[self.index]
            car = self._make_vehicle(player)
            ball = self._make_ball(packet)

            match_info = self._value(packet, "match_info")
            game_time = float(
                self._value(
                    match_info,
                    "seconds_elapsed",
                    default=0.0,
                )
            )

            dt = self._dt(game_time)

            trajectory = self._make_trajectory(ball, packet)
            self.current_trajectory = trajectory

            try:
                self.current_state = self._build_game_state(packet)
            except Exception as exc:
                self.current_state = None
                self.debug.exception(
                    "GameState conversion failed",
                    exc,
                )

            # 1. Emergency save has highest priority.
            save_controls = self._save_controls(
                car,
                trajectory,
                dt,
            )

            if save_controls is not None:
                self.boost_active = False
                return save_controls

            # 2. Teamplay decides the role and strategic position.
            decision = None

            if self.current_state is not None:
                decision = self._get_team_decision(
                    self.current_state
                )

            team_target = self._decision_target(decision)

            # 3. Default target is a short predicted ball position.
            target = trajectory.position_at(0.35)

            if target is None:
                target = ball.location

            if team_target is not None:
                target = team_target

            # 4. Collect boost only when it won't undermine the current role.
            boost_controls = self._boost_controls(
                car,
                target,
                decision,
                ball,
                dt,
            )

            if boost_controls is not None:
                return boost_controls

            # 5. Normal driving.
            drive_output = self.drive.update(
                car,
                target,
                dt,
            )

            self.last_action = (
                "teamplay_drive"
                if team_target is not None
                else "ball_drive"
            )
            self.last_target = target

            if self.frame % 120 == 0:
                self.debug.status(
                    f"Grid active | action={self.last_action} "
                    f"| speed={car.speed:.0f} "
                    f"| boost={car.boost:.0f}",
                    force=True,
                )

            return self._controller(
                throttle=drive_output.throttle,
                steer=drive_output.steer,
                boost=drive_output.boost,
                handbrake=drive_output.handbrake,
            )

        except Exception as exc:
            self.debug.exception(
                "Grid agent tick failed",
                exc,
            )

            print(
                f"[GRID][ERROR] frame={self.frame}: {exc}",
                flush=True,
            )

            return self._controller()


if __name__ == "__main__":
    GridAgent("grid-ai/grid/v1").run()
