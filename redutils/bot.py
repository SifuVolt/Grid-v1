"""RedUtils bot lifecycle translated to the RLBot v5 Python Bot interface."""
from __future__ import annotations
from rlbot.flat import ControllerState
from rlbot.managers import Bot
from .objects import Game, Field, Cars, Ball, Goal
from .renderer import ExtendedRenderer
from .tools import ToolsMixin
from .actions import IAction


def _name(v): return getattr(v,"name",str(v)).split(".")[-1].lower()

class RUBot(ToolsMixin,Bot):
    def __init__(self,default_agent_id=None):
        try: super().__init__(default_agent_id)
        except TypeError: super().__init__()
        self.controller=ControllerState(); self.action=None; self.is_kickoff=False; self.delta_time=1/120; self.last_time=0.; self.last_touch_time=-1.; self.ready=False
        self._rlbot_renderer=getattr(self,"renderer",None)
        self.renderer=ExtendedRenderer(self._rlbot_renderer)
    @property
    def me(self):
        return Cars.all_cars[self.index] if 0<=self.index<len(Cars.all_cars) else None
    @property
    def teammates(self):return [c for c in Cars.all_cars if c.team==self.team and c.index!=self.index]
    @property
    def living_teammates(self):return [c for c in self.teammates if not c.is_demolished]
    @property
    def opponents(self):return [c for c in Cars.all_cars if c.team!=self.team]
    @property
    def living_opponents(self):return [c for c in self.opponents if not c.is_demolished]
    @property
    def our_goal(self):return Field.goals[int(self.team)]
    @property
    def their_goal(self):return Field.goals[1-int(self.team)]
    @property
    def our_score(self):return Game.scores[int(self.team)]
    @property
    def their_score(self):return Game.scores[1-int(self.team)]
    @property
    def Renderer(self):return self.renderer
    @property
    def Index(self):return self.index
    @property
    def Team(self):return self.team
    @property
    def Me(self):return self.me
    @property
    def Action(self):return self.action
    @Action.setter
    def Action(self,v):self.action=v
    @property
    def Controller(self):return self.controller
    @property
    def DeltaTime(self):return self.delta_time
    @property
    def IsKickoff(self):return self.is_kickoff
    @property
    def OurGoal(self):return self.our_goal
    @property
    def TheirGoal(self):return self.their_goal
    @property
    def LivingOpponents(self):return self.living_opponents
    @property
    def LivingTeammates(self):return self.living_teammates
    @property
    def Teammates(self):return self.teammates
    @property
    def Opponents(self):return self.opponents
    def initialize(self):
        Game.initialize();Field.initialize(self.field_info)
        self.ready = True
        raw = getattr(self, "renderer", None)
        if raw is not None and not isinstance(raw, ExtendedRenderer):
            self._rlbot_renderer = raw
            self.renderer = ExtendedRenderer(raw)
    def Run(self):
        """Override with the bot's per-tick strategy."""
        raise NotImplementedError
    def get_output(self,packet):
        self.controller=ControllerState()
        if not self.ready:
            self.initialize();self.ready=True
        now=float(packet.match_info.seconds_elapsed)
        # The C# RedUtils loop updates shared state only once per match tick.
        if now>Game.time or not Cars.all_cars:
            if len(Cars.all_cars)!=len(packet.players):Cars.Initialize(packet)
            else:Cars.Update(packet)
            if packet.balls:Ball.update(self,packet.balls[0],packet)
            Game.update(packet);Field.update(packet)
        phase=_name(packet.match_info.match_phase)
        if not self.is_kickoff and phase=="kickoff":self.action=None
        self.is_kickoff=phase=="kickoff"
        current_renderer = getattr(self, "renderer", None)
        if current_renderer is not None and not isinstance(current_renderer, ExtendedRenderer):
            self._rlbot_renderer = current_renderer
            self.renderer = ExtendedRenderer(current_renderer)
        try:
            self.renderer.begin_rendering()
            self.Run()
            if self.action is not None:
                current=self.action;current.Run(self)
                touch=Ball.latest_touch.time if Ball.latest_touch is not None else -1
                if current.finished or (self.last_touch_time!=touch and current.interruptible) or (self.me and self.me.is_demolished):
                    self.last_touch_time=touch
                    if self.action is current:self.action=None
            self.renderer.end_rendering()
        except Exception as exc:
            self.logger.exception("Party Cannon Python port tick failed: %s",exc)
            try:self.renderer.end_rendering()
            except Exception:pass
            self.delta_time=max(1/240,min(1/30,now-self.last_time if self.last_time else 1/120));self.last_time=now
            return ControllerState()
        self.delta_time=max(1/240,min(1/30,now-self.last_time if self.last_time else 1/120));self.last_time=now
        return self.controller
    def get_ball_prediction(self):return self.ball_prediction
    def GetBallPrediction(self):return self.ball_prediction
