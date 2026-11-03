"""Party Cannon bot strategy, translated from Bot/Bot.cs."""
from __future__ import annotations
from math import fabs
from redutils.bot import RUBot
from redutils.actions import Kickoff, GetBoost, Drive, Arrive
from redutils.objects import Ball, Car, Target
from redutils.math3d import Vec3
from intercept_calculator import InterceptCalculator

class PartyBot(RUBot):
    def Run(self):
        if self.me is None or not Ball.prediction.slices:return
        self.renderer.Text3D(str(self.action) if self.action is not None else "",self.me.location+Vec3.Up*30,1,"red")
        opp_beats=False
        if self.living_opponents:
            team=[self.me]+self.living_teammates
            opp_beats=InterceptCalculator.CalculateInterceptAdvantage(team,self.living_opponents,Ball.MainBall()).opponents_are_faster
        if self.is_kickoff and self.action is None:
            do_kickoff=True
            for mate in self.teammates:
                md=mate.location.Dist(Ball.location);od=self.me.location.Dist(Ball.location)
                if md<od or (md==od and ((self.team!=0 or self.me.location.x<=mate.location.x) and (self.team!=1 or self.me.location.x>=mate.location.x))):
                    do_kickoff=False
            if do_kickoff:self.action=Kickoff()
            elif abs(self.me.location.x)>2000:self.action=GetBoost(self.me,-1,interruptible=False)
            else:self.action=Drive(self.me,Ball.location,1400)
        else:
            if self.is_kickoff or (self.action is not None and (not isinstance(self.action,(Drive,GetBoost,Arrive)) or not self.action.interruptible)):
                return
            shot=self.FindShot(self.DefaultShotCheck,Target(self.their_goal))
            if self.ShouldBeOffensive():
                if self.me.boost<0:self.action=GetBoost(self.me);return
                self.action=shot or self.action or Drive(self.me,Ball.location,2300,allow_dodges=True,waste_boost=True)
            elif self.me.boost<30:self.action=GetBoost(self.me)
            elif not opp_beats:self.action=shot
            else:
                target=Ball.location+(self.our_goal.location-Ball.location).Normalize()*2500
                self.action=Drive(self.me,target,1500,allow_dodges=False)
    def ShouldBeOffensive(self):
        if not self.living_teammates:return True
        biased=self.me.location.FlatDist(Ball.location)-self.me.boost*5
        return not any(m.location.FlatDist(Ball.location)-m.boost*20<biased for m in self.living_teammates)
