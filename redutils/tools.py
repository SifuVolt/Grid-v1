"""Convenience controls and shot selection translated from RedUtils.Tools.cs."""
from __future__ import annotations
from math import atan2, copysign
from .math3d import Vec3, clamp
from .objects import Ball, Car, Field, Game, Target
from .actions import AerialShot, GroundShot, JumpShot, DoubleJumpShot
from .utils import shot_power_modifier

class ToolsMixin:
    def Throttle(self,target_speed,backwards=False):
        speed=self.me.Local(self.me.velocity).x; diff=target_speed*(-1 if backwards else 1)-speed
        self.controller.throttle=clamp((abs(diff)**2)*(1 if diff>=0 else -1)/1000,-1,1)
        self.controller.boost=bool(target_speed>1400 and diff>50 and speed<2250 and self.controller.throttle==1 and not backwards)
        return speed
    @staticmethod
    def SteerPD(angle,rate):return clamp((35*(angle+rate))**3/10,-1,1)
    def AimAt(self,target_location,up=None,backwards=False):
        local=self.me.Local(Vec3.from_value(target_location)-self.me.location)*(-1 if backwards else 1)
        safe=Vec3.Up if up is None or Vec3.from_value(up).length==0 else Vec3.from_value(up).Normalize(); local_up=self.me.Local(safe)
        angles=[atan2(local.z,local.x),atan2(local.y,local.x),atan2(local_up.y,local_up.z)]
        self.controller.steer=self.SteerPD(angles[1],-self.me.local_angular_velocity[2]*.01)*(-1 if backwards else 1)
        self.controller.pitch=self.SteerPD(angles[0],self.me.local_angular_velocity[1]*.2)
        self.controller.yaw=self.SteerPD(angles[1],-self.me.local_angular_velocity[2]*.15)
        self.controller.roll=self.SteerPD(angles[2],self.me.local_angular_velocity[0]*.25)
        return angles
    def DefaultShotCheck(self,slice_,target):
        if slice_ is None:return None
        rem=slice_.time-Game.time
        if rem<=0 or not target.Fits(slice_.location):return None
        ball=slice_.ToBall();fin=((slice_.location-self.me.location)/rem).Cap(0,Car.MAX_SPEED)
        ball.velocity=fin+slice_.velocity.Flatten(fin.Normalize())*.8;shot_target=target.Clamp(ball)
        aerial=AerialShot(self.me,slice_,shot_target)
        if aerial.IsValid(self.me):return aerial
        ground=GroundShot(self.me,slice_,shot_target)
        if ground.IsValid(self.me):return ground
        jump=JumpShot(self.me,slice_,shot_target)
        if jump.IsValid(self.me):return jump
        double=DoubleJumpShot(self.me,slice_,shot_target)
        if double.IsValid(self.me):return double
        return None
    def FindShot(self,shot_check,target):
        slice_=Ball.prediction.Find(lambda s:shot_check(s,target) is not None)
        return shot_check(slice_,target) if slice_ is not None else None
