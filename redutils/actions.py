"""Party Cannon's complete action layer, ported from C# to Python.
Copyright (c) 2025 RLBot. Distributed under the MIT license.
"""
from __future__ import annotations
from math import acos, asin, cos, exp, isfinite, pi, sin, sqrt
from copy import copy
from typing import Optional
from .math3d import Vec3, Mat3x3, clamp
from .objects import Ball, BallSlice, Car, Cars, Field, Game, Goal, Surface, Target
from .utils import cap, time_to_jump, predict_rotation, shot_power_modifier


class IAction:
    finished=False
    interruptible=True
    def Run(self,bot): raise NotImplementedError


class Shot(IAction):
    def __init__(self,slice_:BallSlice,shot_target:Vec3):
        self.finished=False; self.interruptible=True; self.slice=slice_; self.shot_target=Vec3.from_value(shot_target)
        self.target_location=Vec3(); self.shot_direction=Vec3()
    def IsValid(self,car): raise NotImplementedError
    def ShotValid(self,threshold=60):
        slices=Ball.prediction.slices
        if len(slices)<2:return False
        lo,hi=0,len(slices)-1
        while hi-lo>2:
            mid=(lo+hi)//2
            if slices[mid].time>self.slice.time:hi=mid
            else:lo=mid
        dt=slices[hi].time-slices[lo].time
        if dt<=1e-6:return slices[lo].location.Dist(self.slice.location)<threshold
        t=clamp((self.slice.time-slices[lo].time)/dt,0,1)
        predicted=slices[lo].location+(slices[hi].location-slices[lo].location)*t
        return self.slice.location.Dist(predicted)<threshold


class Drive(IAction):
    def __init__(self,car:Car,target:Vec3,target_speed:float=Car.MAX_SPEED,allow_dodges:bool=True,waste_boost:bool=False):
        self.finished=False; self.interruptible=True; self.target=Vec3.from_value(target); self.target_speed=float(target_speed)
        fwd=self.GetEta(car,target,False,False); back=self.GetEta(car,target,True,False)
        self.backwards=(back+.5<fwd and car.forward.Dot(car.velocity)<500 and car.forward.FlatAngle(car.location.Direction(target),car.up)>pi*.6)
        self.allow_dodges=allow_dodges; self.waste_boost=waste_boost; self.action=None; self.time_on_ground=0.; self.time_remaining=0.
    def Run(self,bot):
        car=bot.me
        self.time_remaining=self.Distance(car)/max(self.target_speed,1)

        # Match Party Cannon's action ownership: normal Drive controls only
        # while no sub-action (Dodge, SpeedFlip, Wavedash, etc.) is active.
        if self.action is None:
            target_surface=Field.NearestSurface(self.target)
            my_surface=Field.NearestSurface(car.location)
            final_target=Field.LimitToNearestSurface(self.target)
            next_surface=target_surface
            car_speed=car.velocity.length
            forward_speed=car.velocity.Dot(car.forward)

            if car.is_grounded:
                self.time_on_ground+=bot.delta_time

            if my_surface.normal.Dot(target_surface.normal)<.95:
                next_surface=self.FindNextSurface(Field.LimitToNearestSurface(car.location),final_target)
                final_target=next_surface.limit(final_target)
                closest=my_surface.limit(final_target)
                final_target=closest-next_surface.normal.FlatNorm(my_surface.normal)*max(closest.Dist(final_target)-75,0)

            if my_surface.key!=target_surface.key:
                final_target=self.FindTargetAroundCorner(bot,final_target,next_surface)

            turn_radius=self.TurnRadius(abs(forward_speed))
            turn_center=my_surface.limit(car.location)+car.right.FlatNorm(my_surface.normal)*((1 if car.right.Dot(final_target-car.location)>=0 else -1)*turn_radius)
            landing_time=car.PredictLandingTime()

            if Field.DistanceBetweenPoints(turn_center,self.target)>turn_radius-40 and car.is_grounded:
                bot.Throttle(self.target_speed,self.backwards)
            elif not car.is_grounded:
                tr=self.time_remaining if isfinite(self.time_remaining) else .01
                bot.Throttle(self.Distance(car)/max(tr-landing_time,.01))
            else:
                bot.Throttle(max(self.SpeedFromTurnRadius(self.TurnRadiusForCar(car,self.target)),400),self.backwards)

            if car.is_grounded or car.velocity.FlatLen()<500:
                angle=bot.AimAt(final_target,backwards=self.backwards)[0]
            else:
                landing_normal=Field.FindLandingSurface(car).normal
                alpha=cap(landing_time*1.5-.6,0,.75)
                td=car.velocity.FlatNorm(landing_normal)*(1-alpha)+(-Vec3.Up)*alpha
                bot.AimAt(car.location+td,landing_normal)
                angle=car.forward.Angle(td)

            bot.controller.boost=bool(bot.controller.boost and (angle<.3 or (angle<.8 and not car.is_grounded)) and not self.backwards and self.waste_boost)

            predicted=car.LocationAfterDodge()
            time_left=car.location.FlatDist(final_target)/max(car_speed+500,1410)
            speedflip_left=car.location.FlatDist(final_target)/max(car_speed+500+min(car.boost,40)*Car.BOOST_ACCEL/2,1410)

            if self.allow_dodges and Field.InField(predicted,50) and car_speed<2000 and car.location.z<600 and Game.gravity.z<-500 and abs(car.velocity.Dot(car.up))<100:
                if forward_speed>0 and self.target_speed>100+forward_speed:
                    if car.location.z<200 and car.is_grounded and car_speed>1000 and car.forward.FlatAngle(car.location.Direction(final_target))<.1 and self.time_on_ground>.2:
                        dodge=Dodge(car.location.FlatDirection(self.target))
                        if speedflip_left>SpeedFlip.DURATION and car.boost>0 and Field.InField(predicted,500) and self.waste_boost:
                            self.action=SpeedFlip(car.location.FlatDirection(self.target))
                        elif time_left>dodge.Duration:
                            self.action=dodge
                    elif car.location.z>100 and not car.has_double_jumped and (not car.is_grounded or car.velocity.Dot(Vec3.Up)<200):
                        wd=Wavedash(car.location.FlatDirection(self.target))
                        if time_left>wd.Duration:
                            self.action=wd
                elif car.location.z<200 and car.is_grounded and car_speed>800 and self.backwards and (-car.forward).FlatAngle(car.location.Direction(final_target))<.1 and self.time_on_ground>.2 and time_left>HalfFlip.DURATION:
                    self.action=HalfFlip()

        elif self.action.finished:
            self.action=None
            self.backwards=False
            self.time_on_ground=0
        else:
            self.action.Run(bot)
            if isinstance(self.action,SpeedFlip):
                bot.Throttle(self.target_speed+500,self.backwards)

        self.interruptible=self.action is None or self.action.interruptible
        if Field.LimitToNearestSurface(car.location).Dist(Field.LimitToNearestSurface(self.target))<100:
            self.finished=True
    def Distance(self,car):return self.GetDistance(car,self.target,self.backwards)
    def Eta(self,car):return self.GetEta(car,self.target,self.backwards,self.allow_dodges)
    @staticmethod
    def FindNextSurface(start,target):
        middle=Field.LimitToNearestSurface((start+target)/2); start_surface=Field.NearestSurface(start)
        for f in [i*.25 for i in range(8)]:
            p=Field.LimitToNearestSurface(start+(middle-start)*cap(f,0,1)+(target-middle)*cap(f-1,0,1))
            if Field.NearestSurface(p).normal.Dot(start_surface.normal)<.95:return Field.NearestSurface(p)
        return Field.NearestSurface(target)
    @staticmethod
    def FindTargetAroundCorner(bot,final_target,next_surface):
        car=bot.me; s=Field.NearestSurface(car.location); goal=bot.our_goal if Field.side(bot.team)==(1 if final_target.y>=0 else -1) else bot.their_goal
        dist=car.location.Dist(final_target)
        if s.key=="Ground":
            enter_l=car.location.Direction(goal.left_post-Vec3((1 if goal.left_post.x>=0 else -1)*100,(1 if goal.left_post.y>=0 else -1)*50,0)); enter_r=car.location.Direction(goal.right_post-Vec3((1 if goal.right_post.x>=0 else -1)*100,(1 if goal.right_post.y>=0 else -1)*50,0))
            exit_l=car.location.Direction(goal.left_post+Vec3((1 if goal.left_post.x>=0 else -1)*60,-(1 if goal.left_post.y>=0 else -1)*50,0)); exit_r=car.location.Direction(goal.right_post+Vec3((1 if goal.right_post.x>=0 else -1)*60,-(1 if goal.right_post.y>=0 else -1)*50,0))
            d=car.location.Direction(final_target).Clamp(enter_l,enter_r,s.normal) if "Goal Ground" in next_surface.key else car.location.Direction(final_target).Clamp(exit_r,exit_l,s.normal)
            return car.location+d.Rescale(dist)
        if "Goal Ground" in s.key:
            l=car.location.Direction(goal.left_post-Vec3(100,50,0)); r=car.location.Direction(goal.right_post-Vec3(-100,50,0)); return car.location+car.location.Direction(final_target).Clamp(r,l,s.normal).Rescale(dist)
        if "Backboard" in s.key or "Backwall" in s.key:
            if "Left Backwall" in s.key:l=car.location.Direction(goal.top_right_corner+Vec3(50,0,50)); r=car.location.Direction(goal.bottom_right_corner+Vec3(50,0,-50))
            elif "Right Backwall" in s.key:l=car.location.Direction(goal.bottom_left_corner+Vec3(-50,0,-50)); r=car.location.Direction(goal.top_left_corner+Vec3(-50,0,50))
            else:l=car.location.Direction(goal.top_left_corner+Vec3(50,0,50)); r=car.location.Direction(goal.top_right_corner+Vec3(-50,0,50))
            return car.location+car.location.Direction(final_target).Clamp(l,r,s.normal).Rescale(dist)
        return final_target
    @staticmethod
    def GetDistance(car,target,backwards=None,with_turn=False):
        if backwards is None:
            fe=Drive.GetEta(car,target,False,False); be=Drive.GetEta(car,target,True,False)
            backwards=be+.5<fe and car.forward.Dot(car.velocity)<500 and car.forward.FlatAngle(car.location.Direction(target),car.up)>pi*.6
        target=Field.LimitToNearestSurface(target); car_pos=car.PredictLandingPosition(); cs=Field.FindLandingSurface(car); normal=cs.normal
        fwd=car.forward if car.is_grounded else (car.velocity.FlatNorm(normal) if car.velocity.FlatLen()>500 else car.location.FlatDirection(target,normal))
        right=fwd.Cross(-car.up if car.is_grounded else -normal).Normalize(); speed=car.velocity.Dot(fwd)
        angle=(-fwd if backwards else fwd).FlatAngle(target-car_pos,normal); turn_speed=Drive.SpeedAfterTurn(-speed if backwards else speed,angle,.4 if backwards else .5); radius=Drive.TurnRadius(turn_speed)
        center=car_pos+right*(1 if right.Dot(target-car_pos)>=0 else -1)*radius; limited=cs.limit(center)
        if center.Dist(limited)>1:
            n=limited.Direction(Field.LimitToNearestSurface(center+cs.normal*500)); center=limited+n*center.Dist(limited)
        distance=Field.DistanceBetweenPoints(center,target)
        if distance<radius:
            radius=max(Drive.TurnRadiusForCar(car,target),1); center=car_pos+right*(1 if right.Dot(target-car_pos)>=0 else -1)*radius; distance=Field.DistanceBetweenPoints(center,target)
        angle=abs((car_pos-center).FlatAngle(target-center,normal)-(2*pi if (target-car_pos).Dot(-fwd if backwards else fwd)<0 else 0))
        angle=cap(angle-acos(cap(radius/max(distance,1e-6),0,1)),0,2*pi)
        total=sqrt(max(distance*distance-radius*radius,0))+radius*angle
        return (total,angle,radius) if with_turn else total
    @staticmethod
    def GetEta(car,target,backwards=None,allow_dodges=True):
        if backwards is None:
            fe=Drive.GetEta(car,target,False,False); be=Drive.GetEta(car,target,True,False)
            backwards=be+.5<fe and car.forward.Dot(car.velocity)<500 and car.forward.FlatAngle(car.location.Direction(target),car.up)>pi*.6
        distance,angle,radius=Drive.GetDistance(car,target,backwards,True); turn_distance=angle*radius; distance-=turn_distance
        normal=Field.NearestSurface(car.location).normal if car.is_grounded else Field.FindLandingSurface(car).normal
        fwd=car.forward if car.is_grounded else (car.velocity.FlatNorm(normal) if car.velocity.FlatLen()>500 else car.location.FlatDirection(target,normal))
        speed=fwd.Dot(car.velocity); landing=car.PredictLandingTime()
        if backwards:
            post=max(Drive.SpeedAfterTurn(-speed,angle,.8),1400)
            return landing+turn_distance/max(Drive.SpeedFromTurnRadius(radius),400)+distance/post
        min_speed=max(Drive.SpeedAfterTurn(speed,angle),1400); fin=cap(min_speed+Car.BOOST_ACCEL*car.boost/Car.BOOST_CONSUMPTION,1400,Car.MAX_SPEED)
        boost_dist=(fin*fin-min_speed*min_speed)/(2*Car.BOOST_ACCEL)
        if distance<boost_dist:
            fin=cap(sqrt(max(min_speed*min_speed+2*Car.BOOST_ACCEL*distance,0)),1400,Car.MAX_SPEED)
            return landing+turn_distance/max(Drive.SpeedFromTurnRadius(radius),400)+distance/max((min_speed+fin)/2,1)
        if allow_dodges and fin>0 and distance/fin>1.25:
            return landing+turn_distance/max(Drive.SpeedFromTurnRadius(radius),400)+boost_dist/max((min_speed+fin)/2,1)+(distance-boost_dist)/(fin+500)
        return landing+turn_distance/max(Drive.SpeedFromTurnRadius(radius),400)+boost_dist/max((min_speed+fin)/2,1)+(distance-boost_dist)/max(fin,1)
    @staticmethod
    def TurnRadius(speed):
        speed=cap(speed,.01,Car.MAX_SPEED)
        ranges=[(500,145,251),(1000,251,425),(1500,425,727),(1750,727,909),(2300,909,1136)]
        low=0
        for high,a,b in ranges:
            if speed<=high:return a+(b-a)*(speed-low)/(high-low)
            low=high
        return 1136
    @staticmethod
    def SpeedFromTurnRadius(radius):
        radius=cap(radius,145,1136); ranges=[(251,145,251,0,500),(425,251,425,500,1000),(727,425,727,1000,1500),(909,727,909,1500,1750),(1136,909,1136,1750,2300)]
        for high,lo,_,a,b in ranges:
            if radius<=high:return a+(b-a)*(radius-lo)/(high-lo)
        return 2300
    @staticmethod
    def SpeedAfterTurn(speed,angle,modifier=1):
        x=angle*.49*modifier*(.2 if speed>1234 else 1); e=exp(min(x,20))
        return cap((1234*e*x+speed)/(e*x+1),0,Car.MAX_SPEED)
    @staticmethod
    def TurnRadiusForCar(car,target):
        d=Field.DistanceBetweenPoints(car.location,target); denom=(car.right*(1 if car.right.Dot(target-car.location)>=0 else -1)).Dot(car.location.FlatDirection(target,car.up))
        return (d/2)/denom if abs(denom)>1e-6 else 1136


class Arrive(IAction):
    def __init__(self,car,target,direction=None,arrival_time=-1,allow_flipping=True,recovery_time=0):
        self.finished=False; self.interruptible=True; self.target=Vec3.from_value(target); self.direction=Vec3.from_value(direction); self.arrival_time=arrival_time; self.allow_flipping=allow_flipping; self.drive=Drive(car,self.target,Car.MAX_SPEED,allow_flipping,arrival_time>=0); self.recovery_time=recovery_time; self.time_remaining=0
    def Run(self,bot):
        car=bot.me; distance=self.Distance(car); speed=car.velocity.length; self.time_remaining=distance/Car.MAX_SPEED if self.arrival_time<0 else max(self.arrival_time-Game.time,.001); target_speed=distance/max(self.time_remaining,.001)
        predicted=car.LocationAfterDodge()
        if self.direction.length>0:
            normal=Field.NearestSurface(self.target).normal; to_target=car.location.FlatDirection(self.target,normal); add=self.recovery_time*speed
            shift=min(Field.DistanceBetweenPoints(self.target,car.location)*.6,cap(speed,1410,Car.MAX_SPEED)*1.5); tr=Drive.TurnRadius(cap(speed,500,Car.MAX_SPEED))*1.2
            shift*=cap((shift-add)/tr,0,1) if target_speed<2200 or self.arrival_time<0 else 0
            left=to_target.Cross(normal).Normalize(); right=to_target.Cross(-normal).Normalize()
            shifted=Field.LimitToNearestSurface(self.target-self.direction.Clamp(left,right,normal).Normalize()*shift)
        else:shifted=self.target
        time_left=car.location.FlatDist(self.target)/max(speed+500,1410)
        before=predicted.FlatDirection(self.target).Dot(self.direction.Cross())
        now=car.location.FlatDirection(self.target).Dot(self.direction.Cross())
        self.drive.allow_dodges=(before>=0)==(now>=0) and time_left>1.35+self.recovery_time
        self.drive.target=shifted; self.drive.target_speed=target_speed; self.drive.Run(bot); self.interruptible=self.drive.interruptible
        if Field.LimitToNearestSurface(car.location).Dist(Field.LimitToNearestSurface(self.target))<100 or (Game.time>self.arrival_time>0):self.finished=True
    def Distance(self,car):return Drive.GetDistance(car,self.target)
    def Eta(self,car):return Drive.GetEta(car,self.target)


class Dodge(IAction):
    def __init__(self,direction,jump_time=.1):
        self.finished=False; self.interruptible=False; self.direction=Vec3.from_value(direction); self.jump_time=jump_time; self.start_time=None; self.jumping=True; self.input=Vec3(); self.step=0
    @property
    def Duration(self):return self.jump_time+1.15
    def Run(self,bot):
        if self.start_time is None:self.start_time=Game.time; self.jumping=bot.me.is_grounded
        elapsed=Game.time-self.start_time
        if bot.me.is_grounded and elapsed>(self.jump_time if self.jumping else 0)+.1:self.finished=True
        elif elapsed<self.jump_time and self.jumping:bot.controller.jump=True
        elif self.step<3 and self.jumping:bot.controller.jump=False; self.step+=1
        elif elapsed<(self.jump_time if self.jumping else 0)+.6:
            if self.input.length==0:
                local=Vec3(-bot.me.forward.FlatNorm().Cross().Dot(self.direction),-bot.me.forward.FlatNorm().Dot(self.direction),0)
                fv=bot.me.forward.Dot(bot.me.velocity); s=abs(fv)/Car.MAX_SPEED
                backwards=(local.x<0) if abs(fv)<100 else ((local.x>=0)!=(fv>0))
                local.x/=((16/15)*(1+1.5*s)) if backwards else 1; local.y/=(1+.9*s); self.input=local.Normalize()
            bot.controller.yaw=self.input.x; bot.controller.pitch=self.input.y; bot.controller.jump=True
        else:self.finished=True


class HalfFlip(IAction):
    DURATION=1.25
    def __init__(self):self.finished=False;self.interruptible=False;self.jumping=True;self.start_time=None;self.step=0
    def Run(self,bot):
        if self.start_time is None:self.start_time=Game.time;self.jumping=bot.me.is_grounded
        elapsed=Game.time-self.start_time
        if elapsed<.1 and self.jumping:bot.controller.jump=True
        elif self.step<3 and self.jumping:bot.controller.jump=False;self.step+=1
        elif elapsed<(.1 if self.jumping else 0)+.2:bot.controller.yaw=0;bot.controller.pitch=1;bot.controller.jump=True
        elif elapsed<(.1 if self.jumping else 0)+1:bot.AimAt(bot.me.location+bot.me.velocity.Flatten())
        else:self.finished=True


class SpeedFlip(IAction):
    DURATION=1.35
    def __init__(self,direction):self.finished=False;self.interruptible=False;self.direction=Vec3.from_value(direction).Normalize();self.start_time=None;self.side=0
    def Run(self,bot):
        bot.controller.throttle=1
        if self.start_time is None:
            angle=.06*bot.me.velocity.Dot(bot.me.forward)/max(Drive.TurnRadius(bot.me.velocity.Dot(bot.me.forward)),1)
            left=self.direction.Rotate(angle).Flatten().Normalize();right=self.direction.Rotate(-angle).Flatten().Normalize()
            if bot.me.velocity.Angle(left)<bot.me.velocity.Angle(right):
                bot.AimAt(bot.me.location+left)
                if bot.me.velocity.FlatAngle(left)<.05:self.start_time=Game.time;self.side=1
            else:
                bot.AimAt(bot.me.location+right)
                if bot.me.velocity.FlatAngle(right)<.05:self.start_time=Game.time;self.side=-1
        else:
            e=Game.time-self.start_time
            if 0<e<.1:bot.controller.jump=True
            elif .12<e<.15:bot.controller.jump=True;bot.controller.pitch=-1;bot.controller.roll=self.side*.5
            elif .15<e<.75:bot.controller.pitch=1;bot.controller.roll=self.side
            elif .75<e<.9:bot.controller.pitch=1;bot.controller.handbrake=True;bot.controller.roll=self.side;bot.controller.yaw=self.side
            elif e>.9:self.finished=True
        if not Field.InField(bot.me.location,150) or (self.start_time is None and not bot.me.is_grounded):self.finished=True


class Wavedash(IAction):
    def __init__(self,direction=None,jump_time=.05):self.finished=False;self.interruptible=False;self.direction=Vec3.from_value(direction);self.jump_time=jump_time;self.jumping=True;self.start_time=None;self.input=Vec3()
    @property
    def Duration(self):return self.jump_time*4+.8
    def Run(self,bot):
        if self.start_time is None:self.start_time=Game.time;self.jumping=bot.me.is_grounded
        e=Game.time-self.start_time
        if e<self.jump_time and self.jumping:bot.controller.jump=True
        elif not bot.me.is_grounded and bot.me.location.z<40 and bot.me.velocity.z<-100:
            if self.input.length==0:
                local=bot.me.Local(self.direction if self.direction.length>0 else bot.me.velocity.Normalize());self.input=Vec3(local.y,-local.x,0)
            bot.controller.yaw=self.input.x;bot.controller.pitch=self.input.y;bot.controller.jump=True
        elif not bot.me.is_grounded:
            n=Field.FindLandingSurface(bot.me).normal; d=self.direction.FlatNorm(n) if self.direction.length>0 else bot.me.velocity.FlatNorm(n);bot.AimAt(bot.me.location+d+n*.2,n)
        else:self.finished=True


class GetBoost(IAction):
    def __init__(self,car,boost_index=-1,interruptible=True,boosts=None):
        self.finished=False;self.interruptible=interruptible;self.initial_interruptible=interruptible;self.boost_index=boost_index;self.eta=0
        candidates=list(boosts) if boosts is not None else Field.boosts
        if boost_index==-1:
            valid=[(Drive.GetEta(car,b.location),b) for b in candidates if (boosts is not None or b.is_large)]
            valid=[x for x in valid if x[1].is_active or x[1].time_until_active<x[0]]
            if valid:self.boost_index=min(valid,key=lambda x:x[0])[1].index
        if not 0<=self.boost_index<len(Field.boosts):
            self.chosen_boost=None;self.drive_action=None;self.finished=True;return
        self.chosen_boost=Field.boosts[self.boost_index];self.drive_action=Drive(car,self.chosen_boost.location,Car.MAX_SPEED,True,self.chosen_boost.is_large)
    def Run(self,bot):
        if self.drive_action is None:self.finished=True;return
        self.drive_action.Run(bot);self.chosen_boost=Field.boosts[self.boost_index];self.eta=Drive.GetEta(bot.me,self.chosen_boost.location)
        self.interruptible=self.initial_interruptible and self.drive_action.interruptible
        self.finished=(not self.chosen_boost.is_active and (self.chosen_boost.time_until_active>Drive.GetEta(bot.me,self.chosen_boost.location) or self.drive_action.finished)) or bot.me.boost>90


class GroundShot(Shot):
    def __init__(self,car,slice_,shot_target):
        super().__init__(slice_,shot_target);self.start_boost=car.boost;self.update_interval=.2;self.update_timer=0;self.left_ground=False;self.SetTargetLocation(car,True)
    def SetTargetLocation(self,car,init=False):
        rem=max(self.slice.time-Game.time,.001);surface=Field.NearestSurface(self.slice.location);normal=surface.normal;ball=self.slice.ToBall()
        fin=(((self.slice.location if init else self.target_location)-car.location).Flatten(normal)/rem).Cap(0,Car.MAX_SPEED);ball.velocity=(fin*6+self.slice.velocity)/7
        direction=self.slice.location.FlatDirection(self.shot_target);vd=(fin-self.slice.velocity).length
        t=self.slice.location.FlatDist(self.shot_target)/cap(vd*shot_power_modifier(vd)+ball.velocity.Dot(direction),500,Ball.MAX_SPEED)
        ball.velocity=(fin+self.slice.velocity*2)/3;self.shot_direction=ball.PredictLocation(t).Direction(self.shot_target);self.target_location=self.slice.location-self.shot_direction*120
        height=(self.target_location-surface.limit(self.target_location)).Dot(normal);ball_h=(self.slice.location-surface.limit(self.slice.location)).Dot(normal)
        if height<20:
            a=asin(cap((ball_h-20)/120,-1,1));self.shot_direction=self.shot_direction.FlatNorm(normal)*cos(a)+normal*sin(a);self.target_location=self.slice.location-self.shot_direction*120
        if init:self.arrive=Arrive(car,self.target_location,self.shot_direction.FlatNorm(normal),self.slice.time,True,.1)
        else:self.arrive.target=self.target_location;self.arrive.direction=self.shot_direction.FlatNorm(normal);self.finished=height>20.5
    def Run(self,bot):
        rem=self.slice.time-Game.time;self.arrive.Run(bot);self.interruptible=self.arrive.interruptible;self.left_ground=self.left_ground or not bot.me.is_grounded;eta=self.arrive.Eta(bot.me);self.update_timer+=bot.delta_time
        if self.interruptible and (rem<0 or (self.left_ground and bot.me.is_grounded) or not self.ShotValid() or bot.me.boost>self.start_boost or eta>max(rem*1.05,rem+.025) or (eta<rem-.25 and self.update_timer>self.update_interval)):self.finished=True
        elif self.update_timer>self.update_interval:self.SetTargetLocation(bot.me);self.update_timer=0
    def IsValid(self,car):
        rem=self.slice.time-Game.time;s=Field.NearestSurface(self.target_location);h=(self.target_location-s.limit(self.target_location)).Dot(s.normal)
        return Drive.GetEta(car,self.target_location)<rem and h<20.5 and self.shot_direction.z<.9


class JumpShot(Shot):
    def __init__(self,car,slice_,shot_target):
        super().__init__(slice_,shot_target);self.start_boost=car.boost;self.update_interval=.2;self.update_timer=0;self.jumped=False;self.left_ground=False;self.jump_elapsed=0;self.latest_touch=-1;self.step=0;self.arrive=None;self.dodge_direction=Vec3();self.SetTargetLocation(car,True)
    def SetTargetLocation(self,car,init=False):
        rem=max(self.slice.time-Game.time,.001);ball=self.slice.ToBall();add=car.location.FlatDirection(self.slice.location)*500 if init else self.shot_direction.FlatNorm()*500
        fin=(((self.slice.location if init else self.target_location)-car.location)/rem+add).Cap(0,Car.MAX_SPEED);ball.velocity=(fin*6+self.slice.velocity)/7
        direction=self.slice.location.FlatDirection(self.shot_target);vd=(fin-self.slice.velocity).length;t=self.slice.location.FlatDist(self.shot_target)/cap(vd*shot_power_modifier(vd)+ball.velocity.Dot(direction),500,Ball.MAX_SPEED)
        ball.velocity=(fin+self.slice.velocity*2)/3;self.shot_direction=ball.PredictLocation(t).Direction(self.shot_target);self.target_location=self.slice.location-self.shot_direction*165
        surface=Field.NearestSurface(self.slice.location);n=surface.normal;height=(self.target_location-surface.limit(self.target_location)).Dot(n);a=asin(cap(self.shot_direction.Dot(n),-1,1))
        da=min(a+.25,pi*.4,max(predict_rotation(Car.PITCH_ANGULAR_ACCEL,time_to_jump(n,max(height,50)-17))+.2,.6));a=da-.25
        self.shot_direction=self.shot_direction.FlatNorm(n)*cos(a)+n*sin(a);self.dodge_direction=self.shot_direction.FlatNorm(n)*cos(da)+n*sin(da);self.target_location=self.slice.location-self.shot_direction*165
        ball_h=(self.slice.location-surface.limit(self.slice.location)).Dot(n);height=(self.target_location-surface.limit(self.target_location)).Dot(n)
        if height<40:
            a=asin(cap((ball_h-40)/165,-1,1));self.shot_direction=(self.shot_direction.FlatNorm(n)*cos(a)+n*sin(a)).Normalize();self.target_location=self.slice.location-self.shot_direction*165
        jt=time_to_jump(n,max(height,40)-17);self.target_location-=self.shot_direction.FlatNorm(n)*25
        if n.Dot(Vec3.Up)<.95:self.target_location=self.target_location.Flatten()+Vec3.Up*(self.slice.location.z+cap(self.target_location.z-self.slice.location.z,-120,120))-Game.gravity*(.5*jt*jt)
        self.arrive=Arrive(car,self.target_location,self.shot_direction.FlatNorm(n),self.slice.time,True,jt+.1) if init else self.arrive
        if not init:self.arrive.target=self.target_location;self.arrive.direction=self.shot_direction.FlatNorm(n);self.arrive.recovery_time=jt+.1
    def ReadyToJump(self,car,jt):
        rem=self.slice.time-Game.time;fin=car.LocationAfterJump(rem,0)-Game.gravity*(.5*rem*rem);n=Field.NearestSurface(car.location).normal
        return rem<jt and abs(car.velocity.Dot(car.up))<200 and fin.FlatDist(self.target_location,n)<50-n.Dot(Vec3.Up)*20 and self.interruptible
    def Run(self,bot):
        rem=self.slice.time-Game.time
        if self.latest_touch<0 and Ball.latest_touch:self.latest_touch=Ball.latest_touch.time
        surface=Field.NearestSurface(bot.me.location)
        if not self.jumped:
            self.arrive.Run(bot);self.interruptible=self.arrive.interruptible;self.left_ground=self.left_ground or not bot.me.is_grounded
            height=cap((self.target_location-surface.limit(self.target_location)).Dot(surface.normal)-17,1,270);jt=max(time_to_jump(surface.normal,height),.15);eta=self.arrive.Eta(bot.me);self.update_timer+=bot.delta_time
            if self.interruptible and (rem<jt-.05 or (self.left_ground and bot.me.is_grounded) or not self.ShotValid() or bot.me.boost>self.start_boost or eta>max(rem*1.05,rem+.025) or (eta<rem-.25 and self.update_timer>self.update_interval)):self.finished=True
            elif self.update_timer>self.update_interval and rem>jt:self.SetTargetLocation(bot.me);self.update_timer=0
            elif self.ReadyToJump(bot.me,jt):self.jumped=True
        else:
            self.interruptible=False;self.jump_elapsed+=bot.delta_time;bot.AimAt(bot.me.location+self.dodge_direction,self.dodge_direction.Cross(self.dodge_direction.Cross(-surface.normal)).Normalize())
            if rem<-.1 or (rem>.4 and not self.ShotValid()) or (Ball.latest_touch and self.latest_touch!=Ball.latest_touch.time and Ball.latest_touch.player_index!=bot.index and Ball.location.Dist(bot.me.location)>200):self.finished=True
            elif rem>.075 or self.jump_elapsed<.05:bot.controller.jump=True
            elif self.step<3 or rem>.05:bot.controller.jump=False;self.step+=1
            else:bot.action=Dodge(self.shot_direction.FlatNorm(),.1)
    def IsValid(self,car):
        rem=self.slice.time-Game.time;s=Field.NearestSurface(self.target_location);h=max((self.target_location-s.limit(self.target_location)).Dot(s.normal),20)
        return Drive.GetEta(car,self.target_location)<rem and h<270 and rem>time_to_jump(s.normal,h)


class DoubleJumpShot(Shot):
    def __init__(self,car,slice_,shot_target):
        super().__init__(slice_,shot_target);self.start_boost=car.boost;self.update_interval=.2;self.update_timer=0;self.jumped=False;self.left_ground=False;self.jump_elapsed=0;self.latest_touch=-1;self.step=0;self.SetTargetLocation(car,True)
    def SetTargetLocation(self,car,init=False):
        rem=max(self.slice.time-Game.time,.001);ball=self.slice.ToBall();fin=(((self.slice.location if init else self.target_location)-car.location)/rem).Cap(0,Car.MAX_SPEED);ball.velocity=(fin*6+self.slice.velocity)/7
        direction=self.slice.location.FlatDirection(self.shot_target);vd=(fin-self.slice.velocity).length;t=self.slice.location.FlatDist(self.shot_target)/cap(vd*shot_power_modifier(vd)+ball.velocity.Dot(direction),500,Ball.MAX_SPEED)
        ball.velocity=(fin+self.slice.velocity*2)/3;self.shot_direction=ball.PredictLocation(t).Direction(self.shot_target);a=min(asin(self.shot_direction.z),pi*.35);self.shot_direction=self.shot_direction.FlatNorm()*cos(a)+Vec3.Up*sin(a);self.target_location=self.slice.location-self.shot_direction*160
        surface=Field.NearestSurface(self.slice.location);n=surface.normal;d=surface.limit(self.target_location).Dist(self.target_location)
        if d<50:
            a=asin(cap((d-50)/160,-1,1));self.shot_direction=self.shot_direction.FlatNorm(n)*cos(a)+n*sin(a);self.target_location=self.slice.location-self.shot_direction*160
        if init:self.arrive=Arrive(car,self.target_location.Flatten(),self.shot_direction.FlatNorm(),self.slice.time,True,time_to_jump(Vec3.Up,self.target_location.z-17,True))
        else:self.arrive.target=self.target_location.Flatten();self.arrive.direction=self.shot_direction.FlatNorm();self.arrive.recovery_time=time_to_jump(Vec3.Up,self.target_location.z-17,True)
    def Run(self,bot):
        rem=self.slice.time-Game.time
        if self.latest_touch<0 and Ball.latest_touch:self.latest_touch=Ball.latest_touch.time
        if not self.jumped:
            self.arrive.Run(bot);self.interruptible=self.arrive.interruptible;self.left_ground=self.left_ground or not bot.me.is_grounded;jt=max(time_to_jump(Vec3.Up,self.target_location.z-17,True),.2);eta=self.arrive.Eta(bot.me);self.update_timer+=bot.delta_time
            if self.interruptible and (rem<jt-.05 or (self.left_ground and bot.me.is_grounded) or not self.ShotValid() or bot.me.boost>self.start_boost or eta>max(rem*1.05,rem+.025) or (eta<rem-.25 and bot.me.boost>10 and self.update_timer>self.update_interval)):self.finished=True
            elif self.update_timer>self.update_interval and rem>jt:self.SetTargetLocation(bot.me);self.update_timer=0
            else:
                fin=bot.me.LocationAfterDoubleJump(rem,0)
                if rem<jt and fin.FlatDist(self.target_location)<40 and self.interruptible:self.jumped=True
        else:
            self.interruptible=False;self.jump_elapsed+=bot.delta_time
            if rem<0 or (Ball.latest_touch and self.latest_touch!=Ball.latest_touch.time and Ball.latest_touch.player_index!=bot.index and Ball.location.Dist(bot.me.location)>200):self.finished=True
            elif self.jump_elapsed<Car.JUMP_MAX_DURATION:bot.controller.jump=True
            elif self.step<3:bot.controller.jump=False;self.step+=1
            elif self.step<6:bot.controller.jump=True;self.step+=1
            else:
                offset=self.target_location-bot.me.PredictLocation(rem);bot.controller.boost=offset.Dot(bot.me.forward)/max(rem,.001)>=(Car.BOOST_ACCEL+Car.AIR_THROTTLE_ACCEL)*max(bot.delta_time,13/120) and offset.Normalize().Dot(bot.me.forward)>.75
                bot.controller.throttle=1 if offset.Normalize().Dot(bot.me.forward)>.5 else 0;bot.AimAt(bot.me.location+self.shot_direction)
    def IsValid(self,car):
        rem=self.slice.time-Game.time;return Drive.GetEta(car,self.target_location.Flatten())<rem and 300<=self.target_location.z<510 and rem>time_to_jump(Vec3.Up,self.target_location.z,True)


class AerialShot(Shot):
    def __init__(self,car,slice_,shot_target):
        super().__init__(slice_,shot_target);self.start_boost=car.boost;self.double_jumping=False;self.drive_action=None;self.drive_location=Vec3();self.jump_immediately=False;self.currently_double=False;self.aerialing=False;self.jumped=False;self.elapsed=0;self.step=0
        self.SetTargetLocation(car);self.jump_immediately=car.velocity.FlatAngle(car.location.Direction(self.target_location),car.up)<.6 or car.velocity.FlatLen(car.up)<500 or not car.is_grounded
        flat=self.target_location.Flatten();self.drive_location=flat if Drive.GetEta(car,flat,False)<=Drive.GetEta(car,self.target_location,False) else self.target_location
        self.drive_action=Drive(car,self.drive_location,Drive.GetDistance(car,self.drive_location)/max(slice_.time-Game.time,.001))
    def SetTargetLocation(self,car):
        rem=max(self.slice.time-Game.time,.001);after=self.slice.ToBall();fin=((self.slice.location-car.location)/rem).Cap(0,Car.MAX_SPEED);after.velocity=(fin*6+self.slice.velocity)/7
        to_goal=self.slice.location.FlatDirection(self.shot_target);vd=(fin-self.slice.velocity).length;t=self.slice.location.FlatDist(self.shot_target)/cap(vd*shot_power_modifier(vd)+after.velocity.Dot(to_goal),500,Ball.MAX_SPEED)
        after.velocity=(fin+self.slice.velocity*2)/3;self.shot_direction=after.PredictLocation(t).Direction(self.shot_target);self.target_location=self.slice.location-self.shot_direction*155
        n=Field.NearestSurface(self.target_location).normal;d=(self.target_location-Field.LimitToNearestSurface(self.target_location)).Dot(n)
        if d<50:
            a=asin(cap((d-50)/155,-1,1));self.shot_direction=(self.shot_direction.FlatNorm(n)*cos(a)+n*sin(a)).Normalize();self.target_location=self.slice.location-self.shot_direction*155
    def CanHit(self,car):return self.GetBoostEstimate(car,False)>=0 or self.GetBoostEstimate(car,True)>=0
    def GetBoostEstimate(self,car,double_jump):
        rem=self.slice.time-Game.time;fp=car.LocationAfterDoubleJump(rem,0) if car.is_grounded and double_jump else car.LocationAfterJump(rem,0) if car.is_grounded else car.PredictLocation(rem)
        fv=car.VelocityAfterDoubleJump(rem,0) if car.is_grounded and double_jump else car.VelocityAfterJump(rem,0) if car.is_grounded else car.PredictVelocity(rem)
        delta=self.target_location-fp;direction=delta.Normalize();angle=max(direction.Angle(car.forward),.0001);turn=.6*(2*sqrt(angle/9));tau1=turn*cap(1-.4/angle,0,1);remaining=max(rem-tau1,.001);req=2*delta.length/(remaining**2);ratio=req/(Car.BOOST_ACCEL+Car.AIR_THROTTLE_ACCEL);tau2=rem-remaining*sqrt(1-cap(ratio,0,1));ve=fv+(Car.BOOST_ACCEL+Car.AIR_THROTTLE_ACCEL)*(tau2-tau1)*direction;boost=(tau2-tau1)*Car.BOOST_CONSUMPTION
        return boost if ve.length<Car.MAX_SPEED*.9 and boost<car.boost*.9 and abs(ratio)<.9 else -1
    def IsValid(self,car):
        jump_immediately=self.CanHit(car) and self.jump_immediately; should_turn=car.is_grounded
        if should_turn:
            _,angle,radius=Drive.GetDistance(car,self.drive_location,self.drive_action.backwards,with_turn=True);normal=Field.NearestSurface(car.location).normal
            drive_direction=car.forward.FlatNorm(normal)*(-1 if self.drive_action.backwards else 1);side_dot=car.right.Dot(self.drive_location-car.location)
            turn_side=car.right.FlatNorm(normal)*(1 if side_dot>0 else -1 if side_dot<0 else 0); car_after_turn=copy(car)
            car_after_turn.location=car.location+turn_side*radius*(1-cos(angle))+drive_direction*sin(angle)
            new_forward=car_after_turn.location.FlatDirection(self.drive_location,normal)
            car_after_turn.orientation=Mat3x3(car.orientation.Forward,car.orientation.Right,car.orientation.Up);car_after_turn.orientation[0]=new_forward;car_after_turn.orientation[2]=normal
            car_after_turn.velocity=new_forward*Drive.SpeedAfterTurn(car.velocity.length,angle,.8 if self.drive_action.backwards else 1)
            should_turn=self.CanHit(car_after_turn) and not self.jump_immediately
        return self.target_location.z>300 and (jump_immediately or should_turn)
    def Run(self,bot):
        rem=self.slice.time-Game.time
        if not self.aerialing:
            self.drive_action.target_speed=Drive.GetDistance(bot.me,self.drive_location)/max(rem,.001);self.drive_action.Run(bot)
            if self.start_boost<bot.me.boost or bot.me.boost<=0 or not self.ShotValid():self.finished=True
            elif self.jump_immediately or abs(bot.controller.steer)<.2:
                jump=self.GetBoostEstimate(bot.me,False);double=self.GetBoostEstimate(bot.me,True)
                if jump>=0 or double>=0:self.double_jumping=double>=0 and (double<jump or jump<0);self.aerialing=True
                else:self.finished=True
        else:
            self.interruptible=False;self.elapsed+=bot.delta_time;self.currently_double=False
            if not self.jumped:
                if self.elapsed<=Car.JUMP_MAX_DURATION:bot.controller.jump=True
                elif self.step<3 and self.double_jumping:bot.controller.jump=False;self.step+=1
                elif self.step<6 and self.double_jumping:bot.controller.jump=True;self.currently_double=True;self.step+=1
                else:self.jumped=True
            fin=bot.me.PredictLocation(rem) if self.jumped else (bot.me.LocationAfterDoubleJump(rem,self.elapsed) if self.double_jumping else bot.me.LocationAfterJump(rem,self.elapsed))
            offset=self.target_location-fin;required=2*offset.length/max(rem*rem,.0001);bot.AimAt(bot.me.location+offset,bot.me.location.Direction(self.slice.location) if self.jumped else Vec3.Up)
            bot.controller.boost=offset.Dot(bot.me.forward)/max(rem,.001)>=(Car.BOOST_ACCEL+Car.AIR_THROTTLE_ACCEL)*max(bot.delta_time,13/120) and offset.Angle(bot.me.forward)<.4
            bot.controller.throttle=cap(offset.Dot(bot.me.forward)/max(rem,.001)/(Car.AIR_THROTTLE_ACCEL*max(bot.delta_time,1/120)),-1,1)
            if self.currently_double:bot.controller.steer=bot.controller.yaw=bot.controller.pitch=bot.controller.roll=0
            if rem<=0 or (self.jumped and offset.length>50 and rem>.5 and required*.8>Car.AIR_THROTTLE_ACCEL and (bot.me.boost==0 or required*.8>Car.BOOST_ACCEL+Car.AIR_THROTTLE_ACCEL)) or (not self.ShotValid() and rem>.5) or (bot.me.is_grounded and self.jumped):self.finished=True
            elif ((not self.double_jumping and self.elapsed<1.45) or (not bot.me.has_jumped and self.jumped)) and rem<.1 and offset.length<100:bot.action=Dodge(self.shot_direction.FlatNorm(),.1)
            elif offset.length<50 and not self.currently_double:bot.AimAt(bot.me.location+self.shot_direction,bot.me.location.Direction(self.slice.location))


class QuickShot(IAction):
    def __init__(self,car,target):
        self.finished=False;self.interruptible=True;self.target=Vec3.from_value(target);self.arrive=Arrive(car,target,car.location.Direction(target));self.latest_touch=-1
    def Run(self,bot):
        if self.latest_touch<0 and Ball.latest_touch:self.latest_touch=Ball.latest_touch.time
        shot_dir=Ball.location.Direction(self.target);normal=Field.NearestSurface(Ball.location).normal;self.arrive.target=Ball.location;self.arrive.direction=shot_dir;self.arrive.Run(bot);self.interruptible=self.arrive.interruptible
        if self.interruptible and Ball.latest_touch and self.latest_touch!=Ball.latest_touch.time and Ball.latest_touch.player_index==bot.index:self.finished=True
        elif (self.arrive.time_remaining<.2 or bot.me.location.Dist(Ball.location)<300) and self.interruptible:bot.action=Dodge(bot.me.location.FlatDirection(Ball.location,normal))


class Kickoff(IAction):
    def __init__(self):self.finished=False;self.interruptible=False;self.is_diagonal=False;self.time_on_ground=0;self.speed_flipped=False;self.speed_flip=None;self.noob_drive=NoobBlackDrive()
    def Run(self,bot):
        if bot.me.velocity.length<200:self.is_diagonal=abs(bot.me.location.x)>1000
        if self.speed_flipped and bot.me.is_grounded:self.time_on_ground+=bot.delta_time
        if self.speed_flip is not None and not self.speed_flip.finished:
            bot.controller.boost=True;self.speed_flip.Run(bot)
        else:
            bot.Throttle(Car.MAX_SPEED)
            if not self.is_diagonal or self.speed_flipped:
                kickoff_target=Ball.location-Ball.location.Direction(bot.their_goal.location)*(170 if self.speed_flipped else 2600)
                bot.controller.steer=self.noob_drive.steer_toward_target(bot.me,kickoff_target)
            elif bot.me.velocity.length>500:
                bot.controller.steer=self.noob_drive.steer_toward_target(bot.me,Ball.location)
            if not bot.is_kickoff:self.finished=True
            elif bot.me.velocity.length>(600 if self.is_diagonal else 700+abs(bot.me.location.x)*3) and not self.speed_flipped:
                self.speed_flipped=True;self.speed_flip=SpeedFlip(bot.me.location.FlatDirection(Ball.location-Ball.location.Direction(bot.their_goal.location)*(250 if self.is_diagonal else -1000)))
            elif bot.me.location.Dist(Ball.location)<800 and self.time_on_ground>.1:bot.action=Dodge(Ball.location.Direction(bot.their_goal.location),.18)
