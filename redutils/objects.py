"""Arena, player, ball and targeting objects from Party Cannon's RedUtils layer.

Ported to the RLBot v5 Python packet model. Copyright (c) 2025 RLBot; MIT.
"""
from __future__ import annotations
from dataclasses import dataclass
from math import sqrt, asin, cos, sin, pi
from typing import Any, Callable
from .math3d import Vec3, Mat3x3
from .utils import cap


def _get(obj: Any, *names: str, default=None):
    for name in names:
        if obj is not None and hasattr(obj,name): return getattr(obj,name)
    return default

def _enum_name(value):
    return getattr(value,"name",str(value)).split(".")[-1].lower()


class Game:
    scores=[0,0]; time=0.0; time_remaining=300.0; game_speed=1.0
    is_unlimited_time=False; is_overtime=False; match_phase=None
    gravity=Vec3(0,0,-650); initialized=False
    @classmethod
    def initialize(cls): cls.initialized=True
    @classmethod
    def update(cls, packet):
        teams=_get(packet,"teams",default=[])
        cls.scores=[int(_get(t,"score",default=0)) for t in teams[:2]] or [0,0]
        info=_get(packet,"match_info")
        cls.time=float(_get(info,"seconds_elapsed",default=0.0))
        cls.time_remaining=float(_get(info,"game_time_remaining",default=300.0))
        cls.game_speed=float(_get(info,"game_speed",default=1.0))
        cls.is_unlimited_time=bool(_get(info,"is_unlimited_time",default=False))
        cls.is_overtime=bool(_get(info,"is_overtime",default=False))
        cls.match_phase=_get(info,"match_phase")
        cls.gravity=Vec3(0,0,float(_get(info,"world_gravity_z",default=-650)))


class Surface:
    def __init__(self,key,location,normal,size,xdirection,ydirection):
        self.key=key; self.location=Vec3.from_value(location); self.normal=Vec3.from_value(normal).Normalize()
        self.size=Vec3.from_value(size); self.xdirection=Vec3.from_value(xdirection).Normalize(); self.ydirection=Vec3.from_value(ydirection).Normalize()
    @property
    def Key(self): return self.key
    @property
    def Normal(self): return self.normal
    @property
    def Location(self): return self.location
    def limit(self,pos):
        d=Vec3.from_value(pos)-self.location
        return self.location+self.xdirection*cap(self.xdirection.Dot(d),-self.size.x/2,self.size.x/2)+self.ydirection*cap(self.ydirection.Dot(d),-self.size.y/2,self.size.y/2)
    Limit=limit


class Goal:
    WIDTH=1780.0; HEIGHT=640.0; DEPTH=850.0
    def __init__(self,team:int):
        self.team=team; s=2*int(team)-1; h=10240.0/2*s
        self.location=Vec3(0,h,0)
        self.left_post=Vec3(self.WIDTH/2*s,h,self.HEIGHT/2); self.right_post=Vec3(-self.WIDTH/2*s,h,self.HEIGHT/2)
        self.top_left_corner=Vec3(self.WIDTH/2*s,h,self.HEIGHT); self.top_right_corner=Vec3(-self.WIDTH/2*s,h,self.HEIGHT)
        self.bottom_left_corner=Vec3(self.WIDTH/2*s,h,0); self.bottom_right_corner=Vec3(-self.WIDTH/2*s,h,0)
        self.crossbar=Vec3(0,h,self.HEIGHT)
        self.Location=self.location; self.LeftPost=self.left_post; self.RightPost=self.right_post
        self.TopLeftCorner=self.top_left_corner; self.TopRightCorner=self.top_right_corner
        self.BottomLeftCorner=self.bottom_left_corner; self.BottomRightCorner=self.bottom_right_corner; self.Crossbar=self.crossbar


class Boost:
    def __init__(self,index:int,pad=None):
        self.index=index; self.location=Vec3.from_value(_get(pad,"location")) if pad is not None else Vec3()
        self.is_large=bool(_get(pad,"is_full_boost",default=False)); self.is_active=bool(_get(pad,"is_active",default=True)); self.time_until_active=float(_get(pad,"timer",default=0))
    def update(self,state):
        self.is_active=bool(_get(state,"is_active",default=self.is_active)); self.time_until_active=float(_get(state,"timer",default=0))
    Index=property(lambda s:s.index); Location=property(lambda s:s.location); IsLarge=property(lambda s:s.is_large)
    IsActive=property(lambda s:s.is_active); TimeUntilActive=property(lambda s:s.time_until_active)


class Field:
    LENGTH=10240.0; WIDTH=8192.0; HEIGHT=1950.0
    CORNER_INTERSECTION=8064.0; CORNER_LENGTH=1629.0; CORNER_WIDTH=1152.0
    goals=[Goal(0),Goal(1)]; boosts=[]; surfaces={}; drivable_surfaces={}
    @classmethod
    def side(cls,team): return 2*int(team)-1
    @classmethod
    def _make_surfaces(cls):
        w,l,h=cls.WIDTH,cls.LENGTH,cls.HEIGHT; cw=cls.CORNER_WIDTH; cl=cls.CORNER_LENGTH; gw=Goal.WIDTH; gh=Goal.HEIGHT; gd=Goal.DEPTH; rt=sqrt(2)/2
        cls.surfaces={
          "Ground":Surface("Ground",Vec3(0,0,0),Vec3.Up,Vec3(w,l),Vec3.X,Vec3.Y),
          "Ceiling":Surface("Ceiling",Vec3(0,0,h),-Vec3.Up,Vec3(w,l),Vec3.X,Vec3.Y),
          "Orange Backboard":Surface("Orange Backboard",Vec3(0,l/2,h/2+gh/2),Vec3(0,-1,0),Vec3(gw,h-gh),Vec3.X,Vec3.Z),
          "Orange Right Backwall":Surface("Orange Right Backwall",Vec3(w/4+gw/2-cw+200,l/2,h/2),Vec3(0,-1,0),Vec3(w/2-gw/2-cw,h),Vec3.X,Vec3.Z),
          "Orange Left Backwall":Surface("Orange Left Backwall",Vec3(-w/4-gw/2+cw-200,l/2,h/2),Vec3(0,-1,0),Vec3(w/2-gw/2-cw,h),Vec3.X,Vec3.Z),
          "Blue Backboard":Surface("Blue Backboard",Vec3(0,-l/2,h/2+gh/2),Vec3(0,1,0),Vec3(gw,h-gh),Vec3.X,Vec3.Z),
          "Blue Right Backwall":Surface("Blue Right Backwall",Vec3(-w/4-gw/2+cw-200,-l/2,h/2),Vec3(0,1,0),Vec3(w/2-gw/2-cw,h),Vec3.X,Vec3.Z),
          "Blue Left Backwall":Surface("Blue Left Backwall",Vec3(w/4+gw/2-cw+200,-l/2,h/2),Vec3(0,1,0),Vec3(w/2-gw/2-cw,h),Vec3.X,Vec3.Z),
          "Right Orange Sidewall":Surface("Right Orange Sidewall",Vec3(w/2,0,h/2),Vec3(-1,0,0),Vec3(h,l),Vec3.Z,Vec3.Y),
          "Right Blue Sidewall":Surface("Right Blue Sidewall",Vec3(-w/2,0,h/2),Vec3(1,0,0),Vec3(h,l),Vec3.Z,Vec3.Y),
          "Right Orange Corner":Surface("Right Orange Corner",Vec3(w/2-cw/2,l/2-cw/2,h/2),Vec3(-rt,-rt,0),Vec3(cl,h),Vec3(-rt,rt,0),Vec3.Z),
          "Left Orange Corner":Surface("Left Orange Corner",Vec3(-w/2+cw/2,l/2-cw/2,h/2),Vec3(rt,-rt,0),Vec3(cl,h),Vec3(rt,rt,0),Vec3.Z),
          "Right Blue Corner":Surface("Right Blue Corner",Vec3(-w/2+cw/2,-l/2+cw/2,h/2),Vec3(rt,rt,0),Vec3(cl,h),Vec3(rt,-rt,0),Vec3.Z),
          "Left Blue Corner":Surface("Left Blue Corner",Vec3(w/2-cw/2,-l/2+cw/2,h/2),Vec3(-rt,rt,0),Vec3(cl,h),Vec3(-rt,-rt,0),Vec3.Z),
          "Orange Goal Ground":Surface("Orange Goal Ground",Vec3(0,l/2+gd/2,0),Vec3.Up,Vec3(gw,gd),Vec3.X,Vec3.Y),
          "Blue Goal Ground":Surface("Blue Goal Ground",Vec3(0,-l/2-gd/2,0),Vec3.Up,Vec3(gw,gd),Vec3.X,Vec3.Y),
        }
        cls.drivable_surfaces={k:v for k,v in cls.surfaces.items() if k!="Ceiling"}
    @classmethod
    def initialize(cls,field_info):
        cls._make_surfaces(); cls.boosts=[]
        for i,p in enumerate(_get(field_info,"boost_pads",default=[])):
            cls.boosts.append(Boost(i,p))
    @classmethod
    def update(cls,packet):
        for i,p in enumerate(_get(packet,"boost_pads",default=[])):
            if i<len(cls.boosts): cls.boosts[i].update(p)
    @classmethod
    def InField(cls,pos,radius):
        p=Vec3.Abs(pos)
        if p.x>cls.WIDTH/2-radius or p.y>cls.LENGTH/2+Goal.DEPTH-radius: return False
        if (p.x>Goal.WIDTH/2-radius or p.z>Goal.HEIGHT-radius) and p.y>cls.LENGTH/2-radius: return False
        return p.x+p.y<=cls.CORNER_INTERSECTION-radius
    @classmethod
    def InGoal(cls,pos,radius):
        p=Vec3.Abs(pos); return p.x<Goal.WIDTH/2+radius and cls.LENGTH/2-radius<p.y<cls.LENGTH/2+Goal.DEPTH+radius and p.z<Goal.HEIGHT+radius
    @classmethod
    def NearestSurface(cls,pos,excluded=()):
        if not cls.drivable_surfaces: cls._make_surfaces()
        values=[v for v in cls.surfaces.values() if v not in excluded]
        if not excluded: values=list(cls.drivable_surfaces.values())
        return min(values,key=lambda s:Vec3.from_value(pos).Dist(s.limit(pos)))
    @classmethod
    def LimitToNearestSurface(cls,pos,excluded=()): return cls.NearestSurface(pos,excluded).limit(pos)
    @classmethod
    def FindLandingSurface(cls,car):
        if car.is_grounded: return cls.NearestSurface(car.location,())
        from .utils import quadratic
        gt=quadratic(Game.gravity.z/2,car.velocity.z,car.location.z-15)[1]
        ct=quadratic(Game.gravity.z/2,car.velocity.z,car.location.z+15-cls.HEIGHT)[1]
        surface=cls.surfaces["Ceiling"] if ct>=0 else cls.surfaces["Ground"]; t=ct if ct>=0 else gt
        landing=car.PredictLocation(t)
        if not cls.InField(landing,150):
            for candidate in cls.drivable_surfaces.values():
                if candidate.key=="Ground": continue
                denom=car.velocity.Dot(-candidate.normal)
                st=car.location.Dist(candidate.limit(car.location))/denom if abs(denom)>1e-6 else -1
                if 0<st<t: t=st; surface=candidate
        return surface
    @classmethod
    def DistanceBetweenPoints(cls,a,b):
        s1=cls.NearestSurface(a); p1=s1.limit(a); s2=cls.NearestSurface(b); p2=s2.limit(b); sm=cls.NearestSurface((p1+p2)/2); pm=sm.limit((p1+p2)/2)
        return p1.Dist(s1.limit(pm))+s1.limit(pm).Dist(sm.limit(p2))+sm.limit(p2).Dist(p2)
    Side=side; InField=InField; InGoal=InGoal

Field._make_surfaces()


class Hitbox:
    def __init__(self,location,dimensions,offset,orientation):
        self.location=Vec3.from_value(location); self.dimensions=Vec3.from_value(dimensions); self.offset=Vec3.from_value(offset); self.orientation=orientation
    @property
    def center(self): return self.location+self.orientation.Forward*self.offset.x+self.orientation.Right*self.offset.y+self.orientation.Up*self.offset.z


class BallTouch:
    def __init__(self,player):
        touch=_get(player,"latest_touch")
        self.time=float(_get(touch,"game_seconds",default=-1)); self.location=Vec3.from_value(_get(touch,"location")); self.normal=Vec3.from_value(_get(touch,"normal"))
        self.player_name=str(_get(player,"name",default="")); self.player_index=int(_get(player,"player_id",default=-1)); self.team=int(_get(player,"team",default=0)); self.ball_index=int(_get(touch,"ball_index",default=0))


class BallSlice:
    def __init__(self,slice_=None,location=None,velocity=None,angular_velocity=None,time=0):
        if slice_ is not None:
            p=_get(slice_,"physics"); self.location=Vec3.from_value(_get(p,"location")); self.velocity=Vec3.from_value(_get(p,"velocity")); self.angular_velocity=Vec3.from_value(_get(p,"angular_velocity")); self.time=float(_get(slice_,"game_seconds",default=0))
        else: self.location=Vec3.from_value(location); self.velocity=Vec3.from_value(velocity); self.angular_velocity=Vec3.from_value(angular_velocity); self.time=float(time)
        self.Location=self.location; self.Velocity=self.velocity; self.AngularVelocity=self.angular_velocity; self.Time=self.time
    def ToBall(self): return Ball(self.location,self.velocity,self.angular_velocity)


class BallPrediction:
    def __init__(self,prediction=None):
        self.slices=[]
        source=_get(prediction,"slices",default=prediction if isinstance(prediction,(list,tuple)) else [])
        for item in source or []: self.slices.append(item if isinstance(item,BallSlice) else BallSlice(item))
    @property
    def Length(self): return len(self.slices)
    def __len__(self): return len(self.slices)
    def __getitem__(self,i): return self.slices[i]
    def Find(self,predicate:Callable):
        if not self.slices: return None
        for i in range(6,len(self.slices),6):
            if predicate(self.slices[i]):
                for j in range(max(0,i-6),i):
                    if abs(self.slices[j].location.y)>5250: break
                    if predicate(self.slices[j]): return self.slices[j]
            elif abs(self.slices[i].location.y)>5250: break
        return None
    def FindGoal(self,team):
        side=-Field.side(team)
        for i in range(6,len(self.slices),6):
            if self.slices[i].location.y*side>5250:
                for j in range(i-6,i):
                    if self.slices[j].location.y*side>5250: return self.slices[j]
        return None


class Ball:
    MAX_SPEED=6000.0; RADIUS=93.15
    location=Vec3(); velocity=Vec3(); angular_velocity=Vec3(); latest_touch=None; prediction=BallPrediction()
    def __init__(self,location=None,velocity=None,angular_velocity=None):
        self.location=Vec3.from_value(location); self.velocity=Vec3.from_value(velocity); self.angular_velocity=Vec3.from_value(angular_velocity)
    @classmethod
    def update(cls,bot,ball_info,packet=None):
        p=_get(ball_info,"physics"); cls.location=Vec3.from_value(_get(p,"location")); cls.velocity=Vec3.from_value(_get(p,"velocity")); cls.angular_velocity=Vec3.from_value(_get(p,"angular_velocity"))
        touches=[c.latest_touch for c in Cars.all_cars if c.latest_touch is not None]; cls.latest_touch=max(touches,key=lambda x:x.time) if touches else None
        pred=_get(bot,"ball_prediction",default=None); cls.prediction=BallPrediction(pred)
    @classmethod
    def MainBall(cls): return cls(cls.location,cls.velocity,cls.angular_velocity)
    def Predict(self,t): return Ball(self.PredictLocation(t),self.PredictVelocity(t))
    def PredictLocation(self,t): return self.location+self.velocity*t+Game.gravity*(0.5*t*t)
    def PredictVelocity(self,t): return self.velocity+Game.gravity*t
    @classmethod
    def GetLatestTouch(cls): return cls.latest_touch


class Car:
    JUMP_MAX_DURATION=.2; JUMP_ACCEL=1458.333; JUMP_VEL=291.667; AIR_THROTTLE_ACCEL=66.667; BRAKE_ACCEL=3500; BOOST_ACCEL=991.667; BOOST_CONSUMPTION=33.3; MAX_SPEED=2300; STICKY_ACCEL=325
    PITCH_ANGULAR_ACCEL=12.46; YAW_ANGULAR_ACCEL=9.11; ROLL_ANGULAR_ACCEL=38.34; MAX_ANGULAR_VEL=5.5
    def __init__(self,index=0,player_info=None):
        self.name=""; self.index=index; self.team=0; self.boost=0.; self.is_bot=True; self.is_grounded=False; self.has_jumped=False; self.has_double_jumped=False; self.is_demolished=False; self.is_supersonic=False
        self.location=Vec3(); self.velocity=Vec3(); self.angular_velocity=Vec3(); self.local_angular_velocity=Vec3(); self.rotation=Vec3(); self.orientation=Mat3x3(self.rotation)
        self.hitbox_dimensions=Vec3(); self.hitbox_offset=Vec3(); self.latest_touch=None
        self.score=self.goals=self.own_goals=self.assists=self.saves=self.shots=self.demolitions=0
        if player_info is not None: self.update(player_info,initial=True)
    def update(self,p,initial=False):
        phys=_get(p,"physics"); self.location=Vec3.from_value(_get(phys,"location")); self.velocity=Vec3.from_value(_get(phys,"velocity")); self.angular_velocity=Vec3.from_value(_get(phys,"angular_velocity")); self.rotation=Vec3.from_value(_get(phys,"rotation")); self.orientation=Mat3x3(self.rotation); self.local_angular_velocity=self.Local(self.angular_velocity)
        if initial: self.name=str(_get(p,"name",default="")); self.team=int(_get(p,"team",default=0))
        self.boost=float(_get(p,"boost",default=0)); self.is_bot=bool(_get(p,"is_bot",default=True)); air=_enum_name(_get(p,"air_state",default="on_ground"))
        self.is_grounded=air in ("onground","on_ground","airstate.onground"); self.has_jumped=air in ("jumping",); self.has_double_jumped=air in ("doublejumping","dodging","double_jumping")
        self.is_demolished=float(_get(p,"demolished_timeout",default=0))>0; self.is_supersonic=bool(_get(p,"is_supersonic",default=False))
        si=_get(p,"score_info"); self.score=int(_get(si,"score",default=0)); self.goals=int(_get(si,"goals",default=0)); self.own_goals=int(_get(si,"own_goals",default=0)); self.assists=int(_get(si,"assists",default=0)); self.saves=int(_get(si,"saves",default=0)); self.shots=int(_get(si,"shots",default=0)); self.demolitions=int(_get(si,"demolitions",default=0))
        touch=_get(p,"latest_touch"); self.latest_touch=BallTouch(p) if touch is not None else None
        hb=_get(p,"hitbox"); self.hitbox_dimensions=Vec3(float(_get(hb,"length",default=0)),float(_get(hb,"width",default=0)),float(_get(hb,"height",default=0))); self.hitbox_offset=Vec3.from_value(_get(p,"hitbox_offset"))
    @property
    def forward(self): return self.orientation.Forward
    @forward.setter
    def forward(self,v): self.orientation[0]=v
    @property
    def right(self): return self.orientation.Right
    @property
    def up(self): return self.orientation.Up
    @property
    def hitbox(self): return Hitbox(self.location,self.hitbox_dimensions,self.hitbox_offset,self.orientation)
    def Local(self,v): return self.orientation.Dot(v)
    def PredictLandingTime(self):
        if self.is_grounded:return 0
        from .utils import quadratic
        ground=quadratic(Game.gravity.z/2,self.velocity.z,self.location.z-15)[1]; ceiling=quadratic(Game.gravity.z/2,self.velocity.z,self.location.z+15-Field.HEIGHT)[1]; t=ceiling if ceiling>=0 else ground
        if not Field.InField(self.PredictLocation(t),150):
            s=Field.FindLandingSurface(self); denom=self.velocity.Dot(-s.normal); t=max((self.location-s.limit(self.location)).Dot(s.normal)/denom,0) if abs(denom)>1e-6 else 0
        return max(t,0)
    def PredictLandingPosition(self): return self.PredictLocation(self.PredictLandingTime())
    def Predict(self,t):
        c=Car(); c.__dict__.update(self.__dict__); c.location=self.PredictLocation(t); c.velocity=self.PredictVelocity(t); return c
    def PredictLocation(self,t): return self.location+self.velocity*t+Game.gravity*(.5*t*t)
    def PredictVelocity(self,t): return self.velocity+Game.gravity*t
    def LocationAfterDodge(self): return self.location+(self.velocity+self.velocity.FlatNorm()*500).Cap(0,Car.MAX_SPEED)*1.3
    def LocationAfterJump(self,time,elapsed):
        jt=cap(Car.JUMP_MAX_DURATION-elapsed,0,Car.JUMP_MAX_DURATION); st=cap(.05-elapsed,0,.05)
        return self.location+self.velocity*time+Game.gravity*(.5*time*time)+(self.up*Car.JUMP_VEL*time if self.is_grounded else Vec3())+self.up*Car.JUMP_ACCEL*jt*(time-.5*jt)-self.up*Car.STICKY_ACCEL*st*(time-.5*st)
    def LocationAfterDoubleJump(self,time,elapsed):
        jt=cap(Car.JUMP_MAX_DURATION-elapsed,0,Car.JUMP_MAX_DURATION); st=cap(.05-elapsed,0,.05)
        return self.location+self.velocity*time+Game.gravity*(.5*time*time)+(self.up*Car.JUMP_VEL*time if self.is_grounded else Vec3())+self.up*Car.JUMP_ACCEL*jt*(time-.5*jt)-self.up*Car.STICKY_ACCEL*st*(time-.5*st)+(self.up*Car.JUMP_VEL*(time-jt) if not self.has_double_jumped else Vec3())
    def VelocityAfterJump(self,time,elapsed):
        jt=cap(Car.JUMP_MAX_DURATION-elapsed,0,Car.JUMP_MAX_DURATION); st=cap(.05-elapsed,0,.05)
        return self.velocity+Game.gravity*time+self.up*(Car.JUMP_VEL if self.is_grounded else 0)+self.up*Car.JUMP_ACCEL*jt-self.up*Car.STICKY_ACCEL*st
    def VelocityAfterDoubleJump(self,time,elapsed):
        jt=cap(Car.JUMP_MAX_DURATION-elapsed,0,Car.JUMP_MAX_DURATION); st=cap(.05-elapsed,0,.05)
        return self.velocity+Game.gravity*time+self.up*Car.JUMP_VEL*(2 if self.is_grounded else 1)+self.up*Car.JUMP_ACCEL*jt-self.up*Car.STICKY_ACCEL*st


class Cars:
    all_cars=[]
    @classmethod
    def Initialize(cls,packet): cls.all_cars=[Car(i,p) for i,p in enumerate(_get(packet,"players",default=[]))]
    @classmethod
    def Update(cls,packet):
        players=_get(packet,"players",default=[])
        if len(players)!=len(cls.all_cars): return cls.Initialize(packet)
        for c,p in zip(cls.all_cars,players): c.update(p)
    Count=classmethod(lambda cls:len(cls.all_cars))


class Target:
    MIN_ERROR=50
    def __init__(self,top_left,bottom_right=None,shoot_away_from_goal=False):
        if bottom_right is None and hasattr(top_left,"top_left_corner"):
            g=top_left
            if shoot_away_from_goal: self.top_left=g.bottom_right_corner; self.bottom_right=g.top_left_corner
            else: self.top_left=g.top_left_corner; self.bottom_right=g.bottom_right_corner
            normal=-Vec3.Y*Field.side(g.team); center=(self.top_left+self.bottom_right)/2+Vec3.Y*Field.side(g.team)*Ball.RADIUS
            self.target_surface=Surface("target",center,normal,Vec3(max(self.top_left.FlatDist(self.bottom_right)-Ball.RADIUS*2,1),max(abs(self.top_left.z-self.bottom_right.z)-Ball.RADIUS*2,1)),(self.top_left-self.bottom_right).FlatNorm(),Vec3.Up)
        else:
            self.top_left=Vec3.from_value(top_left); self.bottom_right=Vec3.from_value(bottom_right)
            self.target_surface=Surface("target",(self.top_left+self.bottom_right)/2,(self.top_left-self.bottom_right).Cross(Vec3.Up).Normalize(),Vec3(max(self.top_left.FlatDist(self.bottom_right)-Ball.RADIUS*2,1),max(abs(self.top_left.z-self.bottom_right.z)-Ball.RADIUS*2,1)),(self.top_left-self.bottom_right).FlatNorm(),Vec3.Up)
    def get_corrected_limits(self,ball):
        tl,br=self.top_left,self.bottom_right; loc=Vec3.from_value(ball); line=(br-tl).FlatNorm()
        left_adj=tl+(tl-loc).Normalize().Rotate(-asin(cap(Ball.RADIUS/max(tl.FlatDist(loc),1e-6),-1,1))).Cross(-Vec3.Up).Normalize()*Ball.RADIUS
        right_adj=br+(br-loc).Normalize().Rotate(asin(cap(Ball.RADIUS/max(br.FlatDist(loc),1e-6),-1,1))).Cross(Vec3.Up).Normalize()*Ball.RADIUS
        top=(tl+br).Flatten()/2+Vec3.Up*tl.z; bottom=(tl+br).Flatten()/2+Vec3.Up*br.z
        top_adj=top+(tl-loc).Normalize().Rotate(-asin(cap(Ball.RADIUS/max(tl.FlatDist(loc,line),1e-6),-1,1)),line).Cross(-line).Normalize()*Ball.RADIUS
        bottom_adj=bottom+(br-loc).Normalize().Rotate(asin(cap(Ball.RADIUS/max(br.FlatDist(loc,line),1e-6),-1,1)),line).Cross(line).Normalize()*Ball.RADIUS
        left=tl if (tl-loc).Flatten().Dot((tl-br).Flatten())>0 else left_adj
        right=br if (br-loc).Flatten().Dot((br-tl).Flatten())>0 else right_adj
        top=top if (tl-loc).Flatten(line).Dot(Vec3.Up)>0 else top_adj
        bottom=bottom if (br-loc).Flatten(line).Dot(-Vec3.Up)>0 else bottom_adj
        return left,right,top,bottom
    def Fits(self,ball_location):
        loc=Vec3.from_value(ball_location); left,right,top,bottom=self.get_corrected_limits(loc); goal_line=(self.bottom_right-self.top_left).FlatNorm()
        width=left.FlatDist(right); height=abs(left.z-right.z); hp=(left-right).Cross(Vec3.Up).Normalize(); vp=(top-bottom).Cross(goal_line).Normalize()
        h=(left+right)/2-loc; v=(top+bottom)/2-loc
        return ((h.Normalize().Dot(-hp)*width>self.MIN_ERROR or h.length<width/2) and (v.Normalize().Dot(-vp)*height>self.MIN_ERROR or v.length<height/2)) or self.top_left.FlatDirection(self.bottom_right).Cross(Vec3.Up).Dot(loc.FlatDirection((self.top_left+self.bottom_right)/2))<0
    def Clamp(self,ball):
        left,right,top,bottom=self.get_corrected_limits(ball.location); line=(self.bottom_right-self.top_left).FlatNorm(); n=line.Cross(-Vec3.Up)
        if n.Dot(ball.location-(self.top_left+self.bottom_right)/2)>0:
            denom=ball.velocity.Dot(n); t=abs((ball.location-self.top_left).Dot(n)/denom) if abs(denom)>1e-6 else 0
            target=self.target_surface.limit(ball.PredictLocation(t)); dh=(target-ball.location).Normalize().Clamp((left-ball.location).Normalize(),(right-ball.location).Normalize()); d=dh.Clamp((top-ball.location).Normalize(),(bottom-ball.location).Normalize(),line)
            k=abs((ball.location-self.top_left).Dot(n)/d.Dot(n)) if abs(d.Dot(n))>1e-6 else 0
            return self.target_surface.limit(ball.location+d*k)
        d=ball.velocity.Normalize().Clamp((left-ball.location).Normalize(),(right-ball.location).Normalize()).Clamp((top-ball.location).Normalize(),(bottom-ball.location).Normalize(),line)
        return ball.location+d.Normalize()*1000

# C#-style aliases used throughout the action port.
for _cls,_pairs in [(Game,{}),(Field,{}),(Ball,{}),(Car,{}),(Cars,{})]: pass
