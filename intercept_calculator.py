"""Party Cannon's reachability and interception advantage calculations."""
from __future__ import annotations
from math import sqrt
from redutils.math3d import Vec3
from redutils.objects import Ball, Car

MAX_SPEED=2300.0; BOOST_ACCEL=991.666; BOOST_CONSUMPTION=33.333; THROTTLE_ACCEL=1600.0; MAX_THROTTLE_SPEED=1410.0
BOOST_SPEEDS=(0,500,1000,1410,1800,2000,2300); BOOST_TIMES=(0,.5,1,1.4,1.8,2.1,2.4)

class InterceptInfo:
    def __init__(self,our_best_time=float("inf"),our_best_car=None,our_best_ball_position=None,opponent_best_time=float("inf"),opponent_best_car=None,opponent_best_ball_position=None):
        self.our_best_time=our_best_time;self.our_best_car=our_best_car;self.our_best_ball_position=our_best_ball_position or Ball.location
        self.opponent_best_time=opponent_best_time;self.opponent_best_car=opponent_best_car;self.opponent_best_ball_position=opponent_best_ball_position or Ball.location
        self.opponents_are_faster=opponent_best_time<our_best_time-.1;self.we_are_faster=our_best_time<opponent_best_time-.1
    @property
    def time_difference(self):return self.opponent_best_time-self.our_best_time
    @property
    def time_advantage(self):return max(0,self.time_difference)

class InterceptCalculator:
    @staticmethod
    def EstimateTime(car,target,allow_backwards=True):
        eta=InterceptCalculator._directional(car,target,1)
        if not allow_backwards:return eta
        reverse_speed=max(-car.velocity.Dot(car.forward),0)
        if reverse_speed>500 or car.location.Dist(target)<300 or car.location.Dist(target)>3000:return min(eta,InterceptCalculator._directional(car,target,-1)+.5)
        return eta
    @staticmethod
    def _directional(car,target,direction):
        dist=car.location.Dist(target)
        if dist<100:return 0
        speed=max(car.velocity.Dot(car.forward)*direction,0); to=(Vec3.from_value(target)-car.location).Normalize(); angle=(car.forward*direction).Angle(to)
        radius=InterceptCalculator.GetTurningRadius(max(speed,500));turn=angle*radius/1800
        if angle<.5:turn=0
        total=turn;left=dist-200
        if left<=0:return turn
        if car.boost>0 and direction>0:
            bt=min(car.boost/33.333,2.4);after=InterceptCalculator._interpolate(BOOST_TIMES,BOOST_SPEEDS,bt,speed);bd=(speed+after)/2*bt
            if bd>=left:return total+InterceptCalculator._solve(speed,991.666,left)
            total+=bt;left-=bd;speed=after
        if speed<1410 and left>0:
            t=(1410-speed)/1600;d=speed*t+800*t*t
            if d>=left:return total+InterceptCalculator._solve(speed,1600,left)
            total+=t;left-=d;speed=1410
        if left>0:total+=left/max(speed,800)
        return total*1.05
    @staticmethod
    def GetTurningRadius(speed):
        k=.0069-6.19e-6*speed+1.1e-8*speed*speed
        return 1/max(k,.001)
    @staticmethod
    def _interpolate(times,speeds,t,base):
        for i in range(len(times)-1):
            if t<=times[i+1]:
                f=(t-times[i])/(times[i+1]-times[i]);return base+speeds[i]+f*(speeds[i+1]-speeds[i])
        return base+speeds[-1]
    @staticmethod
    def _solve(v,a,d):
        inside=v*v+2*a*d
        return d/max(v,400) if inside<0 else (-v+sqrt(inside))/a
    @staticmethod
    def CalculateInterceptAdvantage(our_team,opponents,ball):
        our_t=opp_t=float("inf");our_car=opp_car=None;our_pos=opp_pos=Ball.location
        t=0.0
        while t<=3.0001:
            bp=Ball.location+Ball.velocity*t+Vec3(0,0,-325)*t*t;bp.x=max(-4096,min(4096,bp.x));bp.y=max(-5120,min(5120,bp.y));bp.z=max(100,bp.z)
            if our_car is None:
                for mate in our_team:
                    eta=InterceptCalculator.EstimateTime(mate,bp)
                    if eta<=t+.05 and eta<our_t:our_t=eta;our_car=mate;our_pos=bp
            if opp_car is None:
                for opp in opponents:
                    eta=InterceptCalculator.EstimateTime(opp,bp)
                    if eta<=t+.05 and eta<opp_t:opp_t=eta;opp_car=opp;opp_pos=bp
            if our_car is not None and opp_car is not None:break
            t+=.1
        return InterceptInfo(our_t,our_car,our_pos,opp_t,opp_car,opp_pos)
