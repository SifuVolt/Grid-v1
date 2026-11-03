"""Physics and mathematical helpers translated from RedUtils.Utils.cs."""
from __future__ import annotations
from math import exp, sqrt
from .math3d import Vec3, clamp


def cap(value, low, high): return clamp(value, low, high)
def lerp(t, a, b): return (b-a)*t+a
def invlerp(v, a, b): return (v-a)/(b-a) if b != a else 0.0

def quadratic(a: float,b: float,c: float):
    d=b*b-4*a*c
    if a == 0 or d < 0: return (-1.0,-1.0)
    q=sqrt(d); return ((-b+q)/(2*a),(-b-q)/(2*a))

def time_to_jump(up: Vec3, height: float, double_jump: bool=False) -> float:
    from .objects import Car, Game
    gravity = up.Dot(Game.gravity) or -0.001
    jump_h = Car.JUMP_VEL*Car.JUMP_MAX_DURATION + Car.JUMP_ACCEL*Car.JUMP_MAX_DURATION**2/2 - Car.STICKY_ACCEL*0.05*(Car.JUMP_MAX_DURATION-0.025) + gravity*Car.JUMP_MAX_DURATION**2/2
    mult=2 if double_jump else 1
    initial=Car.JUMP_VEL*mult-16.25+(gravity+Car.JUMP_ACCEL)*Car.JUMP_MAX_DURATION
    final=sqrt(max(initial**2+2*gravity*(height-jump_h),0))
    if height < jump_h:
        fin=sqrt(max((Car.JUMP_VEL-16.25)**2+2*(gravity+Car.JUMP_ACCEL)*height,0))
        return (fin-Car.JUMP_VEL+16.25)/(gravity+Car.JUMP_ACCEL)
    return Car.JUMP_MAX_DURATION+(final-initial)/gravity

def predict_rotation(acceleration: float, time: float) -> float:
    from .objects import Car
    if acceleration*time > Car.MAX_ANGULAR_VEL:
        t=Car.MAX_ANGULAR_VEL/acceleration
        return acceleration*t*t/2+Car.MAX_ANGULAR_VEL*(time-t)
    return acceleration*time*time/2

def shot_power_modifier(value: float) -> float:
    from .objects import Car
    value=cap(value,0,4600)
    if value<=500: return 0.65
    if value<=Car.MAX_SPEED: return lerp((value-500)/1800,0.65,0.55)
    return lerp((value-Car.MAX_SPEED)/4600,0.55,0.3)

# C#-style aliases retained to make the port easy to compare with upstream.
Cap=cap; Lerp=lerp; Invlerp=invlerp; Quadratic=quadratic; TimeToJump=time_to_jump; PredictRotation=predict_rotation; ShotPowerModifier=shot_power_modifier
