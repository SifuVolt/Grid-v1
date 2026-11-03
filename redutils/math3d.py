"""Python port of Party Cannon's RedUtils.Math types (MIT, Copyright (c) 2025 RLBot)."""
from __future__ import annotations
from dataclasses import dataclass
from math import acos, cos, sin, sqrt, atan2, isfinite
from typing import Any, Iterable


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass(slots=True)
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classmethod
    def from_value(cls, value: Any) -> "Vec3":
        if isinstance(value, cls):
            return cls(value.x, value.y, value.z)
        if value is None:
            return cls()
        if isinstance(value, (tuple, list)):
            v = list(value)
            return cls(float(v[0]) if len(v) > 0 else 0.0,
                       float(v[1]) if len(v) > 1 else 0.0,
                       float(v[2]) if len(v) > 2 else 0.0)
        if hasattr(value, "pitch") or hasattr(value, "Pitch"):
            return cls(
                float(getattr(value, "pitch", getattr(value, "Pitch", 0.0))),
                float(getattr(value, "yaw", getattr(value, "Yaw", 0.0))),
                float(getattr(value, "roll", getattr(value, "Roll", 0.0))),
            )
        return cls(float(getattr(value, "x", getattr(value, "X", 0.0))),
                   float(getattr(value, "y", getattr(value, "Y", 0.0))),
                   float(getattr(value, "z", getattr(value, "Z", 0.0))))

    def __iter__(self): return iter((self.x, self.y, self.z))
    def __getitem__(self, i: int) -> float: return (self.x, self.y, self.z)[i]
    def __setitem__(self, i: int, v: float) -> None:
        if i == 0: self.x = v
        elif i == 1: self.y = v
        elif i == 2: self.z = v
        else: raise IndexError(i)
    def __add__(self, o: Any) -> "Vec3":
        o = Vec3.from_value(o); return Vec3(self.x+o.x, self.y+o.y, self.z+o.z)
    __radd__ = __add__
    def __sub__(self, o: Any) -> "Vec3":
        o = Vec3.from_value(o); return Vec3(self.x-o.x, self.y-o.y, self.z-o.z)
    def __neg__(self) -> "Vec3": return Vec3(-self.x, -self.y, -self.z)
    def __mul__(self, o: Any) -> "Vec3":
        if isinstance(o, (int, float)): return Vec3(self.x*o, self.y*o, self.z*o)
        o = Vec3.from_value(o); return Vec3(self.x*o.x, self.y*o.y, self.z*o.z)
    __rmul__ = __mul__
    def __truediv__(self, o: Any) -> "Vec3":
        if isinstance(o, (int, float)):
            if o == 0: raise ZeroDivisionError("Vec3 division by zero")
            return Vec3(self.x/o, self.y/o, self.z/o)
        o = Vec3.from_value(o); return Vec3(self.x/o.x, self.y/o.y, self.z/o.z)
    def __repr__(self) -> str: return f"({self.x:g}, {self.y:g}, {self.z:g})"

    @property
    def length(self) -> float: return sqrt(self.x*self.x+self.y*self.y+self.z*self.z)
    def Length(self) -> float: return self.length
    def Dot(self, v: Any):
        if isinstance(v, Mat3x3):
            return v.Forward*self.x + v.Right*self.y + v.Up*self.z
        v=Vec3.from_value(v); return self.x*v.x+self.y*v.y+self.z*v.z
    def Cross(self, v: Any = None) -> "Vec3":
        if v is None: return Vec3(self.y, -self.x, 0.0)
        v=Vec3.from_value(v)
        return Vec3(self.y*v.z-self.z*v.y, self.z*v.x-self.x*v.z, self.x*v.y-self.y*v.x)
    def Normalize(self) -> "Vec3": return self/self.length if self.length > 1e-12 else Vec3()
    def NormalizeInPlace(self) -> "Vec3":
        n=self.Normalize(); self.x,self.y,self.z=n.x,n.y,n.z; return self
    def Flatten(self, up: Any = None) -> "Vec3":
        if up is None: return Vec3(self.x,self.y,0)
        up=Vec3.from_value(up).Normalize(); return self-up*self.Dot(up)
    def Flattened(self, up: Any = None) -> "Vec3": return self.Flatten(up)
    def FlatNorm(self, up: Any = None) -> "Vec3": return self.Flatten(up).Normalize()
    def FlatLen(self, up: Any = None) -> float: return self.Flatten(up).length
    def Dist(self, v: Any) -> float: return (Vec3.from_value(v)-self).length
    def FlatDist(self, v: Any, up: Any = None) -> float: return (Vec3.from_value(v)-self).Flatten(up).length
    def Direction(self, v: Any) -> "Vec3": return (Vec3.from_value(v)-self).Normalize()
    def FlatDirection(self, v: Any, up: Any = None) -> "Vec3": return (Vec3.from_value(v)-self).FlatNorm(up)
    def Rescale(self, n: float) -> "Vec3": return self.Normalize()*n
    def Cap(self, low: float, high: float) -> "Vec3": return self.Rescale(clamp(self.length,low,high))
    def Angle(self, v: Any) -> float:
        v=Vec3.from_value(v); return acos(clamp(self.Normalize().Dot(v.Normalize()),-1,1))
    def FlatAngle(self, v: Any, up: Any = None) -> float:
        a=self.Flatten(up).Normalize(); b=Vec3.from_value(v).Flatten(up).Normalize()
        return acos(clamp(a.Dot(b),-1,1))
    def Clamp(self, start: Any, end: Any, up: Any = None) -> "Vec3":
        start=Vec3.from_value(start); end=Vec3.from_value(end); up=Vec3.Up if up is None else Vec3.from_value(up).Normalize()
        v=self.Normalize(); right=v.Dot(end.Cross(-up))<0; left=v.Dot(start.Cross(-up))>0
        between=(left and right) if end.Dot(start.Cross(-up))>0 else (left or right)
        if between: return Vec3(self.x,self.y,self.z)
        chosen=end if start.Dot(v)<end.Dot(v) else start
        return chosen.FlatNorm(up)*self.FlatLen(up)+up*self.Dot(up)
    def Rotate(self, angle: float, axis: Any = None) -> "Vec3":
        axis=Vec3.Up if axis is None else Vec3.from_value(axis).Normalize()
        c,s=cos(angle),sin(angle)
        return self*c + self.Cross(axis)*s + axis*(axis.Dot(self)*(1-c))

    @staticmethod
    def Abs(v: Any) -> "Vec3":
        v=Vec3.from_value(v); return Vec3(abs(v.x),abs(v.y),abs(v.z))

    Zero = None
    Up = None
    Z = None
    X = None
    Y = None

Vec3.Zero=Vec3(0,0,0); Vec3.Up=Vec3(0,0,1); Vec3.Z=Vec3(0,0,1); Vec3.X=Vec3(1,0,0); Vec3.Y=Vec3(0,1,0)


class Mat3x3:
    """3x3 car orientation matrix; rows are forward, right and up vectors."""
    __slots__=("data",)
    def __init__(self, a: Any = None, b: Any = None, c: Any = None):
        if b is not None and c is not None:
            self.data=[Vec3.from_value(a),Vec3.from_value(b),Vec3.from_value(c)]
        elif isinstance(a, (tuple,list)) and len(a)==3 and all(hasattr(v,"x") or isinstance(v,(tuple,list,Vec3)) for v in a):
            self.data=[Vec3.from_value(v) for v in a]
        else:
            r=Vec3.from_value(a)
            cp,sp=cos(r.x),sin(r.x); cy,sy=cos(r.y),sin(r.y); cr,sr=cos(r.z),sin(r.z)
            self.data=[Vec3(cp*cy,cp*sy,sp),
                       Vec3(cy*sp*sr-cr*sy,sy*sp*sr+cr*cy,-cp*sr),
                       Vec3(-cr*cy*sp-sr*sy,-cr*sy*sp+sr*cy,cp*cr)]
    @property
    def Data(self): return self.data
    @Data.setter
    def Data(self, values): self.data=[Vec3.from_value(v) for v in values]
    @property
    def Forward(self): return self.data[0]
    @property
    def Right(self): return self.data[1]
    @property
    def Up(self): return self.data[2]
    def __getitem__(self,i): return self.data[i]
    def __setitem__(self,i,v): self.data[i]=Vec3.from_value(v)
    def Dot(self,v):
        v=Vec3.from_value(v); return Vec3(self.Forward.Dot(v),self.Right.Dot(v),self.Up.Dot(v))
    def Transpose(self):
        return Mat3x3(Vec3(self.Forward.x,self.Right.x,self.Up.x),Vec3(self.Forward.y,self.Right.y,self.Up.y),Vec3(self.Forward.z,self.Right.z,self.Up.z))
    @staticmethod
    def RotationFromAxis(axis, angle):
        axis=Vec3.from_value(axis).Normalize(); c,s=cos(angle),sin(angle); n=1-c; x,y,z=axis
        return Mat3x3(Vec3(c+x*x*n,x*y*n-z*s,x*z*n+y*s),Vec3(y*x*n+z*s,c+y*y*n,y*z*n-x*s),Vec3(z*x*n-y*s,z*y*n+x*s,c+z*z*n))
