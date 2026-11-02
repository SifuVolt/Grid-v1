
"""Small, dependency-free geometry and numerical utilities used by Grid."""
from __future__ import annotations
from dataclasses import dataclass
from math import atan2, cos, sin, sqrt, pi, exp
from typing import Iterable, Sequence

EPS = 1e-8
FIELD_HALF_LENGTH = 5120.0
FIELD_HALF_WIDTH = 4096.0
CEILING = 2044.0
GOAL_HALF_WIDTH = 892.755
GOAL_HEIGHT = 642.775
BALL_RADIUS = 91.25
CAR_LENGTH = 118.0
CAR_WIDTH = 84.2
CAR_HEIGHT = 36.16
BOOST_SMALL = 12
BOOST_LARGE = 100

@dataclass(frozen=True, slots=True)
class Vec3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classmethod
    def from_obj(cls, obj) -> "Vec3":
        if obj is None:
            return cls()
        return cls(float(getattr(obj, "x", 0.0)),
                   float(getattr(obj, "y", 0.0)),
                   float(getattr(obj, "z", 0.0)))

    def __add__(self, other):
        other = as_vec(other)
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)

    def __sub__(self, other):
        other = as_vec(other)
        return Vec3(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar: float):
        return Vec3(self.x * scalar, self.y * scalar, self.z * scalar)

    __rmul__ = __mul__

    def __truediv__(self, scalar: float):
        if abs(scalar) < EPS:
            return Vec3()
        return Vec3(self.x / scalar, self.y / scalar, self.z / scalar)

    def dot(self, other) -> float:
        other = as_vec(other)
        return self.x*other.x + self.y*other.y + self.z*other.z

    def cross(self, other) -> "Vec3":
        other = as_vec(other)
        return Vec3(self.y*other.z-self.z*other.y,
                    self.z*other.x-self.x*other.z,
                    self.x*other.y-self.y*other.x)

    def magnitude(self) -> float:
        return sqrt(self.x*self.x + self.y*self.y + self.z*self.z)

    def flat_magnitude(self) -> float:
        return sqrt(self.x*self.x + self.y*self.y)

    def normalized(self) -> "Vec3":
        m = self.magnitude()
        return self / m if m > EPS else Vec3()

    def flat_normalized(self) -> "Vec3":
        m = self.flat_magnitude()
        return Vec3(self.x/m, self.y/m, 0.0) if m > EPS else Vec3()

    def distance(self, other) -> float:
        return (self - other).magnitude()

    def flat_distance(self, other) -> float:
        return (self - other).flat_magnitude()

    def clamp_magnitude(self, maximum: float) -> "Vec3":
        m = self.magnitude()
        return self * (maximum / m) if m > maximum > 0 else self

    def with_z(self, z: float) -> "Vec3":
        return Vec3(self.x, self.y, z)

    def tuple(self):
        return (self.x, self.y, self.z)

def as_vec(value) -> Vec3:
    return value if isinstance(value, Vec3) else Vec3.from_obj(value)

def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))

def lerp(a: float, b: float, t: float) -> float:
    return a + (b-a)*clamp(t, 0.0, 1.0)

def sign(value: float) -> int:
    return 1 if value >= 0 else -1

def safe_acos(value: float) -> float:
    from math import acos
    return acos(clamp(value, -1.0, 1.0))

def angle_between(a, b) -> float:
    av, bv = as_vec(a).normalized(), as_vec(b).normalized()
    if av.magnitude() < EPS or bv.magnitude() < EPS:
        return pi
    return safe_acos(av.dot(bv))

def rotate_2d(vector, angle: float) -> Vec3:
    v = as_vec(vector)
    c, s = cos(angle), sin(angle)
    return Vec3(v.x*c-v.y*s, v.x*s+v.y*c, v.z)

def distance_point_segment(point, start, end) -> float:
    p, a, b = as_vec(point), as_vec(start), as_vec(end)
    ab = b-a
    denom = ab.dot(ab)
    if denom < EPS:
        return p.distance(a)
    t = clamp((p-a).dot(ab)/denom, 0.0, 1.0)
    return p.distance(a + ab*t)

def smoothstep(edge0: float, edge1: float, x: float) -> float:
    if abs(edge1-edge0) < EPS:
        return float(x >= edge1)
    t = clamp((x-edge0)/(edge1-edge0), 0.0, 1.0)
    return t*t*(3-2*t)

def exp_decay(value: float, rate: float, dt: float) -> float:
    return value * exp(-max(0.0, rate*dt))

def team_sign(team: int) -> int:
    """Blue attacks +Y; orange attacks -Y."""
    return 1 if int(team) == 0 else -1

def to_attack_coordinates(vector, team: int) -> Vec3:
    v = as_vec(vector)
    return Vec3(v.x, v.y*team_sign(team), v.z)

def from_attack_coordinates(vector, team: int) -> Vec3:
    return to_attack_coordinates(vector, team)

def own_goal_y(team: int) -> float:
    return -5120.0 if team == 0 else 5120.0

def enemy_goal_y(team: int) -> float:
    return -own_goal_y(team)

def is_in_own_half(point, team: int, margin: float = 0.0) -> bool:
    p = to_attack_coordinates(point, team)
    return p.y < margin

def inside_field(point, margin: float = 0.0) -> bool:
    p = as_vec(point)
    return abs(p.x) <= FIELD_HALF_WIDTH-margin and abs(p.y) <= FIELD_HALF_LENGTH-margin

def ground_distance(a, b) -> float:
    return as_vec(a).flat_distance(b)

def forward_vector(yaw: float) -> Vec3:
    return Vec3(cos(yaw), sin(yaw), 0.0)

def local_coordinates(world_vector, yaw: float) -> Vec3:
    v = as_vec(world_vector)
    c, s = cos(yaw), sin(yaw)
    return Vec3(v.x*c+v.y*s, -v.x*s+v.y*c, v.z)

def world_coordinates(local_vector, yaw: float) -> Vec3:
    v = as_vec(local_vector)
    c, s = cos(yaw), sin(yaw)
    return Vec3(v.x*c-v.y*s, v.x*s+v.y*c, v.z)

def weighted_average(items: Iterable[tuple[float, float]]) -> float:
    total = sum(max(0.0, w) for _, w in items)
    return sum(v*max(0.0, w) for v, w in items)/total if total > EPS else 0.0
