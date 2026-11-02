
"""Grid 3D mathematics.

Shared vector and orientation types used throughout Grid.

Coordinate convention:
    X = field length
    Y = field width
    Z = height

Angles are in radians. Distances and velocities use Rocket League units.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import acos, atan2, cos, pi, sin, sqrt
from typing import Any, Iterable


_EPSILON = 1e-8


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


@dataclass(frozen=True, slots=True)
class Vec3:
    """Immutable three-dimensional vector."""

    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @classmethod
    def from_value(cls, value: Any) -> "Vec3":
        """Convert a tuple, list, vector-like object or RLBot vector."""
        if isinstance(value, cls):
            return value

        if value is None:
            return cls()

        if isinstance(value, (tuple, list)):
            values = list(value)
            return cls(
                float(values[0]) if len(values) > 0 else 0.0,
                float(values[1]) if len(values) > 1 else 0.0,
                float(values[2]) if len(values) > 2 else 0.0,
            )

        return cls(
            float(getattr(value, "x", 0.0)),
            float(getattr(value, "y", 0.0)),
            float(getattr(value, "z", 0.0)),
        )

    @classmethod
    def from_iterable(cls, values: Iterable[float]) -> "Vec3":
        values = list(values)
        return cls(
            float(values[0]) if len(values) > 0 else 0.0,
            float(values[1]) if len(values) > 1 else 0.0,
            float(values[2]) if len(values) > 2 else 0.0,
        )

    def __iter__(self):
        yield self.x
        yield self.y
        yield self.z

    def __getitem__(self, index: int) -> float:
        if index == 0:
            return self.x
        if index == 1:
            return self.y
        if index == 2:
            return self.z
        raise IndexError(index)

    def __add__(self, other: Any) -> "Vec3":
        other = Vec3.from_value(other)
        return Vec3(
            self.x + other.x,
            self.y + other.y,
            self.z + other.z,
        )

    def __radd__(self, other: Any) -> "Vec3":
        return self + other

    def __sub__(self, other: Any) -> "Vec3":
        other = Vec3.from_value(other)
        return Vec3(
            self.x - other.x,
            self.y - other.y,
            self.z - other.z,
        )

    def __rsub__(self, other: Any) -> "Vec3":
        return Vec3.from_value(other) - self

    def __mul__(self, scalar: float) -> "Vec3":
        scalar = float(scalar)
        return Vec3(
            self.x * scalar,
            self.y * scalar,
            self.z * scalar,
        )

    def __rmul__(self, scalar: float) -> "Vec3":
        return self * scalar

    def __truediv__(self, scalar: float) -> "Vec3":
        scalar = float(scalar)
        if abs(scalar) < _EPSILON:
            raise ZeroDivisionError("Cannot divide Vec3 by zero.")
        return Vec3(
            self.x / scalar,
            self.y / scalar,
            self.z / scalar,
        )

    def __neg__(self) -> "Vec3":
        return Vec3(-self.x, -self.y, -self.z)

    def __abs__(self) -> float:
        return self.length()

    def __repr__(self) -> str:
        return f"Vec3({self.x:.2f}, {self.y:.2f}, {self.z:.2f})"

    def dot(self, other: Any) -> float:
        other = Vec3.from_value(other)
        return (
            self.x * other.x
            + self.y * other.y
            + self.z * other.z
        )

    def cross(self, other: Any) -> "Vec3":
        other = Vec3.from_value(other)
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def length_squared(self) -> float:
        return self.dot(self)

    def length(self) -> float:
        return sqrt(self.length_squared())

    def flat_length_squared(self) -> float:
        return self.x * self.x + self.y * self.y

    def flat_length(self) -> float:
        return sqrt(self.flat_length_squared())

    def normalized(self) -> "Vec3":
        magnitude = self.length()
        if magnitude < _EPSILON:
            return Vec3()
        return self / magnitude

    def flat_normalized(self) -> "Vec3":
        magnitude = self.flat_length()
        if magnitude < _EPSILON:
            return Vec3()
        return Vec3(self.x / magnitude, self.y / magnitude, 0.0)

    def normalize(self) -> "Vec3":
        """Compatibility alias for normalized()."""
        return self.normalized()

    def flatten(self) -> "Vec3":
        return Vec3(self.x, self.y, 0.0)

    def with_z(self, z: float) -> "Vec3":
        return Vec3(self.x, self.y, float(z))

    def distance(self, other: Any) -> float:
        return (self - Vec3.from_value(other)).length()

    def flat_distance(self, other: Any) -> float:
        return (self - Vec3.from_value(other)).flat_length()

    def direction_to(self, other: Any) -> "Vec3":
        return (Vec3.from_value(other) - self).normalized()

    def flat_direction_to(self, other: Any) -> "Vec3":
        return (Vec3.from_value(other) - self).flat_normalized()

    def angle_to(self, other: Any) -> float:
        other = Vec3.from_value(other)
        denominator = self.length() * other.length()

        if denominator < _EPSILON:
            return 0.0

        cosine = clamp(self.dot(other) / denominator, -1.0, 1.0)
        return acos(cosine)

    def flat_angle_to(self, other: Any) -> float:
        return self.flatten().angle_to(Vec3.from_value(other).flatten())

    def signed_flat_angle_to(self, other: Any) -> float:
        """Signed yaw angle from this vector to another vector."""
        a = self.flat_normalized()
        b = Vec3.from_value(other).flat_normalized()

        if a.length_squared() < _EPSILON or b.length_squared() < _EPSILON:
            return 0.0

        cross_z = a.x * b.y - a.y * b.x
        dot = clamp(a.dot(b), -1.0, 1.0)
        return atan2(cross_z, dot)

    def project_onto(self, other: Any) -> "Vec3":
        other = Vec3.from_value(other)
        denominator = other.length_squared()

        if denominator < _EPSILON:
            return Vec3()

        return other * (self.dot(other) / denominator)

    def reject_from(self, other: Any) -> "Vec3":
        return self - self.project_onto(other)

    def cap(self, minimum: float, maximum: float) -> "Vec3":
        magnitude = self.length()

        if magnitude < _EPSILON:
            return self

        target = clamp(magnitude, minimum, maximum)
        return self * (target / magnitude)

    def clamp_length(self, maximum: float) -> "Vec3":
        return self.cap(0.0, maximum)

    def rotate_around(self, axis: Any, angle: float) -> "Vec3":
        """Rotate this vector around an arbitrary axis using Rodrigues' formula."""
        axis = Vec3.from_value(axis).normalized()

        if axis.length_squared() < _EPSILON:
            return self

        c = cos(angle)
        s = sin(angle)

        return (
            self * c
            + axis.cross(self) * s
            + axis * (axis.dot(self) * (1.0 - c))
        )

    def rotate_flat(self, angle: float) -> "Vec3":
        """Rotate around the vertical Z axis."""
        c = cos(angle)
        s = sin(angle)
        return Vec3(
            self.x * c - self.y * s,
            self.x * s + self.y * c,
            self.z,
        )

    def almost_equal(self, other: Any, tolerance: float = 1e-5) -> bool:
        return self.distance(other) <= tolerance

    @property
    def magnitude(self) -> float:
        return self.length()

    @property
    def flat_magnitude(self) -> float:
        return self.flat_length()


@dataclass(frozen=True, slots=True)
class Orientation:
    """Car orientation represented by orthogonal world-space basis vectors."""

    forward: Vec3
    right: Vec3
    up: Vec3

    @classmethod
    def from_rotator(
        cls,
        rotator: Any = None,
        *,
        pitch: float | None = None,
        yaw: float | None = None,
        roll: float | None = None,
    ) -> "Orientation":
        """Build orientation from an RLBot rotator.

        RLBot rotator angles are in radians:
            pitch: nose up/down
            yaw: heading around Z
            roll: rotation around the forward axis
        """
        if rotator is not None:
            pitch = float(getattr(rotator, "pitch", 0.0))
            yaw = float(getattr(rotator, "yaw", 0.0))
            roll = float(getattr(rotator, "roll", 0.0))

        pitch = float(pitch or 0.0)
        yaw = float(yaw or 0.0)
        roll = float(roll or 0.0)

        cp = cos(pitch)
        sp = sin(pitch)
        cy = cos(yaw)
        sy = sin(yaw)
        cr = cos(roll)
        sr = sin(roll)

        forward = Vec3(cp * cy, cp * sy, sp)

        right = Vec3(
            cy * sp * sr - cr * sy,
            sy * sp * sr + cr * cy,
            -cp * sr,
        )

        up = Vec3(
            -cr * cy * sp - sr * sy,
            -cr * sy * sp + sr * cy,
            cr * cp,
        )

        return cls(
            forward=forward.normalized(),
            right=right.normalized(),
            up=up.normalized(),
        )

    @classmethod
    def from_basis(
        cls,
        forward: Any,
        right: Any,
        up: Any,
    ) -> "Orientation":
        forward = Vec3.from_value(forward).normalized()
        right = Vec3.from_value(right).normalized()
        up = Vec3.from_value(up).normalized()

        return cls(forward, right, up)

    def local_to_world(self, local: Any) -> Vec3:
        local = Vec3.from_value(local)
        return (
            self.forward * local.x
            + self.right * local.y
            + self.up * local.z
        )

    def world_to_local(self, world: Any) -> Vec3:
        world = Vec3.from_value(world)
        return Vec3(
            world.dot(self.forward),
            world.dot(self.right),
            world.dot(self.up),
        )


ZERO = Vec3(0.0, 0.0, 0.0)
UP = Vec3(0.0, 0.0, 1.0)
DOWN = Vec3(0.0, 0.0, -1.0)
FORWARD = Vec3(1.0, 0.0, 0.0)
RIGHT = Vec3(0.0, 1.0, 0.0)
PI = pi
TAU = 2.0 * pi
