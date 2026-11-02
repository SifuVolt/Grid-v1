
"""Rocket League vehicle state and movement properties.

This module contains the shared representation of a car used throughout Grid.
It does not read RLBot packets directly; util/game_state.py will construct
VehicleState instances from the current game packet.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace
from math import atan2, pi, sqrt

from .math3d import (
    Vec3,
    Orientation,
    ZERO,
    UP,
    clamp,
)


# ---------------------------------------------------------------------------
# Approximate Rocket League vehicle constants
# ---------------------------------------------------------------------------

MAX_FORWARD_SPEED = 2300.0
MAX_REVERSE_SPEED = 1400.0
MAX_GROUND_SPEED = MAX_FORWARD_SPEED

MAX_BOOST = 100.0
BOOST_CONSUMPTION = 33.333
BOOST_ACCELERATION = 991.667

THROTTLE_ACCELERATION = 1600.0
MAX_THROTTLE_SPEED = 1410.0

BRAKE_ACCELERATION = 3500.0

JUMP_IMPULSE = 291.667
JUMP_ACCELERATION = 1458.333
GRAVITY = 650.0

CAR_RESTING_HEIGHT = 17.0
CAR_LENGTH = 118.0
CAR_WIDTH = 84.0
CAR_HEIGHT = 36.0

# Approximate yaw-rate ceiling at low speed. Actual turning is speed-dependent.
MAX_YAW_RATE = 5.5


@dataclass(slots=True)
class VehicleState:
    """A snapshot of one Rocket League car.

    orientation is the car's world-space basis:
      forward: nose direction
      right: car's right side
      up: roof direction

    The default orientation faces toward positive Y.
    """

    location: Vec3 = field(default_factory=Vec3)
    velocity: Vec3 = field(default_factory=Vec3)

    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

    boost: float = 0.0
    on_ground: bool = True
    demolished: bool = False

    angular_velocity: Vec3 = field(default_factory=Vec3)
    team: int = 0
    index: int = -1

    orientation: Orientation | None = None

    # Useful packet-derived flags.
    has_wheel_contact: bool | None = None
    supersonic: bool = False
    is_bot: bool = False

    def __post_init__(self) -> None:
        self.boost = clamp(float(self.boost), 0.0, MAX_BOOST)

        self.orientation = Orientation.from_rotator(
            pitch=self.pitch,
            yaw=self.yaw,
            roll=self.roll,
        )

        # Keep the Euler fields consistent with the orientation supplied by
        # the game-state converter.
        self.angular_velocity = Vec3.from_value(self.angular_velocity)

    @property
    def forward(self) -> Vec3:
        return self.orientation.forward

    @property
    def right(self) -> Vec3:
        return self.orientation.right

    @property
    def up(self) -> Vec3:
        return self.orientation.up

    @property
    def speed(self) -> float:
        return self.velocity.length()

    @property
    def flat_speed(self) -> float:
        return self.velocity.flat_length()

    @property
    def forward_speed(self) -> float:
        return self.velocity.dot(self.forward)

    @property
    def lateral_speed(self) -> float:
        return self.velocity.dot(self.right)

    @property
    def vertical_speed(self) -> float:
        return self.velocity.z

    @property
    def boost_fraction(self) -> float:
        return self.boost / MAX_BOOST

    @property
    def has_boost(self) -> bool:
        return self.boost > 0.0

    @property
    def is_supersonic(self) -> bool:
        return self.supersonic or self.speed >= 2200.0

    @property
    def is_airborne(self) -> bool:
        return not self.on_ground

    @property
    def is_alive(self) -> bool:
        return not self.demolished

    @property
    def forward_flat(self) -> Vec3:
        return self.forward.flat_normalized()

    @property
    def right_flat(self) -> Vec3:
        return self.right.flat_normalized()

    @property
    def angular_velocity_local(self) -> Vec3:
        """Angular velocity expressed in the car's local coordinate frame."""
        return Vec3(
            self.angular_velocity.dot(self.forward),
            self.angular_velocity.dot(self.right),
            self.angular_velocity.dot(self.up),
        )

    def copy(self, **changes) -> VehicleState:
        """Return a copy with selected fields changed."""
        return replace(self, **changes)

    def distance_to(self, point: Vec3) -> float:
        return self.location.distance(point)

    def flat_distance_to(self, point: Vec3) -> float:
        return self.location.flat_distance(point)

    def direction_to(self, point: Vec3) -> Vec3:
        return self.location.direction_to(point)

    def flat_direction_to(self, point: Vec3) -> Vec3:
        return self.location.flat_direction_to(point)

    def angle_to(self, point: Vec3) -> float:
        """3D angle between the car's nose and a target."""
        direction = self.direction_to(point)
        if direction.length_squared() <= 1e-9:
            return 0.0
        return self.forward.angle_to(direction)

    def flat_angle_to(self, point: Vec3) -> float:
        """Signed yaw error to a target, in radians."""
        direction = self.flat_direction_to(point)
        forward = self.forward_flat

        if direction.length_squared() <= 1e-9:
            return 0.0

        return atan2(
            forward.x * direction.y - forward.y * direction.x,
            forward.dot(direction),
        )

    def facing_dot(self, point: Vec3) -> float:
        """How directly the car faces a target. -1 to +1."""
        direction = self.direction_to(point)
        if direction.length_squared() <= 1e-9:
            return 1.0
        return self.forward.dot(direction)

    def facing_dot_flat(self, point: Vec3) -> float:
        direction = self.flat_direction_to(point)
        if direction.length_squared() <= 1e-9:
            return 1.0
        return self.forward_flat.dot(direction)

    def boost_time(self) -> float:
        """Maximum continuous boost time available."""
        if BOOST_CONSUMPTION <= 0:
            return 0.0
        return self.boost / BOOST_CONSUMPTION

    def can_boost(self) -> bool:
        return self.boost > 0.0 and not self.demolished

    def turning_radius(self) -> float:
        """Approximate turning radius from current speed.

        Uses the curvature approximation commonly used by RL bot
        intercept estimators. This is a planning estimate, not a
        wheel-contact simulation.
        """
        speed = clamp(self.flat_speed, 0.0, MAX_FORWARD_SPEED)
        curvature = (
            0.0069
            - 6.19e-6 * speed
            + 1.1e-8 * speed * speed
        )
        curvature = max(curvature, 1e-5)
        return 1.0 / curvature

    def maximum_yaw_rate(self) -> float:
        """Approximate yaw-rate limit at the current speed."""
        speed_factor = 1.0 - 0.55 * (
            clamp(self.flat_speed, 0.0, MAX_FORWARD_SPEED)
            / MAX_FORWARD_SPEED
        )
        return MAX_YAW_RATE * max(0.25, speed_factor)

    def can_reach_speed(self, target_speed: float) -> bool:
        return target_speed <= MAX_FORWARD_SPEED

    def with_orientation(self, orientation: Orientation) -> VehicleState:
        return replace(self, orientation=orientation)


@dataclass(frozen=True, slots=True)
class VehicleDimensions:
    """Collision dimensions used by approximate reach and contact checks."""

    length: float = CAR_LENGTH
    width: float = CAR_WIDTH
    height: float = CAR_HEIGHT
    resting_height: float = CAR_RESTING_HEIGHT


DEFAULT_VEHICLE_DIMENSIONS = VehicleDimensions()


def speed_after_acceleration(
    initial_speed: float,
    acceleration: float,
    duration: float,
    maximum_speed: float = MAX_FORWARD_SPEED,
) -> float:
    """Estimate final speed under constant positive acceleration."""
    if duration <= 0.0:
        return initial_speed
    return min(maximum_speed, initial_speed + acceleration * duration)


def distance_under_acceleration(
    initial_speed: float,
    acceleration: float,
    duration: float,
    maximum_speed: float = MAX_FORWARD_SPEED,
) -> float:
    """Estimate distance travelled while accelerating to a speed cap."""
    if duration <= 0.0:
        return 0.0

    initial_speed = max(0.0, initial_speed)
    acceleration = max(0.0, acceleration)

    if acceleration <= 1e-9:
        return min(initial_speed, maximum_speed) * duration

    if initial_speed >= maximum_speed:
        return maximum_speed * duration

    time_to_cap = (maximum_speed - initial_speed) / acceleration

    if duration <= time_to_cap:
        return initial_speed * duration + 0.5 * acceleration * duration**2

    accelerating_distance = (
        initial_speed * time_to_cap
        + 0.5 * acceleration * time_to_cap**2
    )
    remaining_time = duration - time_to_cap

    return accelerating_distance + maximum_speed * remaining_time


def estimate_boost_used(duration: float) -> float:
    return max(0.0, duration) * BOOST_CONSUMPTION


def estimate_boost_acceleration(
    vehicle: VehicleState,
    duration: float,
) -> float:
    """Estimate extra speed from boost over a time interval."""
    if duration <= 0.0 or not vehicle.can_boost():
        return 0.0

    usable_time = min(duration, vehicle.boost_time())
    return BOOST_ACCELERATION * usable_time


def estimate_stopping_distance(
    speed: float,
    deceleration: float = BRAKE_ACCELERATION,
) -> float:
    """Estimate straight-line stopping distance."""
    if deceleration <= 0.0:
        return float("inf")
    speed = abs(speed)
    return speed * speed / (2.0 * deceleration)


def estimate_jump_height(
    hold_time: float = 0.2,
    total_time: float = 1.0,
) -> float:
    """Approximate vertical jump displacement.

    The jump impulse is applied once. Jump acceleration is applied only
    during the requested hold interval, then gravity acts throughout.
    This is a simplified planning estimate.
    """
    total_time = max(0.0, total_time)
    hold_time = clamp(hold_time, 0.0, min(0.2, total_time))

    displacement = JUMP_IMPULSE * total_time
    displacement += 0.5 * JUMP_ACCELERATION * hold_time**2
    displacement += JUMP_ACCELERATION * hold_time * (
        total_time - hold_time
    )
    displacement -= 0.5 * GRAVITY * total_time**2

    return displacement


def estimate_air_time(
    initial_vertical_speed: float = JUMP_IMPULSE,
    gravity: float = GRAVITY,
) -> float:
    """Approximate time to return to the original height."""
    if gravity <= 0.0:
        return 0.0
    return max(0.0, 2.0 * initial_vertical_speed / gravity)


def estimate_turn_time(
    angle: float,
    speed: float,
    minimum_angle: float = 0.5,
) -> float:
    """Estimate time needed to turn through an angle.

    The estimate uses Party Cannon's broad turning-radius approach:
    small angles are treated as negligible; larger turns depend on
    current turning radius and speed.
    """
    angle = abs(angle)
    if angle < minimum_angle:
        return 0.0

    speed = clamp(abs(speed), 0.0, MAX_FORWARD_SPEED)
    radius = (
        1.0
        / max(
            1e-5,
            0.0069 - 6.19e-6 * speed + 1.1e-8 * speed * speed,
        )
    )

    # Approximate arc length divided by speed, with a conservative
    # low-speed floor so the estimate remains finite.
    arc_length = radius * angle
    return arc_length / max(400.0, speed)


def estimate_time_to_speed(
    initial_speed: float,
    target_speed: float,
    acceleration: float = THROTTLE_ACCELERATION,
) -> float:
    if target_speed <= initial_speed:
        return 0.0
    if acceleration <= 0.0:
        return float("inf")
    return (target_speed - initial_speed) / acceleration


def make_vehicle_state(
    location: Vec3,
    velocity: Vec3,
    rotation: tuple[float, float, float],
    boost: float,
    on_ground: bool,
    team: int,
    index: int = -1,
    angular_velocity: Vec3 = ZERO,
    demolished: bool = False,
) -> VehicleState:
    """Convenience constructor for packet conversion code.

    rotation is ordered (pitch, yaw, roll), in radians.
    """
    pitch, yaw, roll = rotation
    orientation = Orientation.from_rotator(
        pitch=pitch,
        yaw=yaw,
        roll=roll,
    )

    return VehicleState(
        location=location,
        velocity=velocity,
        pitch=pitch,
        yaw=yaw,
        roll=roll,
        boost=boost,
        on_ground=on_ground,
        demolished=demolished,
        angular_velocity=angular_velocity,
        team=team,
        index=index,
        orientation=orientation,
    )
