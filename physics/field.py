
"""Rocket League field geometry and surface helpers.

Coordinates:
- X: left/right across the field
- Y: length of the field; Blue defends negative Y
- Z: height above the floor

The dimensions are standard Soccar approximations in Unreal units.
All geometry is expressed in world coordinates.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from math import copysign
from typing import Iterable

from .math3d import Vec3, UP, clamp


# ---------------------------------------------------------------------------
# Field dimensions
# ---------------------------------------------------------------------------

FIELD_HALF_LENGTH = 5120.0
FIELD_HALF_WIDTH = 4096.0
CEILING_HEIGHT = 2044.0

SIDE_WALL_X = 4096.0
BACK_WALL_Y = 5120.0

CORNER_RADIUS = 1152.0

GOAL_HALF_WIDTH = 892.75
GOAL_HEIGHT = 642.775
GOAL_DEPTH = 880.0

# The front of each goal is at the back wall.
BLUE_GOAL_Y = -FIELD_HALF_LENGTH
ORANGE_GOAL_Y = FIELD_HALF_LENGTH

BALL_RADIUS = 91.25
CAR_RESTING_HEIGHT = 17.0

# Approximate boost-pad positions. These are fixed map locations.
SMALL_BOOST_PADS: tuple[Vec3, ...] = (
    Vec3(-3072, -4096, 70),
    Vec3(-1792, -4184, 70),
    Vec3(0, -4240, 70),
    Vec3(1792, -4184, 70),
    Vec3(3072, -4096, 70),
    Vec3(-3584, -2484, 70),
    Vec3(-1024, -2484, 70),
    Vec3(1024, -2484, 70),
    Vec3(3584, -2484, 70),
    Vec3(-3584, 0, 70),
    Vec3(-1024, 0, 70),
    Vec3(1024, 0, 70),
    Vec3(3584, 0, 70),
    Vec3(-3584, 2484, 70),
    Vec3(-1024, 2484, 70),
    Vec3(1024, 2484, 70),
    Vec3(3584, 2484, 70),
    Vec3(-3072, 4096, 70),
    Vec3(-1792, 4184, 70),
    Vec3(0, 4240, 70),
    Vec3(1792, 4184, 70),
    Vec3(3072, 4096, 70),
)

# The six large pads.
LARGE_BOOST_PADS: tuple[Vec3, ...] = (
    Vec3(-3072, -4096, 70),
    Vec3(3072, -4096, 70),
    Vec3(-3584, 0, 70),
    Vec3(3584, 0, 70),
    Vec3(-3072, 4096, 70),
    Vec3(3072, 4096, 70),
)


class Surface(Enum):
    FLOOR = "floor"
    CEILING = "ceiling"
    SIDE_WALL = "side_wall"
    BACK_WALL = "back_wall"
    CORNER = "corner"
    BLUE_GOAL_BACK = "blue_goal_back"
    ORANGE_GOAL_BACK = "orange_goal_back"
    BLUE_GOAL_SIDE = "blue_goal_side"
    ORANGE_GOAL_SIDE = "orange_goal_side"


@dataclass(frozen=True, slots=True)
class SurfaceInfo:
    """Nearest detected field surface."""

    surface: Surface
    point: Vec3
    normal: Vec3
    distance: float


@dataclass(frozen=True, slots=True)
class Goal:
    """Goal geometry in world coordinates."""

    team: int
    center: Vec3
    left_post: Vec3
    right_post: Vec3
    back_center: Vec3
    forward: Vec3
    half_width: float = GOAL_HALF_WIDTH
    height: float = GOAL_HEIGHT
    depth: float = GOAL_DEPTH

    @property
    def left(self) -> Vec3:
        return self.left_post

    @property
    def right(self) -> Vec3:
        return self.right_post

    def contains(self, point: Vec3, margin: float = 0.0) -> bool:
        """Whether a point lies inside the rectangular goal opening."""
        return (
            abs(point.x - self.center.x) <= self.half_width + margin
            and point.z <= self.height + margin
            and point.z >= -margin
        )

    def mouth_distance(self, point: Vec3) -> float:
        """Absolute distance from the goal-mouth plane."""
        return abs(point.y - self.center.y)

    def distance_to_mouth(self, point: Vec3) -> float:
        """Distance to the nearest point on the goal-mouth rectangle."""
        nearest_x = clamp(
            point.x,
            self.center.x - self.half_width,
            self.center.x + self.half_width,
        )
        nearest_z = clamp(point.z, 0.0, self.height)
        nearest = Vec3(nearest_x, self.center.y, nearest_z)
        return point.distance(nearest)


BLUE_GOAL = Goal(
    team=0,
    center=Vec3(0, BLUE_GOAL_Y, GOAL_HEIGHT / 2),
    left_post=Vec3(-GOAL_HALF_WIDTH, BLUE_GOAL_Y, 0),
    right_post=Vec3(GOAL_HALF_WIDTH, BLUE_GOAL_Y, 0),
    back_center=Vec3(0, BLUE_GOAL_Y - GOAL_DEPTH, GOAL_HEIGHT / 2),
    forward=Vec3(0, 1, 0),
)

ORANGE_GOAL = Goal(
    team=1,
    center=Vec3(0, ORANGE_GOAL_Y, GOAL_HEIGHT / 2),
    left_post=Vec3(GOAL_HALF_WIDTH, ORANGE_GOAL_Y, 0),
    right_post=Vec3(-GOAL_HALF_WIDTH, ORANGE_GOAL_Y, 0),
    back_center=Vec3(0, ORANGE_GOAL_Y + GOAL_DEPTH, GOAL_HEIGHT / 2),
    forward=Vec3(0, -1, 0),
)


def goal_for_team(team: int) -> Goal:
    """Return the goal defended by the given team."""
    return BLUE_GOAL if team == 0 else ORANGE_GOAL


def enemy_goal_for_team(team: int) -> Goal:
    """Return the goal attacked by the given team."""
    return ORANGE_GOAL if team == 0 else BLUE_GOAL


def clamp_to_field(
    point: Vec3,
    margin: float = 0.0,
    include_goal_area: bool = False,
) -> Vec3:
    """Clamp a point to the playable field bounds.

    This is a rectangular clamp, not the curved corner boundary.
    Goal depth is included only when include_goal_area=True.
    """
    half_length = FIELD_HALF_LENGTH + (GOAL_DEPTH if include_goal_area else 0.0)
    return Vec3(
        clamp(point.x, -FIELD_HALF_WIDTH + margin, FIELD_HALF_WIDTH - margin),
        clamp(point.y, -half_length + margin, half_length - margin),
        max(0.0, point.z),
    )


def is_in_goal_tunnel(point: Vec3, margin: float = 0.0) -> bool:
    """Check whether a point is behind either goal line and between its posts."""
    inside_width = abs(point.x) <= GOAL_HALF_WIDTH + margin
    behind_blue = point.y < BLUE_GOAL_Y
    behind_orange = point.y > ORANGE_GOAL_Y
    return inside_width and (behind_blue or behind_orange)


def is_in_goal(point: Vec3, team: int | None = None, margin: float = 0.0) -> bool:
    """Check whether a point is inside a goal volume."""
    if team is None:
        return BLUE_GOAL.contains(point, margin) and point.y <= BLUE_GOAL_Y + margin or (
            ORANGE_GOAL.contains(point, margin)
            and point.y >= ORANGE_GOAL_Y - margin
        )

    goal = goal_for_team(team)
    if team == 0:
        return goal.contains(point, margin) and point.y <= BLUE_GOAL_Y + margin
    return goal.contains(point, margin) and point.y >= ORANGE_GOAL_Y - margin


def distance_to_goal_line(point: Vec3, team: int) -> float:
    """Signed distance from a point to the defending goal line.

    Positive means the point is in front of the goal; negative means behind it.
    """
    goal_y = goal_for_team(team).center.y
    if team == 0:
        return point.y - goal_y
    return goal_y - point.y


def distance_to_own_goal(point: Vec3, team: int) -> float:
    """3D distance to the centre of the defended goal mouth."""
    return point.distance(goal_for_team(team).center)


def distance_to_enemy_goal(point: Vec3, team: int) -> float:
    """3D distance to the centre of the opponent's goal mouth."""
    return point.distance(enemy_goal_for_team(team).center)


def in_own_half(point: Vec3, team: int, margin: float = 0.0) -> bool:
    """True when the point is on the team's defensive half."""
    if team == 0:
        return point.y < margin
    return point.y > -margin


def in_enemy_half(point: Vec3, team: int, margin: float = 0.0) -> bool:
    """True when the point is on the opponent's half."""
    if team == 0:
        return point.y > -margin
    return point.y < margin


def attack_direction(team: int) -> Vec3:
    """Unit vector pointing toward the opponent's goal."""
    return Vec3(0, 1 if team == 0 else -1, 0)


def side_sign(x: float, fallback: float = 1.0) -> float:
    """Return -1 or +1 for a side coordinate."""
    if abs(x) < 1e-6:
        return 1.0 if fallback >= 0 else -1.0
    return copysign(1.0, x)


def nearest_surface(
    point: Vec3,
    include_ceiling: bool = False,
    include_corners: bool = True,
) -> SurfaceInfo:
    """Find the nearest approximate field surface.

    The ceiling is excluded by default because it is rarely the relevant
    surface for ordinary ground navigation.
    """
    candidates: list[SurfaceInfo] = []

    # Floor.
    candidates.append(
        SurfaceInfo(
            Surface.FLOOR,
            Vec3(point.x, point.y, 0),
            UP,
            abs(point.z),
        )
    )

    # Side walls.
    right_x = FIELD_HALF_WIDTH
    left_x = -FIELD_HALF_WIDTH

    candidates.append(
        SurfaceInfo(
            Surface.SIDE_WALL,
            Vec3(right_x, point.y, point.z),
            Vec3(-1, 0, 0),
            abs(right_x - point.x),
        )
    )
    candidates.append(
        SurfaceInfo(
            Surface.SIDE_WALL,
            Vec3(left_x, point.y, point.z),
            Vec3(1, 0, 0),
            abs(point.x - left_x),
        )
    )

    # Back walls. The goal openings are excluded from these flat-wall
    # candidates so the goal tunnel can be handled separately.
    if abs(point.x) > GOAL_HALF_WIDTH:
        candidates.extend(
            (
                SurfaceInfo(
                    Surface.BACK_WALL,
                    Vec3(point.x, BACK_WALL_Y, point.z),
                    Vec3(0, -1, 0),
                    abs(BACK_WALL_Y - point.y),
                ),
                SurfaceInfo(
                    Surface.BACK_WALL,
                    Vec3(point.x, -BACK_WALL_Y, point.z),
                    Vec3(0, 1, 0),
                    abs(point.y + BACK_WALL_Y),
                ),
            )
        )

    # Approximate rounded corner boundaries. These are useful for
    # identifying corner play, not for collision-accurate simulation.
    if include_corners:
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                cx = sx * (FIELD_HALF_WIDTH - CORNER_RADIUS)
                cy = sy * (FIELD_HALF_LENGTH - CORNER_RADIUS)
                dx = point.x - cx
                dy = point.y - cy
                radial = Vec3(dx, dy, 0)
                radial_length = radial.length()

                if radial_length > 1e-6:
                    normal = radial / radial_length
                else:
                    normal = Vec3(sx, sy, 0).normalized()

                boundary = Vec3(
                    cx + normal.x * CORNER_RADIUS,
                    cy + normal.y * CORNER_RADIUS,
                    point.z,
                )
                candidates.append(
                    SurfaceInfo(
                        Surface.CORNER,
                        boundary,
                        -normal,
                        abs(radial_length - CORNER_RADIUS),
                    )
                )

    # Goal tunnel back and side walls.
    if abs(point.x) <= GOAL_HALF_WIDTH and point.z <= GOAL_HEIGHT:
        candidates.extend(
            (
                SurfaceInfo(
                    Surface.BLUE_GOAL_BACK,
                    Vec3(point.x, BLUE_GOAL_Y - GOAL_DEPTH, point.z),
                    Vec3(0, 1, 0),
                    abs(point.y - (BLUE_GOAL_Y - GOAL_DEPTH)),
                ),
                SurfaceInfo(
                    Surface.ORANGE_GOAL_BACK,
                    Vec3(point.x, ORANGE_GOAL_Y + GOAL_DEPTH, point.z),
                    Vec3(0, -1, 0),
                    abs(point.y - (ORANGE_GOAL_Y + GOAL_DEPTH)),
                ),
            )
        )

    if include_ceiling:
        candidates.append(
            SurfaceInfo(
                Surface.CEILING,
                Vec3(point.x, point.y, CEILING_HEIGHT),
                Vec3(0, 0, -1),
                abs(CEILING_HEIGHT - point.z),
            )
        )

    return min(candidates, key=lambda candidate: candidate.distance)


def nearest_wall(point: Vec3) -> SurfaceInfo:
    """Return the nearest vertical wall or corner."""
    candidates = [
        nearest_surface(point, include_ceiling=False, include_corners=True)
    ]
    candidates = [
        candidate
        for candidate in candidates
        if candidate.surface != Surface.FLOOR
    ]
    if candidates:
        return candidates[0]

    # Fallback: nearest of the four rectangular wall planes.
    distances = (
        (abs(FIELD_HALF_WIDTH - point.x), Vec3(-1, 0, 0), Surface.SIDE_WALL,
         Vec3(FIELD_HALF_WIDTH, point.y, point.z)),
        (abs(point.x + FIELD_HALF_WIDTH), Vec3(1, 0, 0), Surface.SIDE_WALL,
         Vec3(-FIELD_HALF_WIDTH, point.y, point.z)),
        (abs(FIELD_HALF_LENGTH - point.y), Vec3(0, -1, 0), Surface.BACK_WALL,
         Vec3(point.x, FIELD_HALF_LENGTH, point.z)),
        (abs(point.y + FIELD_HALF_LENGTH), Vec3(0, 1, 0), Surface.BACK_WALL,
         Vec3(point.x, -FIELD_HALF_LENGTH, point.z)),
    )
    distance, normal, surface, location = min(distances, key=lambda item: item[0])
    return SurfaceInfo(surface, location, normal, distance)


def is_near_wall(point: Vec3, distance: float = 250.0) -> bool:
    """Check proximity to a side or back wall."""
    return (
        abs(point.x) >= FIELD_HALF_WIDTH - distance
        or abs(point.y) >= FIELD_HALF_LENGTH - distance
    )


def is_near_ceiling(point: Vec3, distance: float = 250.0) -> bool:
    return point.z >= CEILING_HEIGHT - distance


def is_near_backboard(point: Vec3, distance: float = 350.0) -> bool:
    """Check whether a point is near either end backboard."""
    return abs(point.y) >= FIELD_HALF_LENGTH - distance


def is_near_side_wall(point: Vec3, distance: float = 250.0) -> bool:
    return abs(point.x) >= FIELD_HALF_WIDTH - distance


def corner_for_position(point: Vec3) -> tuple[int, int]:
    """Return the signs of the nearest corner: (side_x, side_y)."""
    return side_sign(point.x), side_sign(point.y)


def corner_waypoint(
    point: Vec3,
    inset: float = 300.0,
    height: float = 0.0,
) -> Vec3:
    """Return a safe approximate waypoint inside the nearest corner."""
    sx, sy = corner_for_position(point)
    return Vec3(
        sx * (FIELD_HALF_WIDTH - inset),
        sy * (FIELD_HALF_LENGTH - inset),
        height,
    )


def backboard_target(team: int, x: float = 0.0, z: float = 500.0) -> Vec3:
    """Return a point on the opponent-facing backboard for the given team.

    team is the team whose attacking direction is being used.
    """
    goal = enemy_goal_for_team(team)
    return Vec3(
        clamp(x, -GOAL_HALF_WIDTH, GOAL_HALF_WIDTH),
        goal.center.y - goal.forward.y * 120.0,
        z,
    )


def goal_post_points(team: int) -> tuple[Vec3, Vec3]:
    goal = goal_for_team(team)
    return goal.left_post, goal.right_post


def boost_pad_locations(large_only: bool = False) -> tuple[Vec3, ...]:
    """Return known boost pad locations."""
    if large_only:
        return LARGE_BOOST_PADS
    return SMALL_BOOST_PADS


def closest_boost_pad(
    point: Vec3,
    large_only: bool = False,
) -> Vec3 | None:
    pads = boost_pad_locations(large_only)
    if not pads:
        return None
    return min(pads, key=point.distance)


def points_are_on_same_half(a: Vec3, b: Vec3) -> bool:
    """Whether two points lie on the same side of midfield."""
    return (a.y <= 0 and b.y <= 0) or (a.y >= 0 and b.y >= 0)
