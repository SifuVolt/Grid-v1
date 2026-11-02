
"""Grid's physics, prediction, interception and tactical evaluation library."""
from __future__ import annotations
from dataclasses import dataclass
from math import atan2, cos, sin, sqrt
from .utils import *
import math
from physics.aerial import find_best_aerial_option

@dataclass(slots=True)
class PhysicsState:
    location: Vec3
    velocity: Vec3
    rotation: object = None
    angular_velocity: Vec3 = Vec3()

    @classmethod
    def from_physics(cls, physics):
        return cls(Vec3.from_obj(getattr(physics, "location", None)),
                   Vec3.from_obj(getattr(physics, "velocity", None)),
                   getattr(physics, "rotation", None),
                   Vec3.from_obj(getattr(physics, "angular_velocity", None)))

@dataclass(slots=True)
class CarState:
    index: int
    team: int
    physics: PhysicsState
    boost: float
    on_ground: bool
    demolished: bool
    jumped: bool
    double_jumped: bool
    name: str = ""

    @property
    def position(self): return self.physics.location
    @property
    def velocity(self): return self.physics.velocity
    @property
    def speed(self): return self.velocity.magnitude()
    @property
    def flat_speed(self): return self.velocity.flat_magnitude()
    @property
    def yaw(self):
        return float(getattr(self.physics.rotation, "yaw", 0.0) or 0.0)
    @property
    def pitch(self):
        return float(getattr(self.physics.rotation, "pitch", 0.0) or 0.0)
    @property
    def roll(self):
        return float(getattr(self.physics.rotation, "roll", 0.0) or 0.0)
    @property
    def forward(self): return forward_vector(self.yaw)

    
    @classmethod
    def from_player(cls, player, index):
        air_state = getattr(player, "air_state", None)
        on_ground = (
            str(air_state).lower().endswith("onground")
            or air_state == 0
        )

        demolished = (
            float(getattr(player, "demolished_timeout", 0.0)) > 0.0
        )

        return cls(
            index=index,
            team=int(getattr(player, "team", 0)),
            physics=PhysicsState.from_physics(player.physics),
            boost=float(getattr(player, "boost", 0.0)),
            on_ground=on_ground,
            demolished=demolished,
            jumped=bool(getattr(player, "has_jumped", False)),
            double_jumped=bool(getattr(player, "has_double_jumped", False)),
            name=str(getattr(player, "name", "")),
        )


@dataclass(slots=True)
class BallState:
    position: Vec3
    velocity: Vec3
    angular_velocity: Vec3
    latest_touch: object = None

    @classmethod
    def from_ball(cls, ball):
        p = ball.physics
        return cls(Vec3.from_obj(p.location), Vec3.from_obj(p.velocity),
                   Vec3.from_obj(getattr(p, "angular_velocity", None)),
                   getattr(ball, "latest_touch", None))

@dataclass(slots=True)
class Intercept:
    time: float
    position: Vec3
    velocity: Vec3
    car_target: Vec3
    mode: str
    score: float
    reachable: bool
    reason: str = ""
    arrival_time: float = 0.0

@dataclass(slots=True)
class PredictionPoint:
    time: float
    position: Vec3
    velocity: Vec3

def _field(obj, *names, default=None):
    for name in names:
        if hasattr(obj, name):
            return getattr(obj, name)
    return default

def prediction_points(prediction, now: float) -> list[PredictionPoint]:
    """Convert an RLBot prediction buffer into safe, time-stamped points."""
    if prediction is None:
        return []
    slices = _field(prediction, "slices", default=[])
    try:
        count = len(slices)
    except TypeError:
        count = 0
    out = []
    for i in range(count):
        sl = slices[i]
        game_seconds = float(_field(sl, "game_seconds", "gameSeconds", default=now))
        physics = _field(sl, "physics", default=None)
        if physics is None:
            continue
        pos = Vec3.from_obj(_field(physics, "location", default=None))
        vel = Vec3.from_obj(_field(physics, "velocity", default=None))
        t = game_seconds-now
        if t >= -0.02:
            out.append(PredictionPoint(max(0.0, t), pos, vel))
    return out

def sample_prediction(points: list[PredictionPoint], time: float):
    if not points:
        return None
    if time <= points[0].time:
        return points[0]
    for a, b in zip(points, points[1:]):
        if a.time <= time <= b.time:
            span = max(1e-6, b.time-a.time)
            f = (time-a.time)/span
            return PredictionPoint(time, a.position+(b.position-a.position)*f,
                                   a.velocity+(b.velocity-a.velocity)*f)
    return points[-1] if time >= points[-1].time else None


def _approach_turn_time(car, target):
    dx = target.x - car.position.x
    dy = target.y - car.position.y

    distance = math.sqrt(dx * dx + dy * dy)

    if distance < 1e-6:
        return 0.0, 1.0

    direction_x = dx / distance
    direction_y = dy / distance

    forward = car.forward
    forward_length = math.sqrt(
        forward.x * forward.x + forward.y * forward.y
    )

    if forward_length < 1e-6:
        return 0.0, 0.0

    forward_x = forward.x / forward_length
    forward_y = forward.y / forward_length

    alignment = clamp(
        forward_x * direction_x + forward_y * direction_y,
        -1.0,
        1.0
    )

    cross_z = forward_x * direction_y - forward_y * direction_x
    angle = abs(math.atan2(cross_z, alignment))

    speed = car.flat_speed
    turn_rate = 2.8 - 1.45 * clamp(speed / 2300.0, 0.0, 1.0)
    turn_time = angle / max(0.9, turn_rate)

    return turn_time, alignment


def estimate_ground_time(
    distance: float,
    speed: float,
    boost: float = 0.0,
    acceleration: float = 1050.0,
    max_speed: float = 2300.0,
) -> float:
    """
    Estimate travel time using speed in the direction of travel.

    Negative projected speed is treated as zero: the car must turn
    around rather than being rewarded for travelling backwards.
    """
    distance = max(0.0, distance - 115.0)
    speed = clamp(speed, 0.0, max_speed)

    if distance <= 0.0:
        return 0.0

    acceleration = acceleration + (250.0 if boost > 10.0 else 0.0)

    time_to_cap = max(0.0, (max_speed - speed) / acceleration)
    distance_to_cap = (
        speed * time_to_cap
        + 0.5 * acceleration * time_to_cap * time_to_cap
    )

    if distance <= distance_to_cap:
        return (
            -speed + sqrt(speed * speed + 2.0 * acceleration * distance)
        ) / acceleration

    return time_to_cap + (
        distance - distance_to_cap
    ) / max_speed


TURN_RADIUS_POINTS = (
    (0.0, 145.0),
    (500.0, 251.0),
    (1000.0, 425.0),
    (1500.0, 727.0),
    (1750.0, 909.0),
    (2300.0, 1136.0),
)


def _turn_radius(speed: float) -> float:
    speed = clamp(abs(speed), 0.0, 2300.0)

    for i in range(len(TURN_RADIUS_POINTS) - 1):
        s0, r0 = TURN_RADIUS_POINTS[i]
        s1, r1 = TURN_RADIUS_POINTS[i + 1]

        if speed <= s1:
            f = (speed - s0) / (s1 - s0)
            return r0 + (r1 - r0) * f

    return 1136.0


def _speed_after_turn(speed: float, angle: float) -> float:
    """Party Cannon's approximate speed loss during a turn."""

    speed = clamp(abs(speed), 0.0, 2300.0)
    angle = abs(angle)

    factor = math.exp(angle * 0.49) * angle * 0.49

    if speed > 1234.0:
        factor *= 0.2

    result = (1234.0 * factor + speed) / (factor + 1.0)

    return clamp(result, 0.0, 2300.0)



def car_arrival_time(
    car: CarState,
    target: Vec3,
) -> tuple[float, str]:
    """Party Cannon-style ground arrival estimate."""

    dx = target.x - car.position.x
    dy = target.y - car.position.y
    distance = sqrt(dx * dx + dy * dy)

    if distance < 100.0:
        return 0.0, "ground"

    # Directional speed: don't reward sideways or backwards velocity.
    forward = car.forward
    forward_speed = max(
        0.0,
        car.velocity.x * forward.x
        + car.velocity.y * forward.y,
    )

    direction_length = max(1.0, distance)
    direction_x = dx / direction_length
    direction_y = dy / direction_length

    alignment = clamp(
        forward.x * direction_x + forward.y * direction_y,
        -1.0,
        1.0,
    )

    angle = math.acos(alignment)

    radius = _turn_radius(max(forward_speed, 500.0))

    turn_time = angle * radius / 1800.0

    if angle < 0.5:
        turn_time = 0.0

    # Keep the turn estimate separate from the driving distance.
    remaining = max(0.0, distance - 200.0)

    speed = forward_speed
    boost_remaining = max(0.0, car.boost)
    elapsed = turn_time

    dt = 1.0 / 60.0

    # Simulate throttle and boost acceleration.
    for _ in range(600):
        if remaining <= 0.0:
            break

        throttle_accel = 1600.0 if speed < 1410.0 else 0.0

        boost_accel = (
            991.0
            if boost_remaining > 0.0 and speed < 2300.0
            else 0.0
        )

        acceleration = throttle_accel + boost_accel

        next_speed = min(
            2300.0,
            speed + acceleration * dt,
        )

        average_speed = (speed + next_speed) * 0.5
        travelled = average_speed * dt

        remaining -= travelled
        elapsed += dt
        speed = next_speed

        if boost_accel > 0.0:
            boost_remaining = max(
                0.0,
                boost_remaining - 33.333 * dt,
            )

        # Avoid an unrealistic zero-speed estimate.
        if speed < 800.0 and throttle_accel == 0.0:
            speed = 800.0

    return elapsed * 1.05, "ground"



def score_intercept(
    car: CarState,
    ball_point: PredictionPoint,
    team: int,
    opponents: list[CarState],
    teammates: list[CarState],
) -> Intercept:

    target = shot_target(team, ball_point.position)
    contact = make_contact_target(ball_point.position, target)
    time_available = max(0.02, ball_point.time)

    turn_time, alignment = _approach_turn_time(car, contact)

    if ball_point.position.z > 430.0:
        option = find_best_aerial_option(
            position=car.position,
            velocity=car.velocity,
            forward=car.forward,
            target=contact,
            time_available=time_available,
            boost=car.boost,
        )

        if option is None:
            return Intercept(
                time=time_available,
                position=ball_point.position,
                velocity=ball_point.velocity,
                car_target=contact,
                mode="aerial",
                score=10.0 + time_available,
                reachable=False,
                reason="aerial_unreachable",
                arrival_time=time_available + 1.0,
            )

        arrival = max(0.0, time_available - 0.05)
        mode = "aerial"

        mechanic_penalty = (
            option.boost_needed * 0.008
            + option.turn_time * 0.25
            + (1.0 - option.alignment) * 0.25
        )

    else:
        arrival, mode = car_arrival_time(car, contact)
        mechanic_penalty = 0.0

    reachable = arrival <= time_available + 0.05

    if not reachable:
        return Intercept(
            time=time_available,
            position=ball_point.position,
            velocity=ball_point.velocity,
            car_target=contact,
            mode=mode,
            score=10.0 + arrival - time_available,
            reachable=False,
            reason="late",
            arrival_time=arrival,
        )

    opponent_eta = min(
        (
            car_arrival_time(opponent, ball_point.position)[0]
            for opponent in opponents
            if not opponent.demolished
        ),
        default=9.0,
    )

    teammate_eta = min(
        (
            car_arrival_time(teammate, ball_point.position)[0]
            for teammate in teammates
            if not teammate.demolished
            and teammate.index != car.index
        ),
        default=9.0,
    )

    contest_penalty = max(
        0.0,
        arrival - opponent_eta + 0.08,
    ) * 0.25

    teammate_penalty = (
        0.12 if teammate_eta + 0.10 < arrival else 0.0
    )

    # Earliest feasible contact is the main priority.
    score = (
        time_available
        + mechanic_penalty
        + turn_time * 0.08
        + (1.0 - alignment) * 0.08
        + contest_penalty
        + teammate_penalty
    )

    return Intercept(
        time=time_available,
        position=ball_point.position,
        velocity=ball_point.velocity,
        car_target=contact,
        mode=mode,
        score=score,
        reachable=True,
        reason="aerial_feasible" if mode == "aerial" else "reachable",
        arrival_time=arrival,
    )


def find_best_intercept(
    car: CarState,
    points: list[PredictionPoint],
    team: int,
    opponents: list[CarState],
    teammates: list[CarState],
    horizon: float = 4.0,
) -> Intercept | None:

    fallback_options = []

    for point in points:
        if point.time > horizon:
            break

        if point.time < 0.08:
            continue

        if point.position.z > 1900.0:
            continue

        candidate = score_intercept(
            car,
            point,
            team,
            opponents,
            teammates,
        )

        fallback_options.append(candidate)

        # First reachable point in chronological order.
        if candidate.reachable:
            return candidate

    if not fallback_options:
        return None

    # If nothing is reachable, choose the least-late candidate.
    return min(
        fallback_options,
        key=lambda item: (
            max(0.0, item.arrival_time - item.time),
            item.time,
        ),
    )


def estimate_aerial_time(car: CarState, target: Vec3, boost_efficiency: float = 1.0) -> float:
    """Approximate aerial reach time; used for ranking, not as a guarantee."""
    delta = target-car.position
    horizontal = delta.flat_magnitude()
    vertical = max(0.0, delta.z)
    initial = 291.7
    gravity = 650.0
    jump_time = clamp(0.18 + vertical/1500.0, 0.18, 0.42)
    jump_height = initial*jump_time + 0.5*1458.0*jump_time*jump_time
    remaining_z = max(0.0, vertical-jump_height)
    boost_accel = 991.0 * (0.55 if car.boost < 10 else boost_efficiency)
    vertical_time = sqrt(2*remaining_z/max(300.0, gravity+boost_accel)) if remaining_z else 0.0
    horizontal_time = horizontal/max(900.0, 1200.0 + min(car.boost, 50)*8.0)
    return max(jump_time, vertical_time, horizontal_time) + 0.12


def shot_target(team: int, ball_position: Vec3, target_y: float | None = None) -> Vec3:
    y = enemy_goal_y(team) if target_y is None else target_y
    # Select a post-biased point away from the nearest defender.
    x = clamp(-ball_position.x*0.18, -650.0, 650.0)
    return Vec3(x, y, 250.0)

def make_contact_target(ball_position: Vec3, target: Vec3, radius: float = 145.0) -> Vec3:
    direction = (target-ball_position).normalized()
    return ball_position-direction*radius


def ball_threat_level(ball: BallState, team: int) -> float:
    attack = to_attack_coordinates(ball.position, team)
    vel = to_attack_coordinates(ball.velocity, team)
    depth = clamp((-attack.y+700.0)/4200.0, 0.0, 1.0)
    toward_goal = clamp((-vel.y)/1800.0, 0.0, 1.0)
    low = 1.0-clamp((ball.position.z-100.0)/900.0, 0.0, 1.0)
    return clamp(depth*0.58+toward_goal*0.32+low*0.10, 0.0, 1.0)

def choose_role(car: CarState, teammates: list[CarState], ball: BallState,
                team: int, player_count: int) -> str:
    active = [p for p in [car, *teammates] if not p.demolished]
    if player_count <= 1 or not teammates:
        return "solo"
    # Closest player pressures; remaining players distribute by depth.
    ranked = sorted(
        active,
        key=lambda p: (
            p.position.flat_distance(ball.position),
            p.index
        )
    )
    if ranked and ranked[0].index == car.index:
        return "first"
    if player_count >= 3:
        own_depth = to_attack_coordinates(car.position, team).y
        deepest = min(active, key=lambda p: to_attack_coordinates(p.position, team).y)
        if deepest.index == car.index:
            return "last"
    return "second"

def choose_boost_target(car: CarState, pads, ball: BallState, team: int,
                        role: str) -> Vec3 | None:
    if car.boost >= (22 if role in ("first", "solo") else 38):
        return None
    best, best_score = None, float("inf")
    for pad in pads:
        loc = Vec3.from_obj(_field(pad, "location", default=None))
        active = bool(_field(pad, "is_active", "active", default=True))
        if not active:
            continue
        large = bool(_field(pad, "is_full_boost", "full_boost", default=False))
        d = car.position.flat_distance(loc)
        ball_d = loc.flat_distance(ball.position)
        if is_in_own_half(ball.position, team) and role != "first":
            # Don't abandon a dangerous own-half ball for a remote pad.
            if d > 900: continue
        score = d - (260 if large and car.boost < 55 else 0) + max(0, 1000-ball_d)*0.18
        if score < best_score:
            best, best_score = loc, score
    return best

def choose_tactical_target(car: CarState, ball: BallState, team: int,
                           role: str, intercept: Intercept | None) -> tuple[Vec3, str]:
    threat = ball_threat_level(ball, team)
    if role == "last" or (is_in_own_half(ball.position, team) and threat > 0.52):
        # Goal-side shadow position, offset toward the ball's lateral lane.
        goal = Vec3(0, own_goal_y(team)*0.82, 0)
        lane = ball.position-goal
        shadow = ball.position-lane.flat_normalized()*1050
        shadow = Vec3(clamp(shadow.x, -3300, 3300),
                      clamp(shadow.y, -4400, 4400), 0)
        return shadow, "shadow"
    if role == "second":
        attack_y = enemy_goal_y(team)
        support = ball.position + Vec3(-sign(ball.position.x)*850,
                                       (attack_y-ball.position.y)*0.14, 0)
        return support, "support"
    if intercept is not None:
        return intercept.car_target, "intercept"
    return ball.position, "challenge"

def tactical_priority(ball: BallState, team: int, car: CarState,
                      role: str, intercept: Intercept | None) -> str:
    threat = ball_threat_level(ball, team)
    if threat > 0.78 and car.position.flat_distance(ball.position) < 2700:
        return "emergency_save"
    if role == "last" and threat > 0.40:
        return "shadow"
    if intercept and intercept.reachable and intercept.time < 1.15:
        return "challenge"
    if car.boost < 18 and role in ("second", "last"):
        return "support_or_boost"
    return "rotate"

def estimate_ball_landing(ball: BallState, gravity: float = -650.0) -> tuple[float, Vec3]:
    z, vz = ball.position.z, ball.velocity.z
    disc = vz*vz-2*gravity*(z-BALL_RADIUS)
    if disc < 0:
        return 0.0, ball.position
    t = (-vz-sqrt(disc))/gravity if gravity else 0.0
    t = max(0.0, t)
    landing = ball.position + ball.velocity*t + Vec3(0, 0, 0.5*gravity*t*t)
    return t, landing.with_z(BALL_RADIUS)

def get_rotation_basis(rotation) -> tuple[Vec3, Vec3, Vec3]:
    """Return forward, right and up vectors from RL's Euler car rotation."""
    pitch = float(getattr(rotation, "pitch", 0.0))
    yaw = float(getattr(rotation, "yaw", 0.0))
    roll = float(getattr(rotation, "roll", 0.0))
    cp, sp, cy, sy, cr, sr = cos(pitch), sin(pitch), cos(yaw), sin(yaw), cos(roll), sin(roll)
    forward = Vec3(cp*cy, cp*sy, sp)
    right = Vec3(cy*sp*sr-cr*sy, sy*sp*sr+cr*cy, -cp*sr)
    up = Vec3(-cr*cy*sp-sr*sy, -cr*sy*sp+sr*cy, cp*cr)
    return forward, right, up
