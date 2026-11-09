"""Grid aerial launch physics and reachability foundation for Volt.

The double-jump model is the launch phase, not the whole aerial. Aerial
control after launch remains with Volt's Aerial routine, which steers and
boosts continuously toward the predicted hitbox contact point.
"""
from math import sqrt


def solve_quadratic(a, b, c, answer, branch):
    root = (answer - c) / a + b * b / (4 * a * a)
    if root < 0:
        return None
    return -b / (2 * a) + sqrt(root) * (1 if branch >= 0 else -1)


def new_max_jump_height(double=False):
    extra = 292 * bool(double)
    x = -(564.9931640625 + extra) / (-325.0556640625 * 2)
    return (
        -325.0556640625 * x * x
        + (564.9931640625 + extra) * x
        - 11.6591796875
        - 292 * 25 / 120 * bool(double)
    )


def new_jump_time(height, double=False):
    single_time = solve_quadratic(
        -325.0556640625, 564.9931640625, -11.6591796875,
        min(height, new_max_jump_height(False)), -1
    )
    if single_time is None:
        return None, 1
    if single_time >= 24 / 120:
        if single_time >= 25 / 120 and double:
            double_time = solve_quadratic(
                -325.0556640625, 856.9931640625,
                -11.6591796875 - 292 * 25 / 120,
                min(height, new_max_jump_height(True)), -1
            )
            return double_time, 2
        return single_time, 1
    short_time = solve_quadratic(392.2802734375, 272.7626953125, 17.671875, height, 1)
    return short_time, 1


def acceleration_table(speed, boost):
    distances = [0.0]
    velocities = [max(0.0, speed)]
    for i in range(600):
        v = velocities[-1]
        use_boost = (i / 120) < boost
        if v <= 1400:
            next_v = v + (1600 - 1440 / 1400 * v + (2975 / 3 if use_boost else 0)) / 120
        elif v < 1410:
            next_v = v + (22560 - 16 * v + (2975 / 3 if use_boost else 0)) / 120
        else:
            next_v = min(v + (2975 / 360 if use_boost else 0), 2300)
        velocities.append(next_v)
        distances.append(distances[-1] + next_v / 120)
    return distances, velocities


def _xyz(value):
    return float(value.x), float(value.y), float(value.z)


def _flat_distance(a, b):
    return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5


def _subtract(a, b):
    return a[0] - b[0], a[1] - b[1], a[2] - b[2]


def _normalize(a):
    length = sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
    if length < 1e-6:
        return 0.0, 0.0, 0.0
    return a[0] / length, a[1] / length, a[2] / length


def find_double_jump(agent, targets=None, horizon=5.0):
    """Return Grid's earliest reachable grounded double-jump candidate."""
    prediction = getattr(agent, "ball_prediction_struct", None)
    slices = getattr(prediction, "slices", None) if prediction else None
    if not slices or agent.me.airborne:
        return None

    car = _xyz(agent.me.location)
    velocity = _xyz(agent.me.velocity)
    flat_speed = sqrt(velocity[0] ** 2 + velocity[1] ** 2)
    travel_d, travel_v = acceleration_table(flat_speed, max(0.0, float(agent.me.boost)))

    if targets is not None:
        left, right = _xyz(targets[0]), _xyz(targets[1])
        target_center = tuple((left[i] + right[i]) * 0.5 for i in range(3))
    else:
        left = right = target_center = _xyz(agent.foe_goal.location)

    best = None
    start_time = agent.time
    for ball_slice in slices:
        t = ball_slice.game_seconds - start_time
        if t < 0.05:
            continue
        if t > horizon:
            break
        if abs(round(t * 60) / 60 - t) > 0.009:
            continue

        bp = ball_slice.physics
        ball = _xyz(bp.location)
        if abs(ball[1]) > 5212.75:
            break
        direction = _normalize(_subtract(target_center, ball))
        contact = (
            ball[0] - direction[0] * 92.75,
            ball[1] - direction[1] * 92.75,
            ball[2] - direction[2] * 92.75
        )
        distance = _flat_distance(contact, car)
        jump_time, jump_amount = new_jump_time(max(contact[2] - car[2], 0.0), True)
        if jump_time is None or contact[2] > car[2] + new_max_jump_height(True) + 60:
            continue

        charging_ticks = max(0, min(round(t * 120), 600))
        driving_ticks = max(0, min(round(min(max(t - jump_time, 0.0), 5.0) * 120), 600))
        reachable = (
            travel_d[driving_ticks]
            + travel_v[driving_ticks] * (charging_ticks - driving_ticks) / 120
        )
        if reachable < distance:
            continue

        candidate = {
            "found": 1, "shot_type": 2, "fast": False,
            "targets": (tuple(left), tuple(right)),
            "time": ball_slice.game_seconds, "ball_location": ball,
            "contact_location": contact, "jump_time": jump_time,
            "jump_amount": jump_amount, "distance": distance,
            "reachable": reachable
        }
        if best is None or candidate["time"] < best["time"]:
            best = candidate
    return best


def find_aerial_launch(agent, targets=None, horizon=6.0):
    """Grid-style aerial candidate: double-jump launch plus boost-assisted flight.

    Grid's jump model decides whether the car can establish vertical lift.
    Boost is then budgeted over the remaining flight, rather than treating
    the double-jump height ceiling as the aerial's final height ceiling.
    The Aerial routine owns the subsequent continuous steering/boost control.
    """
    prediction = getattr(agent, "ball_prediction_struct", None)
    slices = getattr(prediction, "slices", None) if prediction else None
    if not slices:
        return None

    car = _xyz(agent.me.location)
    velocity = _xyz(agent.me.velocity)
    boost = max(0.0, float(agent.me.boost))
    up = _xyz(agent.me.up)
    best = None

    if targets is not None:
        left, right = _xyz(targets[0]), _xyz(targets[1])
        target_center = tuple((left[i] + right[i]) * 0.5 for i in range(3))
    else:
        left = right = target_center = _xyz(agent.foe_goal.location)

    for ball_slice in slices:
        t = ball_slice.game_seconds - agent.time
        if t < 0.12:
            continue
        if t > horizon:
            break
        if abs(round(t * 60) / 60 - t) > 0.009:
            continue

        bp = ball_slice.physics
        ball = _xyz(bp.location)
        if abs(ball[1]) > 5212.75:
            break
        direction = _normalize(_subtract(target_center, ball))
        contact = (
            ball[0] - direction[0] * 92.75,
            ball[1] - direction[1] * 92.75,
            ball[2] - direction[2] * 92.75
        )
        vertical_gap = contact[2] - car[2]
        jump_time, jump_amount = new_jump_time(max(vertical_gap, 0.0), True)

        # Aerial may begin from an already airborne car. Grounded launches
        # need enough time for the Grid double-jump sequence; airborne cars
        # retain their current vertical velocity and can boost immediately.
        if agent.me.airborne:
            launch_time = 0.0
            launch_vz = velocity[2]
            launch_height = car[2]
        else:
            if jump_time is None:
                continue
            launch_time = min(jump_time + 25 / 120, t)
            launch_vz = velocity[2] + 2 * 291.6667
            launch_height = car[2] + new_max_jump_height(True)

        flight_time = max(0.0, t - launch_time)
        boost_time = min(flight_time, boost / (100 / 3))
        # Rocket League boost acceleration is approximately 991 uu/s².
        # Use a conservative projection along current up for candidate
        # screening; the Aerial controller refines direction every frame.
        boost_rise = 0.5 * 991.0 * boost_time * boost_time
        vertical_reach = (
            launch_height
            + launch_vz * flight_time
            + 0.5 * (-650.0) * flight_time * flight_time
            + boost_rise
        )
        if not agent.me.airborne:
            vertical_reach = max(
                vertical_reach,
                car[2] + new_max_jump_height(True)
                + max(0.0, launch_vz) * flight_time
                + boost_rise
                - 325.0 * flight_time * flight_time
            )

        if contact[2] > vertical_reach + 110:
            continue

        # Horizontal travel is estimated from current speed and available
        # boost; avoid rejecting useful high contacts just because jump
        # height alone cannot reach them.
        flat_distance = _flat_distance(contact, car)
        flat_speed = sqrt(velocity[0] ** 2 + velocity[1] ** 2)
        boost_distance = 0.5 * 991.0 * boost_time * boost_time
        horizontal_reach = flat_speed * t + 0.5 * 500.0 * t * t + boost_distance + 180
        if flat_distance > horizontal_reach:
            continue

        candidate = {
            "found": 1, "shot_type": 3, "fast": True,
            "targets": (tuple(left), tuple(right)),
            "time": ball_slice.game_seconds, "ball_location": ball,
            "contact_location": contact, "jump_time": jump_time or 0.0,
            "jump_amount": 2 if not agent.me.airborne else 0,
            "reachable": horizontal_reach,
            "vertical_reach": vertical_reach
        }
        if best is None or candidate["time"] < best["time"]:
            best = candidate
    return best
