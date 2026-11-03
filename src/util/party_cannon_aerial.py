"""Party Cannon AerialShot port for the Noob Black RLBot v5 bot.

Source: NicEastvillage/PartyCannon
Commit: ea84e4325ddb60565bbac6fb9766dc45f38b58cb
Original: RedUtils/Actions/AerialShot.cs

The shot planning, target-offset calculation, boost estimate, jump sequence,
airborne acceleration control, and abort conditions follow the source action.
Noob Black's existing ground approach and orientation controller are supplied
by the caller because Party Cannon's Drive/Car wrapper types are not present
in this bot.
"""
import math

from util.vec import Vec3


MAX_SPEED = 2300.0
MAX_JUMP_DURATION = 0.2
JUMP_ACCEL = 1458.333
JUMP_VEL = 291.667
AIR_THROTTLE_ACCEL = 66.667
BOOST_ACCEL = 991.667
BOOST_CONSUMPTION = 33.3
STICKY_ACCEL = 325.0
BALL_MAX_SPEED = 6000.0
GRAVITY = Vec3(0, 0, -650)


def _cap(value, low, high):
    return max(low, min(high, value))


def _unit(vector):
    length = vector.length()
    return vector / length if length > 1e-6 else Vec3(0, 0, 0)


def _cap_vector(vector, maximum):
    length = vector.length()
    return vector if length <= maximum or length <= 1e-6 else vector * (maximum / length)


def _angle(a, b):
    la, lb = a.length(), b.length()
    if la <= 1e-6 or lb <= 1e-6:
        return 0.0
    return math.acos(_cap(a.dot(b) / (la * lb), -1.0, 1.0))


def _shot_power_modifier(value):
    value = _cap(value, 0, 4600)
    if value <= 500:
        return 0.65
    if value <= MAX_SPEED:
        t = (value - 500) / 1800
        return 0.65 + (0.55 - 0.65) * t
    t = (value - MAX_SPEED) / 4600
    return 0.55 + (0.3 - 0.55) * t


def _grounded(car):
    return str(car.air_state) == "AirState.OnGround"


def _jump_times(elapsed):
    return (
        _cap(MAX_JUMP_DURATION - elapsed, 0, MAX_JUMP_DURATION),
        _cap(0.05 - elapsed, 0, 0.05),
    )


class AerialShot:
    """Party Cannon AerialShot state machine, adapted to Noob Black's packet API."""

    def __init__(self, car, ball_slice, slice_time, shot_target, up):
        self.slice_time = slice_time
        self.slice_location = Vec3(ball_slice.physics.location)
        self.slice_velocity = Vec3(ball_slice.physics.velocity)
        self.shot_target = Vec3(shot_target)
        self.start_boost = float(car.boost)
        self.up = Vec3(up)

        self.finished = False
        self.interruptible = True
        self.double_jumping = False
        self.currently_double_jumping = False
        self.aerialing = False
        self.jumped = False
        self.elapsed_time = 0.0
        self.step = 0
        self.dodging = False
        self.dodge_elapsed = 0.0
        self.dodge_step = 0
        self.dodge_input = None
        self.dodge_jump = True

        self.shot_direction = Vec3(0, 0, 1)
        self.target_location = Vec3(self.slice_location)
        self._set_target_location(car)

        velocity = Vec3(car.physics.velocity)
        to_target = self.target_location - Vec3(car.physics.location)
        flat_velocity = velocity - self.up * velocity.dot(self.up)
        flat_target = to_target - self.up * to_target.dot(self.up)
        self.jump_immediately = (
            _angle(flat_velocity, flat_target) < 0.6
            or flat_velocity.length() < 500
            or not _grounded(car)
        )

        single = self._get_boost_estimate(car, False, 0.0)
        double = self._get_boost_estimate(car, True, 0.0)
        self.can_hit = single >= 0 or double >= 0
        self.double_jumping = double >= 0 and (double < single or single < 0)

    def _set_target_location(self, car):
        time_remaining = max(self.slice_time - 0.0, 0.001)
        car_location = Vec3(car.physics.location)
        car_velocity = Vec3(car.physics.velocity)

        car_final_velocity = _cap_vector(
            (self.slice_location - car_location) / time_remaining,
            MAX_SPEED,
        )

        # Predict the ball's velocity immediately after contact, then its
        # travel time toward the selected shot target.
        ball_after_velocity = (car_final_velocity * 6 + self.slice_velocity) / 7
        direction_to_score = _unit(
            Vec3(
                self.shot_target.x - self.slice_location.x,
                self.shot_target.y - self.slice_location.y,
                0,
            )
        )
        velocity_difference = (car_final_velocity - self.slice_velocity).length()
        scoring_speed = _cap(
            velocity_difference * _shot_power_modifier(velocity_difference)
            + ball_after_velocity.dot(direction_to_score),
            500,
            BALL_MAX_SPEED,
        )
        distance_to_score = math.sqrt(
            (self.shot_target.x - self.slice_location.x) ** 2
            + (self.shot_target.y - self.slice_location.y) ** 2
        )
        time_to_score = distance_to_score / scoring_speed

        ball_after_velocity = (car_final_velocity + self.slice_velocity * 2) / 3
        predicted_ball = (
            self.slice_location
            + ball_after_velocity * time_to_score
            + GRAVITY * (0.5 * time_to_score * time_to_score)
        )
        self.shot_direction = _unit(self.shot_target - predicted_ball)
        self.target_location = self.slice_location - self.shot_direction * 155

        # Party Cannon flattens the contact direction when the contact point
        # would be less than 50 uu from the nearest arena surface.
        surface_point, normal = self._nearest_surface(self.target_location)
        distance_from_surface = (self.target_location - surface_point).dot(normal)
        if distance_from_surface < 50:
            angle = math.asin(_cap((distance_from_surface - 50) / 155, -1, 1))
            flat_direction = self.shot_direction - normal * self.shot_direction.dot(normal)
            self.shot_direction = _unit(
                _unit(flat_direction) * math.cos(angle) + normal * math.sin(angle)
            )
            self.target_location = self.slice_location - self.shot_direction * 155

    @staticmethod
    def _nearest_surface(location):
        candidates = [
            (abs(location.z), Vec3(location.x, location.y, 0), Vec3(0, 0, 1)),
            (abs(2044 - location.z), Vec3(location.x, location.y, 2044), Vec3(0, 0, -1)),
            (abs(4096 - abs(location.x)), Vec3(math.copysign(4096, location.x), location.y, location.z), Vec3(-math.copysign(1, location.x), 0, 0)),
            (abs(5120 - abs(location.y)), Vec3(location.x, math.copysign(5120, location.y), location.z), Vec3(0, -math.copysign(1, location.y), 0)),
        ]
        _, point, normal = min(candidates, key=lambda item: item[0])
        return point, normal

    def _jump_location(self, car, time, elapsed, double_jump):
        jump_remaining, stick_remaining = _jump_times(elapsed)
        location = Vec3(car.physics.location)
        velocity = Vec3(car.physics.velocity)
        grounded = _grounded(car)
        result = (
            location
            + velocity * time
            + GRAVITY * (0.5 * time * time)
            + (self.up * (JUMP_VEL * time) if grounded else Vec3(0, 0, 0))
            + self.up * JUMP_ACCEL * jump_remaining * (time - 0.5 * jump_remaining)
            - self.up * STICKY_ACCEL * stick_remaining * (time - 0.5 * stick_remaining)
        )
        if double_jump and not (car.has_double_jumped or car.has_dodged):
            result = result + self.up * JUMP_VEL * (time - jump_remaining)
        return result

    def _jump_velocity(self, car, time, elapsed, double_jump):
        jump_remaining, stick_remaining = _jump_times(elapsed)
        velocity = Vec3(car.physics.velocity)
        grounded = _grounded(car)
        result = (
            velocity
            + GRAVITY * time
            + self.up * (JUMP_VEL if grounded else 0)
            + self.up * JUMP_ACCEL * jump_remaining
            - self.up * STICKY_ACCEL * stick_remaining
        )
        if double_jump and not (car.has_double_jumped or car.has_dodged):
            result = result + self.up * JUMP_VEL
        return result

    def _get_boost_estimate(self, car, double_jump, elapsed, time_remaining=None):
        time_remaining = max(self.slice_time if time_remaining is None else time_remaining, 0.001)
        if _grounded(car):
            final_position = self._jump_location(car, time_remaining, elapsed, double_jump)
            final_velocity = self._jump_velocity(car, time_remaining, elapsed, double_jump)
        else:
            location = Vec3(car.physics.location)
            velocity = Vec3(car.physics.velocity)
            final_position = location + velocity * time_remaining + GRAVITY * (0.5 * time_remaining ** 2)
            final_velocity = velocity + GRAVITY * time_remaining

        delta = self.target_location - final_position
        direction = _unit(delta)
        angle = max(_angle(direction, self._forward(car)), 0.0001)
        turn_time = 0.6 * (2 * math.sqrt(angle / 9))
        tau1 = turn_time * _cap(1 - 0.4 / angle, 0, 1)
        available_time = max(time_remaining - tau1, 0.001)
        required_accel = 2 * delta.length() / (available_time ** 2)
        ratio = required_accel / (BOOST_ACCEL + AIR_THROTTLE_ACCEL)
        tau2 = time_remaining - available_time * math.sqrt(1 - _cap(ratio, 0, 1))
        velocity_estimate = final_velocity + (BOOST_ACCEL + AIR_THROTTLE_ACCEL) * (tau2 - tau1) * direction
        boost_estimate = (tau2 - tau1) * BOOST_CONSUMPTION
        enough_boost = boost_estimate < float(car.boost) * 0.9
        enough_time = abs(ratio) < 0.9
        return (
            boost_estimate
            if velocity_estimate.length() < MAX_SPEED * 0.9 and enough_boost and enough_time
            else -1
        )

    @staticmethod
    def _forward(car):
        rotation = car.physics.rotation
        cp = math.cos(rotation.pitch)
        return Vec3(cp * math.cos(rotation.yaw), cp * math.sin(rotation.yaw), math.sin(rotation.pitch))

    def _run_dodge(self, car, controls, delta_time):
        self.interruptible = False
        self.dodge_elapsed += max(delta_time, 1 / 120)
        if _grounded(car) and self.dodge_elapsed > (0.1 if self.dodge_jump else 0) + 0.1:
            self.finished = True
        elif self.dodge_elapsed < 0.1 and self.dodge_jump:
            controls.jump = True
        elif self.dodge_step < 3 and self.dodge_jump:
            controls.jump = False
            self.dodge_step += 1
        elif self.dodge_elapsed < (0.1 if self.dodge_jump else 0) + 0.6:
            if self.dodge_input is None:
                forward = self._forward(car)
                flat_forward = _unit(forward.flat())
                direction = _unit(self.shot_direction.flat())
                perpendicular = Vec3(flat_forward.y, -flat_forward.x, 0)
                local = Vec3(-perpendicular.dot(direction), -flat_forward.dot(direction), 0)
                forward_velocity = forward.dot(Vec3(car.physics.velocity))
                speed_fraction = abs(forward_velocity) / MAX_SPEED
                backwards = (
                    local.x < 0
                    if abs(forward_velocity) < 100
                    else ((local.x >= 0) != (forward_velocity > 0))
                )
                local.x /= ((16 / 15) * (1 + 1.5 * speed_fraction)) if backwards else 1
                local.y /= 1 + 0.9 * speed_fraction
                self.dodge_input = _unit(local)
            controls.yaw = self.dodge_input.x
            controls.pitch = self.dodge_input.y
            controls.jump = True
        else:
            self.finished = True

    def run(self, car, controls, point_toward, time_remaining, delta_time, steer, up=None):
        if up is not None:
            self.up = Vec3(up)
        if self.finished:
            return
        if self.dodging:
            self._run_dodge(car, controls, delta_time)
            return

        if not self.aerialing:
            if self.start_boost < float(car.boost) or float(car.boost) <= 0 or time_remaining <= 0:
                self.finished = True
                return
            if self.jump_immediately or abs(steer) < 0.2:
                single = self._get_boost_estimate(car, False, self.elapsed_time, time_remaining)
                double = self._get_boost_estimate(car, True, self.elapsed_time, time_remaining)
                if single >= 0 or double >= 0:
                    self.double_jumping = double >= 0 and (double < single or single < 0)
                    self.aerialing = True
                else:
                    self.finished = True
                    return
            else:
                return

        self.interruptible = False
        self.elapsed_time += max(delta_time, 1 / 120)
        self.currently_double_jumping = False

        if not self.jumped:
            if self.elapsed_time <= MAX_JUMP_DURATION:
                controls.jump = True
            elif self.step < 3 and self.double_jumping:
                controls.jump = False
                self.step += 1
            elif self.step < 6 and self.double_jumping:
                controls.jump = True
                self.currently_double_jumping = True
                self.step += 1
            else:
                self.jumped = True

        if self.jumped:
            location = Vec3(car.physics.location)
            velocity = Vec3(car.physics.velocity)
            final_position = location + velocity * time_remaining + GRAVITY * (0.5 * time_remaining ** 2)
        else:
            final_position = self._jump_location(
                car, time_remaining, self.elapsed_time, self.double_jumping
            )

        offset = self.target_location - final_position
        required_accel = 2 * offset.length() / max(time_remaining ** 2, 0.0001)
        forward = self._forward(car)
        point_toward(offset if not self.jumped else self.slice_location - Vec3(car.physics.location))

        forward_projection = offset.dot(forward) / max(time_remaining, 0.001)
        controls.boost = (
            forward_projection
            >= (BOOST_ACCEL + AIR_THROTTLE_ACCEL) * max(delta_time, 13 / 120)
            and _angle(offset, forward) < 0.4
        )
        controls.throttle = _cap(
            forward_projection / (AIR_THROTTLE_ACCEL * max(delta_time, 1 / 120)),
            -1,
            1,
        )

        if self.currently_double_jumping:
            controls.steer = 0
            controls.yaw = 0
            controls.pitch = 0
            controls.roll = 0

        if (
            time_remaining <= 0
            or (
                self.jumped
                and offset.length() > 50
                and time_remaining > 0.5
                and required_accel * 0.8 > AIR_THROTTLE_ACCEL
                and (
                    float(car.boost) <= 0
                    or required_accel * 0.8 > BOOST_ACCEL + AIR_THROTTLE_ACCEL
                )
            )
            or (_grounded(car) and self.jumped)
        ):
            self.finished = True
        elif (
            (
                (not self.double_jumping and self.elapsed_time < 1.45)
                or (not car.has_jumped and self.jumped)
            )
            and time_remaining < 0.1
            and offset.length() < 100
        ):
            self.dodging = True
            self.dodge_elapsed = 0.0
            self.dodge_step = 0
            self.dodge_input = None
            self.dodge_jump = _grounded(car)
            self._run_dodge(car, controls, delta_time)
        elif offset.length() < 50 and not self.currently_double_jumping:
            point_toward(self.shot_direction)
