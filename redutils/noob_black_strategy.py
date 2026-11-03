"""Noob Black ground positioning controls, translated from src/bot.py.

The defense() and prepare_for_attack() control equations follow the
9c8a02b5a81f1d4854cb2f6474b99dd83d3e0c6e source. The adapter uses Grid's
existing car, boost-pad and controller objects. Party Cannon's action
mechanics remain separate and unchanged.
"""
from __future__ import annotations

from math import asin, inf, pi, tan

from .math3d import Vec3, clamp
from .objects import Ball, Field, Game
from .noob_black_drive import NoobBlackDrive


def sign(value: float, pref: float = 0.0) -> int:
    if value != 0:
        return int(value > 0) - int(value < 0)
    return int(pref > 0) - int(pref < 0)


def get_angle(a: Vec3, b: Vec3) -> float:
    try:
        a = Vec3.from_value(a)
        b = Vec3.from_value(b)
        if a.length == 0 or b.length == 0:
            return 0.0
        return a.Angle(b)
    except Exception:
        return 0.0


class NoobBlackFrontFlip:
    """The three control steps used by Noob Black's begin_front_flip()."""

    def __init__(self):
        self.finished = False
        self.interruptible = False
        self.start_time = None

    def Run(self, bot):
        if self.start_time is None:
            self.start_time = Game.time
        elapsed = Game.time - self.start_time

        # Sequence ControlSteps return a fresh ControllerState each frame.
        bot.controller.throttle = 0.0
        bot.controller.steer = 0.0
        bot.controller.pitch = 0.0
        bot.controller.yaw = 0.0
        bot.controller.roll = 0.0
        bot.controller.jump = False
        bot.controller.boost = False
        bot.controller.handbrake = False
        bot.controller.use_item = False

        if elapsed < 0.08:
            bot.controller.jump = True
        elif elapsed < 0.10:
            bot.controller.jump = False
        elif elapsed < 0.75:
            bot.controller.jump = True
            bot.controller.pitch = -1.0
        else:
            self.finished = True


class NoobBlackGroundStrategy:
    """Grounded defense and off-ball positioning from Noob Black."""

    def __init__(self):
        self.previous_forward = None
        self.drive = NoobBlackDrive()

    def _steer(self, car, target):
        return self.drive.steer_toward_target(car, target)

    def prepare_for_attack(self, bot, side=0):
        my_car = bot.me
        if my_car is None or not my_car.is_grounded:
            return False

        car_location = my_car.location
        car_velocity = my_car.velocity
        front_direction = my_car.forward
        prev_front = self.previous_forward if self.previous_forward is not None else front_direction
        goal_location = bot.their_goal.location
        ball_location = Ball.location
        controls = bot.controller

        tpos = Vec3(4000 * side, 0, 0)
        tpos = -goal_location + (tpos + goal_location).Rescale(
            tpos.Dist(-goal_location) / 5120
            * max(min(ball_location.y * goal_location.y + 1000, 5120), 1000)
        )
        if car_location.FlatDist(tpos) <= 100:
            controls.throttle = sign(
                -sign(pi / 2 - get_angle(car_velocity, front_direction))
                * car_velocity.length ** 2 / 6000
                + sign(pi / 2 - get_angle(tpos - car_location, front_direction))
                * tpos.Dist(car_location.Flatten()),
                1,
            )
        else:
            controls.throttle = sign(
                tpos.Dist(car_location.Flatten()) - car_velocity.length ** 2 / 6000
            )
        controls.handbrake = (
            get_angle(car_velocity, front_direction) <= pi / 4
            and get_angle(front_direction, tpos - car_location.Flatten()) > pi / 4
            and my_car.angular_velocity.length >= min(car_velocity.length / 500, 2)
            and get_angle(tpos - car_location.Flatten(), front_direction)
            < get_angle(tpos - car_location.Flatten(), prev_front)
        )
        controls.steer = self._steer(my_car, tpos)
        self.previous_forward = Vec3.from_value(front_direction)
        return True

    def defense(self, bot, goalie=False, fast=False):
        my_car = bot.me
        if my_car is None or not my_car.is_grounded:
            return False

        car_location = my_car.location
        car_velocity = my_car.velocity
        car_rotation = my_car.rotation
        front_direction = my_car.forward
        prev_front = self.previous_forward if self.previous_forward is not None else front_direction
        goal_location = bot.our_goal.location
        ball_location = Ball.location
        controls = bot.controller

        best_pad = None
        large_available = False
        if my_car.boost < 100 and fast == False:
            best_score = inf
            for pad in Field.boosts:
                best_score = inf
                if (
                    (abs(pad.location.x) > 3000 or abs(pad.location.y) > 4000)
                    and pad.location.Dist(-goal_location) < car_location.Dist(-goal_location)
                    and sign(pad.location.x) == sign(car_location.x)
                ):
                    if pad.is_active or pad.time_until_active <= (pad.location - car_location).length / 2300:
                        score = pad.location.Dist(car_location) + pad.location.Dist(-goal_location)
                        if pad.is_large:
                            large_available = True
                        if (
                            large_available
                            and ((best_pad is not None and not best_pad.is_large) or score < best_score)
                        ) or (not large_available or score < best_score):
                            best_pad = pad
                            best_score = score

        if best_pad is not None:
            tl = Vec3.from_value(best_pad.location)
        elif goalie == True:
            tl = Vec3(0, -goal_location.y - sign(goal_location.y) * 150, 0)
        else:
            tl = Vec3(sign(car_location.x) * 900, -goal_location.Rescale(5050).y, 0)

        if car_location.Dist(-goal_location) <= 1500:
            if (
                car_velocity.length <= 100
                and -5320 + 200 * bool(goalie == False) > car_location.y * sign(goal_location.y)
                and get_angle(front_direction, tl - car_location) > pi / 4
            ):
                controls.handbrake = True
                controls.throttle = -sign(pi / 2 - get_angle(front_direction, car_velocity))
                controls.steer = self._steer(my_car, tl) * controls.throttle
            elif (
                car_velocity.length <= 100
                and -5320 + 200 * bool(goalie == False)
                <= car_location.y * sign(goal_location.y)
                <= -5120 + 100 * bool(goalie == False)
                and get_angle(front_direction, ball_location - car_location) >= 0.2
            ):
                controls.handbrake = True
                controls.throttle = -sign(pi / 2 - get_angle(front_direction, car_velocity))
                if goalie == True:
                    if (
                        (ball_location.Flatten() - Vec3(clamp(ball_location.x, -900, 900), 0, 0) + goal_location)
                        .Angle(goal_location) <= pi / 3
                    ):
                        controls.steer = self._steer(my_car, ball_location) * controls.throttle
                    else:
                        controls.steer = self._steer(
                            my_car,
                            -goal_location + Vec3(
                                ball_location.x,
                                sign(ball_location.y + goal_location.y) / tan(pi / 3)
                                * (abs(ball_location.x) - 900),
                                0,
                            ),
                        ) * controls.throttle
                else:
                    controls.steer = self._steer(my_car, Vec3(0, car_location.y, 0)) * controls.throttle
            else:
                controls.throttle = sign(
                    -sign(pi / 2 - get_angle(car_velocity, front_direction))
                    * car_velocity.length ** 2 / 6000
                    + sign(pi / 2 - get_angle(tl - car_location, front_direction))
                    * tl.Dist(car_location),
                    1,
                )
                controls.steer = self._steer(my_car, tl)
                if (
                    goalie == False
                    and car_velocity.FlatLen() > 100
                    and get_angle(
                        (Vec3(sign(car_location.x) * 900, -goal_location.Rescale(5050).y, 0) - car_location).Flatten(),
                        car_velocity.Flatten(),
                    )
                    > min(asin(min(25000 / 11 * (tl - car_location).FlatLen(), 1)) * 2, 1)
                    < get_angle(
                        (Vec3(-sign(car_location.x) * 900, -goal_location.Rescale(5050).y, 0) - car_location).Flatten(),
                        car_velocity.Flatten(),
                    )
                ):
                    controls.throttle = -sign(pi / 2 - get_angle(car_velocity, front_direction))
        else:
            controls.throttle = 1
            controls.handbrake = (
                car_velocity.length < 1000
                and get_angle(car_velocity, front_direction) <= pi / 4
                and get_angle(front_direction, tl - car_location) > pi / 4
                and my_car.angular_velocity.length >= min(car_velocity.length / 500, 2)
                and get_angle(tl - car_location, front_direction)
                < get_angle(tl - car_location, prev_front)
            )
            controls.boost = bool(
                best_pad is not None
                and best_pad.is_large
                and my_car.is_grounded
                and not controls.handbrake
            )
            if my_car.is_grounded and abs(car_rotation.z) > 0.2:
                tl = car_location.Flatten()
            controls.steer = self._steer(my_car, tl)
            flip_distance = car_velocity.FlatLen() * 1.4 + 500 * 1.5 + car_velocity.FlatLen() ** 2 / 7000
            if (
                my_car.is_grounded
                and abs(controls.steer) <= 0.01
                and car_velocity.FlatLen() >= 1000
                and (
                    car_location.FlatDist(tl) >= flip_distance
                    or (
                        best_pad is not None
                        and best_pad.is_large
                        and abs(car_location.y + front_direction.y * flip_distance) < 5000
                    )
                )
            ):
                bot.action = NoobBlackFrontFlip()

        self.previous_forward = Vec3.from_value(front_direction)
        return True
