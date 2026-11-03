"""Noob Black ground-driving controls ported for Grid.

The steering and powerslide conditions are adapted from Noob-Black-425
(src/util/drive.py and its general-purpose drive/defense control logic).
Copyright (c) 2025 RLBot. Distributed under the MIT license.
"""
from __future__ import annotations

from math import atan2, pi

from .math3d import Vec3, clamp


class NoobBlackDrive:
    """Ground steering and controlled powerslide logic from Noob Black."""

    def __init__(self):
        self.previous_forward: Vec3 | None = None

    @staticmethod
    def steer_toward_target(car, target: Vec3, backwards: bool = False) -> float:
        relative = car.Local(Vec3.from_value(target) - car.location)
        angle = atan2(relative.y, relative.x)
        steer = clamp(angle * 5.0, -1.0, 1.0)
        return -steer if backwards else steer

    def reset(self) -> None:
        self.previous_forward = None

    def apply_ground_controls(self, bot, target: Vec3, backwards: bool = False) -> None:
        car = bot.me
        if car is None or not car.is_grounded:
            self.previous_forward = None
            return

        target = Vec3.from_value(target)
        front = car.forward.Normalize()
        speed = car.velocity.length
        to_target = target - car.location

        bot.controller.steer = self.steer_toward_target(car, target, backwards)
        bot.controller.handbrake = False

        # Noob Black only powerslides while the car is already tracking
        # forward, rotating into a substantial turn, and closing its angle
        # to the target. This avoids holding handbrake through a full spin.
        if (
            not backwards
            and speed > 100.0
            and speed < 1000.0
            and to_target.length > 1.0
            and self.previous_forward is not None
        ):
            velocity_aligned = car.velocity.Angle(front) <= pi / 4
            target_off_axis = front.Angle(to_target) > pi / 4
            angular_velocity_high = car.angular_velocity.length >= min(speed / 500.0, 2.0)
            turning_toward_target = to_target.Angle(front) < to_target.Angle(self.previous_forward)

            bot.controller.handbrake = bool(
                velocity_aligned
                and target_off_axis
                and angular_velocity_high
                and turning_toward_target
            )

        self.previous_forward = Vec3.from_value(front)
