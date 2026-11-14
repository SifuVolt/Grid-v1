import virxrlcu
from util.agent import VirxERLU
from util.utils import (Vector, almost_equals, cap, cap_in_field, defaultDrive,
                        defaultPD, lerp, math, peek_generator, side, sign,
                        dodge_impulse)
import random

dt = 1/120
max_speed = 2300
throttle_accel = 66 + (2/3)
brake_accel = Vector(x=-3500)
boost_per_second = 33 + (1/3)
min_boost_time = 1/30
jump_max_duration = 0.2
jump_speed = 291 + (2/3)
jump_acc = 1458 + (1/3)


def request_half_flip(agent: VirxERLU) -> bool:
    """Start a half-flip only when one is not already being spammed."""
    last_start = getattr(agent, "_last_half_flip_time", -10000.0)
    if agent.time - last_start < 1.5:
        return False
    agent._last_half_flip_time = agent.time
    agent.push(half_flip())
    return True


class wave_dash:
    def __init__(self, target=None):
        self.step = -1
        # 0 = forward, 1 = right, 2 = backwards, 3 = left
        self.direction = 0
        self.start_time = -1
        self.target = target

    def run(self, agent: VirxERLU):
        if self.start_time == -1:
            self.start_time = agent.time
        T = agent.time - self.start_time

        self.step += 1

        if self.target is not None:
            self.direction = 0 if abs(self.target.x) > abs(self.target.y) else 1

            if (self.direction == 0 and self.target.x < 0) or (self.direction == 1 and self.target.y < 0):
                self.direction += 2

        forward_target = agent.me.velocity.flatten().normalize() * 100

        target_switch = {
            0: forward_target + Vector(z=50),
            1: forward_target,
            2: forward_target - Vector(z=50),
            3: forward_target
        }

        target_up = {
            0: Vector(z=1),
            1: Vector(y=-50, z=1),
            2: Vector(z=1),
            3: Vector(y=50, z=1)
        }

        defaultPD(agent, agent.me.local(target_switch[self.direction]), up=agent.me.local(target_up[self.direction]))
        if self.direction == 0:
            agent.controller.throttle = 1
        elif self.direction == 2:
            agent.controller.throttle = -1
        else:
            agent.controller.handbrake = True

        if self.step < 1:
            agent.controller.jump = True
        elif self.step < 4:
            pass
        elif not agent.me.airborne:
            agent.pop()
        elif T > 2:
            agent.pop()
            agent.push(recovery(self.target))
        elif agent.me.location.z + (agent.me.velocity.z * 0.2) < 5:
            agent.controller.jump = True
            agent.controller.yaw = 0
            if self.direction in {0, 2}:
                agent.controller.roll = 0
                agent.controller.pitch = -1 if self.direction == 0 else 1
            else:
                agent.controller.roll = -1 if self.direction == 1 else 1
                agent.controller.pitch = 0


class double_jump:
    # Hits a target point at a target time towards a target direction
    def __init__(self, intercept_time, targets=None):
        self.ball_location = None
        self.shot_vector = None
        self.offset_target = None
        self.intercept_time = intercept_time
        self.targets = targets
        # Flags for what part of the routine we are in
        self.jumping = False
        self.dodged = False
        self.jump_time = -1
        self.needed_jump_time = -1
        self.counter = 0
        self.speed_gain_attempted = False

    def update(self, shot):
        self.intercept_time = shot.intercept_time
        self.targets = shot.targets
    def needs_half_flip(self, agent):
        if self.targets is None:
            return False

        if self.intercept_time <= agent.time:
            return False

        slice_n = round((self.intercept_time - agent.time) * 120) - 1

        if slice_n < 0:
            slice_n = 0

        ball = agent.ball_prediction_struct.slices[slice_n].physics.location
        ball_location = Vector(ball.x, ball.y, ball.z)

        direction = (ball_location - agent.me.location).normalize()

        shot_vector = direction.clamp2D(
            (self.targets[0] - ball_location).normalize(),
            (self.targets[1] - ball_location).normalize()
        )

        offset_target = ball_location - shot_vector * agent.best_shot_value

        local_target = agent.me.local_location(offset_target)
        local_ball = agent.me.local_location(agent.ball.location)

        # A predicted contact point can be behind the car even while the live
        # ball is in front. Only half-flip when the ball itself is truly behind.
        return (
            local_target.x < 0
            and local_ball.x < -250
            and agent.me.location.flat_dist(agent.ball.location) > 700
        )


    def run(self, agent: VirxERLU):
        # This routine is the same as jump_shot, but it's designed to hit the ball above 300uus and below 450uus without requiring boost
        if not agent.shooting:
            agent.shooting = True

        T = self.intercept_time - agent.time
        # Capping T above 0 to prevent division problems
        time_remaining = cap(T, 0.000001, 6)

        if not self.jumping or self.ball_location is None:
            slice_n = round(T * 120) - 1
            agent.dbg_2d(f"Shot slice #: {slice_n}")

            ball = agent.ball_prediction_struct.slices[slice_n].physics.location
            self.ball_location = Vector(ball.x, ball.y, ball.z)
            self.needed_jump_time = virxrlcu.get_double_jump_time(ball.z - agent.me.location.z, agent.me.velocity.z, agent.gravity.z)

        direction = (self.ball_location - agent.me.location).normalize()
        self.shot_vector = direction if self.targets is None else direction.clamp2D((self.targets[0] - self.ball_location).normalize(), (self.targets[1] - self.ball_location).normalize())
        self.offset_target = self.ball_location - (self.shot_vector * agent.best_shot_value)
        agent.sphere(self.ball_location, 92.75)

        car_to_ball = self.ball_location - agent.me.location
        final_target = self.offset_target.copy().flatten()
        Tj = T - self.needed_jump_time * 1.15
        agent.dbg_2d(f"Needed jump time: {round(self.needed_jump_time, 2)}")

        if Tj > 0 and self.targets is not None:
            # whether we are to the left or right of the shot vector
            side_of_shot = sign(self.shot_vector.cross(Vector(z=1)).dot(car_to_ball))
            car_to_offset_target = final_target - agent.me.location
            car_to_dodge_perp = car_to_offset_target.cross(Vector(z=side_of_shot))  # perpendicular

            # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
            # The adjustment slowly decreases to 0 as the bot nears the time to jump
            adjustment = car_to_offset_target.angle2D(self.shot_vector) * Tj * 500  # size of adjustment
            # we don't adjust the final target if we are already jumping
            final_target += car_to_dodge_perp.normalize() * adjustment

        distance_remaining = final_target.flat_dist(agent.me.location)

        # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts or walls to reach it
        final_target = cap_in_field(agent, final_target)
        local_final_target = agent.me.local_location(final_target)

        # whether we should go forwards or backwards
        angle_to_target = abs(Vector(x=1).angle2D(local_final_target))
        local_ball = agent.me.local_location(agent.ball.location)

        direction = 1

        if (
            angle_to_target >= 1.6
            and agent.me.local_velocity().x <= 1000
            and local_ball.x < -250
        ):
            direction = -1

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.offset_target, agent.renderer.white)
        agent.line(self.offset_target-Vector(z=100), self.offset_target+Vector(z=100), agent.renderer.green)
        agent.line(final_target-Vector(z=100), final_target+Vector(z=100), agent.renderer.purple)

        vf = agent.me.velocity + agent.gravity * T

        distance_remaining = agent.me.local_location(self.offset_target).x if agent.me.airborne else distance_remaining
        distance_remaining -= agent.me.hitbox.length / 3
        speed_required = distance_remaining / time_remaining
        agent.dbg_2d(f"Speed required: {round(speed_required, 2)}")

        ball_speed_toward_shot = agent.ball.velocity.dot(self.shot_vector)

        if (
            speed_required < 1900
            and agent.me.boost > 50
            and T > 2
            and distance_remaining > 2560
            and ball_speed_toward_shot > 600    
        ):
            agent.dbg_2d(
                f"Building speed | Ball speed toward shot: {round(ball_speed_toward_shot)}"
            )
            speed_required *= 0.75

        if not self.jumping:
            velocity = defaultDrive(agent, speed_required * direction, local_final_target)[1]
            if velocity == 0: velocity = 1

            local_offset_target = agent.me.local_location(self.offset_target.flatten())
            local_vf = agent.me.local(vf.flatten())
            time = distance_remaining / (velocity + dodge_impulse(agent))

            if ((abs(velocity) < 100 and distance_remaining < agent.me.hitbox.length / 2) or (abs(local_offset_target.y) < 92 and direction * local_vf.x >= direction * (local_offset_target.x - agent.me.hitbox.length / 3) and direction * local_offset_target.x > 0)) and T <= self.needed_jump_time * 1.025:
                self.jumping = True
            elif agent.me.airborne:
                agent.push(recovery(final_target if Tj > 0 else None))
            elif T <= self.needed_jump_time or (Tj > 0 and distance_remaining > 50 and (not virxrlcu.double_jump_shot_is_viable(T, agent.boost_accel, tuple(agent.gravity), agent.me.get_raw(agent), self.offset_target.z, tuple((final_target - agent.me.location).normalize()), distance_remaining))):
                # If we're out of time or the ball was hit away or we just can't get enough speed, pop
                agent.pop()
                agent.shooting = False
                if agent.me.airborne:
                    agent.push(ball_recovery())
            elif not self.speed_gain_attempted and agent.boost_amount != 'unlimited' and agent.me.boost < 48 and angle_to_target < 0.03 and velocity > 600 and time >= self.needed_jump_time * 1.15 + 1.75:
                self.speed_gain_attempted = True
                if agent.gravity.z < -450 and time < 5:
                    agent.push(wave_dash(agent.me.local_location(self.offset_target)))
                else:
                    agent.push(flip(agent.me.local_location(self.offset_target)))
            elif not self.speed_gain_attempted and agent.boost_amount != 'unlimited' and angle_to_target >= 2 and velocity < 200 and time >= self.needed_jump_time * 1.15 + 2 and distance_remaining > 1280 and agent.me.local_location(agent.ball.location).x < -250:
                self.speed_gain_attempted = True
                agent.push(flip(agent.me.local_location(self.offset_target), True))
        else:
            # Mark the time we started jumping so we know when to dodge
            if self.jump_time == -1:
                self.jump_time = agent.time

            jump_elapsed = agent.time - self.jump_time
            tau = jump_max_duration - jump_elapsed

            xf = agent.me.location + agent.me.velocity * T + 0.5 * agent.gravity * T * T

            if jump_elapsed == 0:
                vf += agent.me.up * jump_speed
                xf += agent.me.up * jump_speed * T

            hf = vf
            vf += agent.me.up * jump_acc * tau
            xf += agent.me.up * jump_acc * tau * (T - 0.5 * tau)

            hf += agent.me.up * jump_speed
            vf += agent.me.up * jump_speed
            xf += agent.me.up * jump_speed * (T - tau)

            delta_x = self.offset_target - xf
            d_direction = delta_x.normalize()

            if direction == 1 and abs(agent.me.forward.dot(d_direction)) > 0.5:
                delta_v = delta_x.dot(agent.me.forward) / T
                if agent.me.boost > 0 and delta_v >= agent.boost_accel * min_boost_time:
                    agent.controller.boost = True
                else:
                    agent.controller.throttle = cap(delta_v / (throttle_accel * min_boost_time), -1, 1)

            if T <= -0.4 or (not agent.me.airborne and self.counter == 4):
                agent.pop()
                agent.shooting = False
                agent.shot_weight = -1
                agent.shot_time = -1
                agent.push(ball_recovery())
            elif jump_elapsed < jump_max_duration and hf.z <= self.offset_target.z:
                agent.controller.jump = True
            elif self.counter < 4:
                self.counter += 1

            if self.counter == 3:
                agent.controller.jump = True
            elif self.counter == 4:
                defaultPD(agent, agent.me.local_location(self.offset_target) * direction, upside_down=True)

            if self.counter < 3:
                defaultPD(agent, agent.me.local_location(self.offset_target.flatten()) * direction)

        l_vf = vf + agent.me.location
        agent.line(l_vf-Vector(z=100), l_vf+Vector(z=100), agent.renderer.red)


class Aerial:
    def __init__(self, intercept_time, targets=None, fast_aerial=True):
        self.intercept_time = intercept_time
        self.fast_aerial = fast_aerial
        self.targets = targets
        self.shot_vector = None
        self.target = None
        self.ball = None

        self.jump_type_fast = None
        self.jumping = False
        self.dodging = False
        self.ceiling = False
        self.jump_time = -1
        self.counter = 0

    def update(self, shot):
        self.intercept_time = shot.intercept_time
        self.fast_aerial = shot.fast_aerial
        self.targets = shot.targets

    @staticmethod
    def orient_for_shot(agent, delta_x, shot_vector, jumping=False):
        """Use the aerial strike controller to align for a goal-directed contact."""
        target = delta_x if delta_x.magnitude() > 50 else shot_vector
        target = agent.me.local(target)

        if jumping:
            defaultPD(agent, target.flatten(), up=agent.me.up)
        elif virxrlcu.find_landing_plane(
            tuple(agent.me.location), tuple(agent.me.velocity), agent.gravity.z
        ) == 4:
            defaultPD(agent, target, upside_down=True)
        else:
            defaultPD(agent, target, upside_down=shot_vector.z < 0)

    def run(self, agent: VirxERLU):
        if not agent.shooting:
            agent.shooting = True

        T = self.intercept_time - agent.time
        xf = agent.me.location + agent.me.velocity * T + 0.5 * agent.gravity * T * T
        vf = agent.me.velocity + agent.gravity * T

        slice_n = math.ceil(T * 120) - 1
        agent.dbg_2d(f"Shot slice #: {slice_n}")

        if T > 0.1 or self.ball is None:
            ball = agent.ball_prediction_struct.slices[slice_n].physics.location
            self.ball = Vector(ball.x, ball.y, ball.z)
            self.ceiling = agent.me.location.z > 2044 - agent.me.hitbox.height * 2 and not agent.me.jumped

        direction = (agent.ball.location - agent.me.location).normalize()
        self.shot_vector = direction if self.targets is None else direction.clamp((self.targets[0] - self.ball).normalize(), (self.targets[1] - self.ball).normalize())
        self.target = self.ball - (self.shot_vector * agent.best_shot_value)
        agent.sphere(self.ball, 92.75)

        if self.ceiling:
            self.target -= Vector(z=92)

        if self.jumping or not agent.me.airborne:
            agent.dbg_2d("Jumping")

            if not self.jumping or not agent.me.airborne:
                self.jump_type_fast = self.fast_aerial
                self.jumping = True
                self.jump_time = agent.time
                self.counter = 0

            jump_elapsed = agent.time - self.jump_time

            # how much of the jump acceleration time is left
            tau = jump_max_duration - jump_elapsed

            # impulse from the first jump
            if jump_elapsed == 0:
                vf += agent.me.up * jump_speed
                xf += agent.me.up * jump_speed * T

            # acceleration from holding jump
            vf += agent.me.up * jump_acc * tau
            xf += agent.me.up * jump_acc * tau * (T - 0.5 * tau)

            if self.jump_type_fast:
                # impulse from the second jump
                vf += agent.me.up * jump_speed
                xf += agent.me.up * jump_speed * (T - tau)

                if jump_elapsed <= jump_max_duration:
                    agent.controller.jump = True
                elif self.counter < 4:
                    self.counter += 1

                if self.counter == 3:
                    agent.controller.jump = True
                    self.dodging = True
                elif self.counter == 4:
                    self.dodging = self.jumping = False
            elif jump_elapsed <= jump_max_duration:
                agent.controller.jump = True
            else:
                self.jumping = False

        if self.ceiling:
            agent.dbg_2d(f"Ceiling shot")

        delta_x = self.target - xf
        direction = delta_x.normalize() if not self.jumping else delta_x.flatten().normalize()

        agent.line(agent.me.location, agent.me.location + (direction * 250), agent.renderer.black)
        c_vf = vf + agent.me.location
        agent.line(c_vf - Vector(z=100), c_vf + Vector(z=100), agent.renderer.blue)
        agent.line(xf - Vector(z=100), xf + Vector(z=100), agent.renderer.red)
        agent.line(self.target - Vector(z=100), self.target + Vector(z=100), agent.renderer.green)

        if not self.dodging:
            self.orient_for_shot(
                agent, delta_x, self.shot_vector,
                jumping=agent.controller.jump
            )

        # only boost/throttle if we're facing the right direction
        if abs(agent.me.forward.dot(direction)) > 0.5 and T > 0:
            if T > 1 and not self.jumping: agent.controller.roll = 1 if self.shot_vector.z < 0 else -1
            # the change in velocity the bot needs to put it on an intercept course with the target
            delta_v = delta_x.dot(agent.me.forward) / T
            if agent.me.boost > 0 and delta_v >= agent.boost_accel * min_boost_time:
                agent.controller.boost = True
                delta_v -= agent.boost_accel * min_boost_time

            if delta_v >= throttle_accel * min_boost_time:
                agent.controller.throttle = cap(delta_v / (throttle_accel * min_boost_time), -1, 1)

        if T <= -0.4 or (not self.jumping and T > 1.5 and not virxrlcu.aerial_shot_is_viable(T, agent.boost_accel, tuple(agent.gravity), agent.me.get_raw(agent), tuple(self.target))):
            agent.pop()
            agent.shooting = False
            agent.push(ball_recovery())
        elif (self.ceiling and self.target.dist(agent.me.location) < 92 + agent.me.hitbox.length and not agent.me.doublejumped and agent.me.location.z < agent.ball.location.z + 92 and self.target.y * side(agent.team) > -4240) or (not self.ceiling and not agent.me.doublejumped and T < 0.1):
            local_target = agent.me.local_location(self.target).flatten()
            local_ball = agent.me.local_location(self.ball.location).flatten()
            if local_ball.x > 0 and local_target.x > 0:
                # Preserve a forward dodge at contact. Don't derive yaw from a
                # target that can move laterally/behind the car at the last tick.
                agent.dbg_2d("Forward aerial dodge")
                agent.controller.pitch = -1.0
                agent.controller.yaw = 0.0
                agent.controller.jump = True
            else:
                # Better to finish the aerial without a dodge than to backflip
                # away from a ball that is already behind the car.
                agent.dbg_2d("Skipping backwards aerial dodge")

class half_flip:
    """
    Performs a proper backwards dodge followed by a dodge cancel.

    Unlike flip(..., cancel=True), this always starts with a true
    backwards dodge rather than using an arbitrary target vector.
    """
    def __init__(self):
        self.time = -1
        self.counter = 0

    def run(self, agent: VirxERLU):
        if self.time == -1:
            self.time = agent.time

        elapsed = agent.time - self.time

        # Initial jump
        if elapsed < 0.075:
            agent.controller.jump = True
            agent.controller.pitch = 1
            return

        # Release jump briefly so the dodge becomes available
        if self.counter < 3:
            agent.controller.jump = False
            self.counter += 1
            return

        # Backflip, then cancel it
        if elapsed < 0.20:
            agent.controller.jump = True
            agent.controller.pitch = 1
            return

        if elapsed < 0.40:
            agent.controller.jump = True
            agent.controller.pitch = -1
            return

        # Let recovery take over
        agent.pop()
        agent.push(recovery())
        return True
class flip:
    # Flip takes a vector in local coordinates and flips/dodges in that direction
    # cancel causes the flip to cancel halfway through, which can be used to half-flip
    def __init__(self, vector, cancel=False):
        vector = vector.flatten().normalize()
        self.pitch = -vector.x
        self.yaw = vector.y
        self.cancel = cancel
        # the time the jump began
        self.time = -1
        # keeps track of the frames the jump button has been released
        self.counter = 0

    def run(self, agent: VirxERLU, manual=False):
        if self.time == -1:
            elapsed = 0
            self.time = agent.time
        else:
            elapsed = agent.time - self.time

        if elapsed < 0.075:
            agent.controller.jump = True
        elif elapsed >= 0.075 and self.counter < 3:
            agent.controller.jump = False
            self.counter += 1
        elif elapsed < 0.4 or (not self.cancel and elapsed < 0.9):
            agent.controller.jump = True
            agent.controller.pitch = self.pitch
            agent.controller.yaw = self.yaw
        else:
            if not manual:
                agent.pop()
            agent.push(recovery())
            return True


class brake:
    @staticmethod
    def run(agent: VirxERLU, manual=False):
        # current forward velocity
        speed = agent.me.local_velocity().x
        if abs(speed) > 100:
            # apply our throttle in the opposite direction
            agent.controller.throttle = -cap(speed / throttle_accel, -1, 1)
        elif not manual:
            agent.pop()


class goto:
    # Drives towards a designated (stationary) target
    # Optional vector controls where the car should be pointing upon reaching the target
    def __init__(self, target, vector=None, brake=False):
        self.target = target
        self.vector = vector
        self.brake = brake

        self.f_brake = False
        self.rule1_timer = -1

    def run(self, agent: VirxERLU, manual=False):
        car_to_target = self.target - agent.me.location
        distance_remaining = car_to_target.flatten().magnitude()

        agent.dbg_2d(f"Distance to target: {round(distance_remaining)}")
        agent.line(self.target - Vector(z=500), self.target + Vector(z=500), (255, 0, 255))

        if self.brake and (self.f_brake or distance_remaining * 0.95 < (agent.me.local_velocity().x ** 2 * -1) / (2 * brake_accel.x)):
            self.f_brake = True
            brake.run(agent, manual=manual)
            return

        if not self.brake and not manual and distance_remaining < 640:
            agent.pop()
            return

        final_target = self.target.copy().flatten()

        if self.vector is not None:
            # See comments for adjustment in jump_shot for explanation
            side_of_vector = sign(self.vector.cross(Vector(z=1)).dot(car_to_target))
            car_to_target_perp = car_to_target.cross(Vector(z=side_of_vector)).normalize()
            adjustment = car_to_target.angle2D(self.vector) * distance_remaining / 3.14
            final_target += car_to_target_perp * adjustment

        final_target = cap_in_field(agent, final_target)  # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts to reach it
        local_target = agent.me.local_location(final_target)
        angle_to_target = abs(Vector(x=1).angle2D(local_target))
        angle_to_target = abs(Vector(x=1).angle2D(local_target))
        agent.dbg_2d(f"Angle to target: {round(angle_to_target, 1)}")

        # Never deliberately reverse toward a positioning target.
        # If the target is behind us, turn around and then drive toward it.
        direction = 1

        velocity = defaultDrive(agent, 2300, local_target)[1]
        if distance_remaining < 1280: agent.controller.boost = False
        if velocity == 0: velocity = 1

        # this is to break rule 1's with TM8'S ONLY
        # 251 is the distance between center of the 2 longest cars in the game, with a bit extra
        if len(agent.friends) > 0 and agent.me.local_velocity().x < 50 and agent.controller.throttle == 1 and min(agent.me.location.flat_dist(car.location) for car in agent.friends) < 251:
            if self.rule1_timer == -1:
                self.rule1_timer = agent.time
            elif agent.time - self.rule1_timer > 1.5:
                agent.push(flip(Vector(y=250)))
                return
        elif self.rule1_timer != -1:
            self.rule1_timer = -1

        if agent.me.airborne:
            agent.push(recovery(self.target))
        elif agent.boost_amount != 'unlimited' and agent.me.boost < 60 and angle_to_target < 0.03 and velocity > 500 and distance_remaining / (velocity + dodge_impulse(agent)) > 1.25:
            agent.push(flip(agent.me.local_location(self.target)))
        elif (
            not agent.me.airborne
            and angle_to_target > 2.5
            and agent.me.local_velocity().x < -300
            and distance_remaining > 600
        ):
            request_half_flip(agent)


class shadow:
    def __init__(self):
        self.goto = goto(Vector(), brake=True)

    def run(self, agent: VirxERLU):
        ball_loc = self.get_ball_loc(agent, True)
        target = self.get_target(agent, ball_loc)

        self_to_target = agent.me.location.flat_dist(target)

        if self_to_target < 100 * (agent.me.velocity.magnitude() / 500) and ball_loc.y < -640 and agent.me.velocity.magnitude() < 50 and abs(Vector(x=1).angle2D(agent.me.local_location(agent.ball.location))) > 1:
            agent.pop()
            if len(agent.friends) > 1:
                agent.push(face_target(ball=True))
        else:
            self.goto.target = target
            self.goto.vector = ball_loc * Vector(y=side(agent.team)) if target.y * side(agent.team) < 1280 else None
            self.goto.run(agent)

    def is_viable(self, agent):
        ball_loc = self.get_ball_loc(agent)
        target = self.get_target(agent, ball_loc)
        self_to_target = agent.me.location.flat_dist(target)

        return self_to_target > 320

    def get_ball_loc(self, agent, render=False):
        ball_slice = agent.ball_prediction_struct.slices[min(round(agent.future_ball_location_slice * 1.1), 6)].physics.location
        ball_loc = Vector(ball_slice.x, ball_slice.y)
        if render: agent.sphere(ball_loc + Vector(z=93), 92.75, color=agent.renderer.black)
        ball_loc.y *= side(agent.team)

        if ball_loc.y < -2560 or (ball_loc.y < agent.ball.location.y * side(agent.team)):
            ball_loc = Vector(agent.ball.location.x, agent.ball.location.y * side(agent.team) + 640)

        return ball_loc

    def get_target(self, agent: VirxERLU, ball_loc=None):
        if ball_loc is None:
            ball_loc = self.get_ball_loc(agent)

        distance = 2560

        target = Vector(y=(ball_loc.y + distance) * side(agent.team))
        if target.y * side(agent.team) > -1280:
            # use linear algebra to find the proper x coord for us to stop a shot going to the net
            # y = mx + b <- yes, finally! 7th grade math is paying off xD
            p1 = agent.friend_goal.location.flatten()
            p2 = ball_loc * Vector(x=1, y=side(agent.team))
            try:
                m = (p2.y - p1.y) / (p2.x - p1.x)
                b = p1.y - (m * p1.x)
                # x = (y - b) / m
                target.x = (target.y - b) / m
            except ZeroDivisionError:
                target.x = 0
        else:
            target.x = (abs(ball_loc.x) + 320) * sign(ball_loc.x)

        return Vector(target.x, target.y)

class defense:
    def __init__(self):
        self.goto = goto(Vector(), brake=True)

    def run(self, agent: VirxERLU):
        ball = self.get_ball_loc(agent, render=True)

        # If the ball gets too close to our goal, abandon the post.
        # Let the normal logic deal with the immediate threat.
        if agent.friend_goal.location.flat_dist(agent.ball.location) < 1000:
            agent.pop()
            return

        target = self.get_target(agent, ball=ball)

        self_to_target = agent.me.location.flat_dist(target)

        if self_to_target < 250:
            agent.pop()

            if agent.me.local_velocity().x > throttle_accel:
                agent.push(brake())
            return

        self.goto.target = target

        # Face the ball while moving to the defensive position.
        self.goto.vector = agent.ball.location

        self.goto.run(agent)

    def is_viable(self, agent):
        return agent.me.location.flat_dist(self.get_target(agent)) > 320

    def get_ball_loc(self, agent: VirxERLU, render=False):
        ball_slice = agent.ball_prediction_struct.slices[
            agent.future_ball_location_slice
        ].physics.location

        ball = Vector(
            ball_slice.x,
            cap(ball_slice.y, -5120, 5120)
        )

        if render:
            agent.sphere(
                ball + Vector(z=93),
                92.75,
                color=agent.renderer.black
            )

        ball.y *= side(agent.team)

        if ball.y < agent.ball.location.y * side(agent.team):
            ball = Vector(
                agent.ball.location.x,
                agent.ball.location.y * side(agent.team) + 640
            )

        return ball

    def get_target(self, agent, ball=None):
        if ball is None:
            ball = self.get_ball_loc(agent)

        left_post = agent.friend_goal.left_post
        right_post = agent.friend_goal.right_post

        # Work in our team's coordinate system.
        ball_x = ball.x

        # Pick whichever post is further from the ball.
        if ball_x >= 0:
            target = left_post
        else:
            target = right_post

        target = target.copy()

        # Move slightly forward from the goal line so the car
        # sits right beside the post instead of inside the wall.
        target.y -= 100 * side(agent.team)

        # Move slightly inward from the post.
        target.x -= sign(target.x) * 70

        return target.flatten()
class retreat:
    def __init__(self):
        self.goto = goto(Vector(), brake=True)

    def run(self, agent: VirxERLU):
        ball = self.get_ball_loc(agent, render=True)
        target = self.get_target(agent, ball=ball)

        self_to_target = agent.me.location.flat_dist(target)

        if self_to_target < 250:
            agent.pop()

            if agent.me.local_velocity().x > throttle_accel:
                agent.push(brake())
            return

        self.goto.target = target
        self.goto.run(agent)

    def is_viable(self, agent):
        return agent.me.location.flat_dist(self.get_target(agent)) > 320

    def get_ball_loc(self, agent: VirxERLU, render=False):
        ball_slice = agent.ball_prediction_struct.slices[agent.future_ball_location_slice].physics.location
        ball = Vector(ball_slice.x, cap(ball_slice.y, -5120, 5120))
        if render: agent.sphere(ball + Vector(z=93), 92.75, color=agent.renderer.black)
        ball.y *= side(agent.team)

        if ball.y < agent.ball.location.y * side(agent.team):
            ball = Vector(agent.ball.location.x, agent.ball.location.y * side(agent.team) + 640)

        return ball

    def get_target(self, agent: VirxERLU, ball=None):
        target = None
        if ball is None:
            ball = self.get_ball_loc(agent)

        team_to_ball = [car.location.flat_dist(ball) for car in agent.friends if car.location.y * side(agent.team) >= ball.y - 50 and abs(car.location.x) < abs(ball.x)]
        self_to_ball = agent.me.location.flat_dist(ball)
        team_to_ball.append(self_to_ball)
        team_to_ball.sort()

        if agent.me.location.y * side(agent.team) >= ball.y - 50 and abs(agent.me.location.x) < abs(ball.x):
            if len(agent.friends) == 0 or abs(ball.x) < 900 or team_to_ball[-1] is self_to_ball:
                target = agent.friend_goal.location
            elif team_to_ball[0] is self_to_ball:
                target = agent.friend_goal.right_post if ball.x * side(agent.team) > 0 else agent.friend_goal.left_post

        if target is None:
            if len(agent.friends) <= 1:
                target = agent.friend_goal.location
            else:
                target = agent.friend_goal.left_post if ball.x * side(agent.team) > 0 else agent.friend_goal.right_post

        target = target.copy()
        target.y += 250 * side(agent.team) if len(agent.friends) == 0 or abs(ball.x) < 900 or team_to_ball[-1] is self_to_ball else -245 * side(agent.team)

        return target.flatten()


class face_target:
    def __init__(self, target=None, ball=False):
        self.target = target
        self.ball = ball
        self.start_loc = None
        self.counter = 0

    @staticmethod
    def get_ball_target(agent):
        ball = agent.ball_prediction_struct.slices[agent.min_intercept_slice].physics.location
        return Vector(ball.x, cap(ball.y, -5120, 5120))

    def run(self, agent: VirxERLU):
        if self.ball:
            target = self.get_ball_target(agent) - agent.me.location.flatten()
        else:
            target = agent.me.velocity.flatten() if self.target is None else (self.target - agent.me.location).flatten()

        if agent.gravity.z < -550 and agent.gravity.z > -750:
            if self.counter == 0 and abs(Vector(x=1).angle(target)) <= 0.05:
                agent.pop()
                return

            if self.counter == 0 and agent.me.airborne:
                self.counter = 3

            if self.counter < 3:
                self.counter += 1

            target = agent.me.local(target)
            if self.counter < 3:
                agent.controller.jump = True
            elif agent.me.airborne and abs(Vector(x=1).angle(target)) > 0.05:
                defaultPD(agent, target)
            else:
                agent.pop()
        else:
            target = agent.me.local(target)
            angle_to_target = abs(Vector(x=1).angle2D(target))
            if angle_to_target > 0.1:
                if self.start_loc is None:
                    self.start_loc = agent.me.location

                direction = -1 if angle_to_target < 1.57 else 1

                agent.controller.steer = cap(target.y / 100, -1, 1) * direction
                agent.controller.throttle = direction
                agent.controller.handbrake = True
            else:
                agent.pop()
                if self.start_loc is not None:
                    agent.push(goto(self.start_loc, target, True))


class goto_boost:
    # very similar to goto() but designed for grabbing boost
    def __init__(self, boost):
        self.boost = boost
        self.goto = goto(self.boost.location)

    def run(self, agent: VirxERLU):
        if not self.boost.active:
            agent.pop()
            return

        self.goto.vector = agent.ball.location if not self.boost.large and self.boost.location.flat_dist(agent.me.location) > 640 else None
        self.goto.run(agent, manual=True)


class jump_shot:
    # Hits a target point at a target time towards a target direction
    def __init__(self, intercept_time, targets=None):
        self.ball_location = None
        self.shot_vector = None
        self.offset_target = None
        self.intercept_time = intercept_time
        self.targets = targets
        # Flags for what part of the routine we are in
        self.jumping = False
        self.dodging = False
        self.counter = 0
        self.jump_time = -1
        self.needed_jump_time = -1
        self.speed_gain_attempted = False
        # Locked local dodge controls, selected from the intended goal direction.
        self.p = -1.0
        self.y = 0.0

    def update(self, shot):
        self.intercept_time = shot.intercept_time
        self.targets = shot.targets
    def needs_half_flip(self, agent):
        if self.targets is None:
            return False

        if self.intercept_time <= agent.time:
            return False

        slice_n = round((self.intercept_time - agent.time) * 120) - 1

        if slice_n < 0:
            slice_n = 0

        ball = agent.ball_prediction_struct.slices[slice_n].physics.location
        ball_location = Vector(ball.x, ball.y, ball.z)

        direction = (ball_location - agent.me.location).normalize()

        shot_vector = direction.clamp2D(
            (self.targets[0] - ball_location).normalize(),
            (self.targets[1] - ball_location).normalize()
        )

        offset_target = ball_location - shot_vector * agent.best_shot_value

        local_target = agent.me.local_location(offset_target)
        local_ball = agent.me.local_location(agent.ball.location)

        # A predicted contact point can be behind the car even while the live
        # ball is in front. Only half-flip when the ball itself is truly behind.
        return (
            local_target.x < 0
            and local_ball.x < -250
            and agent.me.location.flat_dist(agent.ball.location) > 700
        )


    def run(self, agent: VirxERLU):
        if not agent.shooting:
            agent.shooting = True

        T = self.intercept_time - agent.time
        # Capping T above 0 to prevent division problems
        time_remaining = cap(T, 0.000001, 6)

        if not self.jumping or self.ball_location is None:
            slice_n = round(T * 120) - 1
            agent.dbg_2d(f"Shot slice #: {slice_n}")

            ball = agent.ball_prediction_struct.slices[slice_n].physics.location
            self.ball_location = Vector(ball.x, ball.y, ball.z)
            self.needed_jump_time = virxrlcu.get_jump_time(ball.z - agent.me.location.z, agent.me.velocity.z, agent.gravity.z)

        direction = (self.ball_location - agent.me.location).normalize()
        self.shot_vector = direction if self.targets is None else direction.clamp2D((self.targets[0] - self.ball_location).normalize(), (self.targets[1] - self.ball_location).normalize())
        self.offset_target = self.ball_location - (self.shot_vector * agent.best_shot_value)
        agent.sphere(self.ball_location, 92.75)

        car_to_ball = self.ball_location - agent.me.location
        final_target = self.offset_target.copy().flatten()
        Tj = T - self.needed_jump_time * 1.15
        agent.dbg_2d(f"Needed jump time: {round(self.needed_jump_time, 2)}")

        if Tj > 0 and self.targets is not None:
            # whether we are to the left or right of the shot vector
            side_of_shot = sign(self.shot_vector.cross(Vector(z=1)).dot(car_to_ball))
            car_to_offset_target = final_target - agent.me.location
            car_to_offset_perp = car_to_offset_target.cross(Vector(z=side_of_shot))  # perpendicular

            # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
            # The adjustment slowly decreases to 0 as the bot nears the time to jump
            adjustment = car_to_offset_target.angle2D(self.shot_vector) * Tj * 500  # size of adjustment
            final_target += car_to_offset_perp.normalize() * adjustment

        distance_remaining = final_target.flat_dist(agent.me.location)

        # If an opponent is close behind/around the ball, approach from the
        # opposite lateral side when there is enough room. This changes the
        # approach path, not the actual ball contact point, and fades out near
        # contact so the bot doesn't miss by trying to over-angle the shot.
        if self.targets is not None and T > 0.7 and distance_remaining > 900:
            lateral = Vector(x=-self.shot_vector.y, y=self.shot_vector.x)
            if lateral.magnitude() > 0:
                lateral = lateral.normalize()
                nearby_foes = [
                    foe for foe in agent.foes
                    if foe.location.dist(self.ball_location) < 1800
                ]
                if nearby_foes:
                    closest_foe = min(
                        nearby_foes,
                        key=lambda foe: foe.location.dist(self.ball_location)
                    )
                    foe_side_value = (closest_foe.location - self.ball_location).dot(lateral)
                    if abs(foe_side_value) > 100:
                        # Move away from the opponent's lateral side.
                        approach_side = -1 if foe_side_value > 0 else 1
                    else:
                        # If the opponent is directly behind the ball, favour
                        # the side that opens the angle toward field centre.
                        centreward = Vector(x=-1 if self.ball_location.x > 0 else 1, y=0)
                        approach_side = 1 if lateral.dot(centreward) > 0 else -1
                    lateral_offset = min(350, max(0, distance_remaining * 0.20))
                    final_target += lateral * approach_side * lateral_offset

        # Keep the adjusted approach point inside the field.
        final_target = cap_in_field(agent, final_target)
        local_final_target = agent.me.local_location(final_target)

        # Shot routines must not reverse toward the ball just because the
        # target angle is awkward. Turn to face the approach instead.
        angle_to_target = abs(Vector(x=1).angle2D(local_final_target))
        direction = 1

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.offset_target, agent.renderer.white)
        agent.line(self.offset_target-Vector(z=100), self.offset_target+Vector(z=100), agent.renderer.green)
        agent.line(final_target-Vector(z=100), final_target+Vector(z=100), agent.renderer.purple)

        vf = agent.me.velocity + agent.gravity * T

        distance_remaining = agent.me.local_location(self.offset_target).x if agent.me.airborne else distance_remaining
        distance_remaining -= agent.me.hitbox.length / 3
        speed_required = distance_remaining / time_remaining
        agent.dbg_2d(f"Speed required: {round(speed_required, 2)}")

        ball_speed_toward_shot = agent.ball.velocity.dot(self.shot_vector)

        if (
            speed_required < 1900
            and agent.me.boost > 50
            and T > 2
            and distance_remaining > 2560
            and ball_speed_toward_shot > 1000    
        ):
            agent.dbg_2d(
                f"Building speed | Ball speed toward shot: {round(ball_speed_toward_shot)}"
            )
            speed_required *= 0.75

        if not self.jumping:
            agent.dbg_2d(f"jump time: {self.needed_jump_time}")

            # Normal driving still uses the predicted ball timing so we don't arrive
            # too early. However, once we're committed to a forward power dodge,
            # don't aggressively brake in the final few frames before the dodge.
            #
            # The goal is to arrive at the ball at the correct time, while preserving
            # enough forward momentum for the dodge to actually go through the ball.

            current_forward_speed = agent.me.local_velocity().x

            power_dodge_window = (
                T < 0.35
                and T > 0.1
                and agent.me.local_location(agent.ball.location).x > 0
                and current_forward_speed > 700
            )

            drive_speed = speed_required * direction

            if power_dodge_window and direction == 1:
                # Still slow down if we're substantially too fast, but don't allow
                # defaultDrive to turn the final approach into a hard brake.
                minimum_final_speed = min(current_forward_speed, 1100)
                drive_speed = max(drive_speed, minimum_final_speed)

            velocity = defaultDrive(agent, drive_speed, local_final_target)[1]

            if velocity == 0:
                velocity = 1

            local_offset_target = agent.me.local_location(self.offset_target.flatten())
            local_vf = agent.me.local(vf.flatten())
            time = distance_remaining / (velocity + dodge_impulse(agent))

            if ((abs(velocity) < 100 and distance_remaining < agent.me.hitbox.length / 2) or (abs(local_offset_target.y) < 92 and direction * local_vf.x >= direction * (local_offset_target.x - agent.me.hitbox.length / 3) and direction * local_offset_target.x > 0)) and T <= self.needed_jump_time * 1.025:
                self.jumping = True
            elif agent.me.airborne:
                agent.push(recovery(final_target if Tj > 0 else None))
            elif T <= self.needed_jump_time or (Tj > 0 and distance_remaining > 50 and (not virxrlcu.jump_shot_is_viable(T, agent.boost_accel, tuple(agent.gravity), agent.me.get_raw(agent), self.offset_target.z, tuple((final_target - agent.me.location).normalize()), distance_remaining))):
                # If we're out of time or not fast enough to be within 45 units of target at the intercept time, we pop
                agent.pop()
                agent.shooting = False
                agent.shot_weight = -1
                agent.shot_time = -1
                if agent.me.airborne:
                    agent.push(recovery())
            elif not self.speed_gain_attempted and agent.boost_amount != 'unlimited' and agent.me.boost < 48 and direction == 1 and angle_to_target < 0.03 and velocity > 600 and time >= self.needed_jump_time * 1.15 + 1.5:
                self.speed_gain_attempted = True
                if agent.gravity.z < -450 and time < 5:
                    agent.push(wave_dash(agent.me.local_location(self.offset_target)))
                else:
                    agent.push(flip(agent.me.local_location(self.offset_target)))
            elif (
                not self.speed_gain_attempted
                and agent.boost_amount != 'unlimited'
                and direction == -1
                and distance_remaining > 2560
                and velocity < 200
                and time >= self.needed_jump_time * 1.15 + 2
                and agent.me.local_location(agent.ball.location).x < -250
            ):
                self.speed_gain_attempted = True
                agent.push(flip(agent.me.local_location(self.offset_target), True))
        else:
            if self.jump_time == -1:
                self.jump_time = agent.time

            jump_elapsed = agent.time - self.jump_time
            tau = jump_max_duration - jump_elapsed

            xf = agent.me.location + agent.me.velocity * T + 0.5 * agent.gravity * T * T

            if jump_elapsed == 0:
                vf += agent.me.up * jump_speed
                xf += agent.me.up * jump_speed * T

            hf = vf.z
            vf += agent.me.up * jump_acc * tau
            xf += agent.me.up * jump_acc * tau * (T - 0.5 * tau)

            delta_x = self.offset_target - xf
            d_direction = delta_x.normalize()

            if direction == 1 and abs(agent.me.forward.dot(d_direction)) > 0.5 and self.counter < 3:
                delta_v = delta_x.dot(agent.me.forward) / T
                if agent.me.boost > 0 and delta_v >= agent.boost_accel * min_boost_time:
                    agent.controller.boost = True
                    delta_v -= agent.boost_accel * min_boost_time

                if abs(delta_v) > throttle_accel * min_boost_time:
                    agent.controller.throttle = cap(delta_v / (throttle_accel * min_boost_time), -1, 1)

            if T <= -0.8 or (not agent.me.airborne and self.counter >= 3):
                agent.pop()
                agent.shooting = False
                agent.shot_weight = -1
                agent.shot_time = -1
                agent.push(recovery())
                return
            else:
                if self.counter == 3 and agent.me.location.dist(self.offset_target) < (92.75 + agent.me.hitbox.length) * 1.02:
                    # Shot contact should be a forward power dodge, not a dodge
                    # toward a moving offset target. Recomputing pitch/yaw from
                    # that offset can produce a diagonal or backwards flip when
                    # the target shifts a little as the opponent approaches.
                    local_ball = agent.me.local_location(self.ball_location).flatten()
                    local_shot = agent.me.local(self.shot_vector).flatten()
                    if local_shot.magnitude() > 0:
                        local_shot = local_shot.normalize()

                    if local_ball.x > 0 and local_shot.x > 0.05:
                        # Convert the desired world-space shot direction into the
                        # car's local dodge axes. The aerial PD has been aligning
                        # the nose with this direction throughout the jump; this
                        # final dodge preserves any remaining lateral correction.
                        self.p = -cap(local_shot.x, -1, 1)
                        self.y = cap(local_shot.y, -1, 1)
                        agent.controller.pitch = self.p
                        agent.controller.yaw = self.y
                        agent.dbg_2d(
                            f"Jump shot dodge: pitch {self.p:.2f} yaw {self.y:.2f}"
                        )
                        self.counter += 1
                    else:
                        # The ball or desired shot direction is behind the car:
                        # don't manufacture a backwards dodge.
                        self.counter = 5
                elif self.counter == 4:
                    # Dodge once using the locked inputs above. The shot routine
                    # is also protected from replanning until this finishes.
                    agent.controller.jump = True
                    agent.controller.pitch = self.p
                    agent.controller.yaw = self.y
                else:
                    # Take over with the same predictive orientation controller
                    # used by Aerial. It first aligns toward the contact point,
                    # then transitions to the goal-directed shot vector near hit.
                    Aerial.orient_for_shot(
                        agent, delta_x, self.shot_vector, jumping=True
                    )

                if jump_elapsed <= jump_max_duration and hf <= self.offset_target.z:
                    # Initial jump to get airborne + we hold the jump button for extra power as required
                    agent.controller.jump = True
                elif self.counter < 3:
                    # Make sure we aren't jumping for at least 3 frames
                    self.counter += 1

        l_vf = vf + agent.me.location
        agent.line(l_vf-Vector(z=100), l_vf+Vector(z=100), agent.renderer.red)


class ground_shot:
    # Hits a target point at a target time towards a target direction
    def __init__(self, intercept_time, targets=None):
        self.ball_location = None
        self.shot_vector = None
        self.offset_target = None
        self.intercept_time = intercept_time
        self.targets = targets
        self.speed_gain_attempted = False

    def update(self, shot):
        self.intercept_time = shot.intercept_time
        self.targets = shot.targets
    def needs_half_flip(self, agent):
        if self.targets is None:
            return False

        if self.intercept_time <= agent.time:
            return False

        slice_n = round((self.intercept_time - agent.time) * 120) - 1

        if slice_n < 0:
            slice_n = 0

        ball = agent.ball_prediction_struct.slices[slice_n].physics.location
        ball_location = Vector(ball.x, ball.y, ball.z)

        direction = (ball_location - agent.me.location).normalize()

        shot_vector = direction.clamp2D(
            (self.targets[0] - ball_location).normalize(),
            (self.targets[1] - ball_location).normalize()
        )

        offset_target = ball_location - shot_vector * agent.best_shot_value

        local_target = agent.me.local_location(offset_target)
        local_ball = agent.me.local_location(agent.ball.location)

        # A predicted contact point can be behind the car even while the live
        # ball is in front. Only half-flip when the ball itself is truly behind.
        return (
            local_target.x < 0
            and local_ball.x < -250
            and agent.me.location.flat_dist(agent.ball.location) > 700
        )

    def run(self, agent: VirxERLU):
        if not agent.shooting:
            agent.shooting = True

        T = self.intercept_time - agent.time
        # Capping T above 0 to prevent division problems
        time_remaining = cap(T, 0.000001, 6)

        if T > 0.1 or self.ball_location is None:
            slice_n = round(T * 120) - 1
            agent.dbg_2d(f"Shot slice #: {slice_n}")

            ball = agent.ball_prediction_struct.slices[slice_n].physics.location
            self.ball_location = Vector(ball.x, ball.y, ball.z)

        direction = (self.ball_location - agent.me.location).normalize()
        self.shot_vector = direction if self.targets is None else direction.clamp2D((self.targets[0] - self.ball_location).normalize(), (self.targets[1] - self.ball_location).normalize())
        self.offset_target = self.ball_location - (self.shot_vector * agent.best_shot_value)
        agent.sphere(self.ball_location, 92.75)

        l_ball = agent.me.local(agent.ball.location)
        car_to_ball = agent.ball.location - agent.me.location
        if abs(l_ball.y) < 92.75 + agent.me.hitbox.width / 2 and abs(l_ball.z) < 92.75 + agent.me.hitbox.height / 2:
            final_target = agent.ball.location - (self.shot_vector * agent.best_shot_value)
            distance_remaining = final_target.flat_dist(agent.me.location) - agent.me.hitbox.length / 3
            speed_required = 2300
            agent.dbg_2d(f"Max speed")
        else:
            # Some adjustment to the final target to ensure it's inside the field and we don't try to drive through any goalposts or walls to reach it
            final_target = self.offset_target.copy().flatten()

            if self.targets is not None:
                # whether we are to the left or right of the shot vector
                side_of_shot = sign(self.shot_vector.cross(Vector(z=1)).dot(car_to_ball))
                car_to_offset_target = final_target - agent.me.location
                car_to_offset_perp = car_to_offset_target.cross(Vector(z=side_of_shot))  # perpendicular

                # The adjustment causes the car to circle around the dodge point in an effort to line up with the shot vector
                # The adjustment slowly decreases to 0 as the bot nears the time to jump
                adjustment = car_to_offset_target.angle2D(self.shot_vector) * T * 500  # size of adjustment
                # we don't adjust the final target if we are already jumping
                final_target += car_to_offset_perp.normalize() * adjustment

            distance_remaining = final_target.flat_dist(agent.me.location) - agent.me.hitbox.length / 3
            speed_required = distance_remaining / time_remaining
            agent.dbg_2d(f"Speed required: {speed_required}")

            ball_speed_toward_shot = agent.ball.velocity.dot(self.shot_vector)

            if (
                speed_required < 1900
                and agent.me.boost > 50
                and T > 2
                and distance_remaining > 2560
                and ball_speed_toward_shot > 1000    
            ):
                agent.dbg_2d(
                    f"Building speed | Ball speed toward shot: {round(ball_speed_toward_shot)}"
                )
                speed_required *= 0.75

        # When an opponent is close to the ball, approach the hit point from
        # the opposite lateral side if there is room to do so. The offset fades
        # out near contact so the actual shot contact stays reliable.
        if self.targets is not None and T > 0.7 and distance_remaining > 900:
            lateral = Vector(x=-self.shot_vector.y, y=self.shot_vector.x)
            if lateral.magnitude() > 0:
                lateral = lateral.normalize()
                nearby_foes = [
                    foe for foe in agent.foes
                    if foe.location.dist(self.ball_location) < 1800
                ]
                if nearby_foes:
                    closest_foe = min(
                        nearby_foes,
                        key=lambda foe: foe.location.dist(self.ball_location)
                    )
                    foe_side_value = (closest_foe.location - self.ball_location).dot(lateral)
                    if abs(foe_side_value) > 100:
                        # Approach from the side opposite the opponent.
                        approach_side = -1 if foe_side_value > 0 else 1
                    else:
                        # If the opponent is almost directly behind the ball,
                        # favour the side that opens space toward field centre.
                        centreward = Vector(x=-1 if self.ball_location.x > 0 else 1, y=0)
                        approach_side = 1 if lateral.dot(centreward) > 0 else -1

                    lateral_offset = min(350, max(0, distance_remaining * 0.20))
                    final_target += lateral * approach_side * lateral_offset

        # Keep the adjusted approach point inside the field.
        final_target = cap_in_field(agent, final_target)
        local_final_target = agent.me.local_location(final_target)

        # the angle to the final target, in radians
        angle_to_target = abs(Vector(x=1).angle2D(local_final_target))
        local_ball = agent.me.local_location(agent.ball.location)

        # Only half-flip for genuine repositioning.
        # Never half-flip when the ball is already in front of the car
        # and we're close enough to be making a shot.
        if (
            angle_to_target >= 1.6
            and agent.me.local_velocity().x <= 1000
            and local_ball.x < -250
            and agent.me.location.flat_dist(agent.ball.location) > 700
        ):
            if request_half_flip(agent):
                return

        direction = 1

        # drawing debug lines to show the dodge point and final target (which differs due to the adjustment)
        agent.line(agent.me.location, self.offset_target, agent.renderer.white)
        agent.line(self.offset_target-Vector(z=100), self.offset_target+Vector(z=100), agent.renderer.green)
        agent.line(final_target-Vector(z=100), final_target+Vector(z=100), agent.renderer.purple)

        velocity = defaultDrive(agent, speed_required * direction, local_final_target)[1]
        if velocity == 0: velocity = 1

        local_offset_target = agent.me.local_location(self.offset_target.flatten())
        time = distance_remaining / (velocity + dodge_impulse(agent))

        vf = agent.me.velocity + agent.gravity * T
        local_vf = agent.me.local(vf.flatten())

        if T < 0.35 and T > 0.1 and min(abs(velocity) + dodge_impulse(agent), 2300) <= (abs(speed_required) if agent.me.boost > 12 else 1900) and ((abs(velocity) < 100 and distance_remaining < agent.me.hitbox.length / 2) or (abs(local_offset_target.y) < 92 and direction * local_vf.x >= direction * (local_offset_target.x - agent.me.hitbox.length / 3) and direction * local_offset_target.x > 0)):
            # This is a power shot, so use a straight front flip once we're
            # lined up. The previous target-vector dodge could turn into a
            # back/diagonal flip if the computed offset point moved behind us.
            local_ball_now = agent.me.local_location(agent.ball.location)
            if local_ball_now.x > 0 and abs(local_ball_now.y) < 280:
                agent.push(flip(Vector(x=1), cancel=False))
        elif agent.me.airborne:
            agent.push(recovery(final_target if T > 0.5 else None))
        elif T <= 0 or (T > 0.35 and distance_remaining > 50 and not virxrlcu.ground_shot_is_viable(T, agent.boost_accel, agent.me.get_raw(agent), self.offset_target.z, tuple((final_target - agent.me.location).normalize()), distance_remaining)):
            # If we're out of time or not fast enough, we pop
            agent.pop()
            agent.shooting = False
            agent.shot_weight = -1
            agent.shot_time = -1
            if agent.me.airborne:
                agent.push(recovery())
        elif not self.speed_gain_attempted and agent.boost_amount != 'unlimited' and agent.me.boost < 48 and angle_to_target < 0.03 and velocity > 500 and T > 1 and time >= 1.5:
            self.speed_gain_attempted = True
            speed_gain_routine = wave_dash if agent.gravity.z < -450 and time < 3 and time > 1 else flip
            agent.push(speed_gain_routine(agent.me.local_location(self.offset_target)))


class emergency_clear:
    """
    Emergency defensive clear.

    The goal is not to make a precise shot.
    The goal is to get the ball away from our goal as quickly as possible.
    """

    def __init__(self):
        self.start_time = -1
        self.target = None
        self.has_dodged = False

    def run(self, agent: VirxERLU):
        if self.start_time == -1:
            self.start_time = agent.time

        # Once the dodge and its recovery finish, don't re-dodge the same ball.
        if self.has_dodged:
            agent.pop()
            return

        # ---------------------------------------------------------
        # Pick a safe direction to clear the ball.
        # Clear toward the nearest side of the field.
        # ---------------------------------------------------------
        ball = agent.ball.location

        if self.target is None:
            # Choose the side with less distance to travel.
            if ball.x >= 0:
                self.target = Vector(x=-3500, y=ball.y)
            else:
                self.target = Vector(x=3500, y=ball.y)

        # Keep the target inside the field.
        self.target = cap_in_field(agent, self.target)

        # ---------------------------------------------------------
        # We want to hit the ball from behind it, relative to
        # the direction we want the ball to travel.
        # ---------------------------------------------------------
        clear_direction = (self.target - ball).flatten().normalize()

        hit_target = ball - clear_direction * agent.best_shot_value

        local_target = agent.me.local_location(hit_target)
        local_ball = agent.me.local_location(ball)

        distance = agent.me.location.flat_dist(hit_target)
        angle_to_target = abs(Vector(x=1).angle2D(local_target))

        agent.dbg_2d(
            f"Emergency clear: {round(distance)} | angle {round(angle_to_target, 2)}"
        )

        # Debug:
        agent.line(
            ball,
            self.target,
            agent.renderer.red
        )

        agent.line(
            ball,
            hit_target,
            agent.renderer.green
        )

        # ---------------------------------------------------------
        # If we're airborne, recover toward the ball.
        # ---------------------------------------------------------
        if agent.me.airborne:
            agent.push(recovery(hit_target))
            return

        # ---------------------------------------------------------
        # Get to the ball aggressively.
        # ---------------------------------------------------------
        velocity = defaultDrive(agent, 2300, local_target)[1]

        if velocity == 0:
            velocity = 1

        # Face the ball / hit position.
        defaultPD(agent, local_target)

        # ---------------------------------------------------------
        # If we're lined up, BOOST HARD.
        #
        # This deliberately does NOT use the normal speed-building
        # logic from ground_shot.
        # ---------------------------------------------------------
        if (
            abs(angle_to_target) < 0.35
            and distance > 500
            and agent.me.boost > 0
        ):
            agent.controller.boost = True

        agent.controller.throttle = 1

        # ---------------------------------------------------------
        # Once we're close enough, dodge THROUGH the ball.
        # ---------------------------------------------------------
        if (
            distance < 700
            and abs(local_ball.y) < 180
            and local_ball.x > 0
        ):
            self.has_dodged = True
            agent.push(
                flip(agent.me.local(clear_direction), cancel=False)
            )
            return

        # ---------------------------------------------------------
        # Emergency timeout.
        #
        # Don't let the routine get stuck forever if the ball moved.
        # ---------------------------------------------------------
        if agent.time - self.start_time > 2.0:
            agent.pop()


class goal_line_defense:
    """Hold a goal-side position and shade toward the likely shot lane."""

    def __init__(self):
        self.start_time = -1

    def run(self, agent: VirxERLU):
        side_value = side(agent.team)
        ball = agent.ball.location
        goal_y = 5120 * side_value
        ball_depth = ball.y * side_value

        # The ball has left our half; let the normal attack/rotation logic resume.
        if ball_depth < 300:
            agent.pop()
            return

        if self.start_time < 0:
            self.start_time = agent.time

        # Predict where the ball would cross the goal plane if its current
        # velocity continued. Only use a forward-in-time projection.
        predicted_x = ball.x
        velocity_toward_goal = agent.ball.velocity.y * side_value
        if velocity_toward_goal > 150:
            time_to_goal = (goal_y - ball.y) / agent.ball.velocity.y
            if 0 <= time_to_goal <= 2.5:
                predicted_x = ball.x + agent.ball.velocity.x * time_to_goal
        predicted_x = cap(predicted_x, -850, 850)

        # Stay just in front of the goal line, moving laterally to cover the
        # projected shot. The 300-unit depth avoids sitting inside the net.
        target = Vector(x=predicted_x, y=goal_y - side_value * 300, z=17)
        local_target = agent.me.local_location(target)
        local_ball = agent.me.local_location(ball)
        distance = agent.me.location.flat_dist(target)
        angle = abs(Vector(x=1).angle2D(local_target))

        agent.dbg_2d(
            f"Goal-line defense: x {round(predicted_x)} | dist {round(distance)}"
        )
        agent.line(agent.me.location, target, agent.renderer.white)
        agent.line(ball, target, agent.renderer.red)

        if agent.me.airborne:
            agent.push(recovery(target))
            return

        # Move toward the save position while keeping the car oriented toward
        # the ball. Do not chase the ball away from the goal line.
        defaultDrive(agent, 1700, local_target)
        defaultPD(agent, local_ball)
        agent.controller.throttle = 1
        if distance > 700 and angle < 0.45 and agent.me.boost > 15:
            agent.controller.boost = True

        # A stale routine must not remain active forever if state changes.
        if agent.time - self.start_time > 5.0 and ball_depth < 1000:
            agent.pop()

class kickoff_dodge:

    def __init__(self, direction):
        self.direction = direction
        self.timer = -1

    def run(self, agent: VirxERLU):

        if self.timer == -1:
            self.timer = agent.time

        elapsed = agent.time - self.timer

        # First jump
        if elapsed < 0.05:
            agent.controller.jump = True

        # Very short jump release
        elif elapsed < 0.083:
            agent.controller.jump = False

        # SECOND JUMP / DODGE
        elif elapsed < 0.15:
            agent.controller.jump = True
            agent.controller.pitch = -self.direction.x
            agent.controller.yaw = self.direction.y

        else:
            agent.pop()
            return True

class speed_flip:
    FIRST_JUMP_DURATION = 0.1
    BETWEEN_JUMPS_DELAY = 0.1
    SECOND_JUMP_DURATION = 0.05
    TIMEOUT = 2.0

    def __init__(self, right_handed=True, use_boost=True):
        self.direction = 1 if right_handed else -1
        self.use_boost = use_boost
        self.timer = 0.0
        self.start_time = -1

    def run(self, agent: VirxERLU):

        if self.start_time == -1:
            self.start_time = agent.time

        self.timer = agent.time - self.start_time

        agent.controller.throttle = 1

        speed = agent.me.velocity.magnitude()

        agent.controller.boost = (
            self.use_boost and speed < 2290
        )

        # First jump
        if self.timer < self.FIRST_JUMP_DURATION:
            agent.controller.jump = True
            agent.controller.pitch = 1

        # Release jump
        elif self.timer < (
            self.FIRST_JUMP_DURATION +
            self.BETWEEN_JUMPS_DELAY
        ):
            agent.controller.jump = False
            agent.controller.pitch = 1

        # Second jump / diagonal dodge
        elif self.timer < (
            self.FIRST_JUMP_DURATION +
            self.BETWEEN_JUMPS_DELAY +
            self.SECOND_JUMP_DURATION
        ):
            agent.controller.jump = True
            agent.controller.pitch = -1
            agent.controller.roll = -0.3 * self.direction

        # Dodge cancel
        else:
            agent.controller.jump = False
            agent.controller.pitch = 1
            agent.controller.roll = -1 * self.direction
            agent.controller.yaw = -1 * self.direction

        # Let the kickoff routine underneath resume.
        if (
            self.timer > self.TIMEOUT
            or (
                not agent.me.airborne
                and self.timer > 0.5
            )
        ):
            agent.pop()
            return True
class speedflip_kickoff:

    def __init__(self):
        self.phase = 0
        self.speedflip_started = False
        

    def run(self, agent: VirxERLU):

        # =========================================================
        # PHASE 0
        # Accelerate toward the ball until 800 speed.
        # =========================================================

        if self.phase == 0:

            local_ball = agent.me.local_location(agent.ball.location)

            # Aim at the ball.
            defaultPD(agent, local_ball)

            agent.controller.throttle = 1
            agent.controller.boost = True

            speed = agent.me.velocity.magnitude()

            if speed > 800:

                # THIS is the important direction calculation
                # from the original SpeedFlipDodgeKickoff.
                right_handed = local_ball.y < 0

                self.phase = 1

                agent.push(
                    speed_flip(
                        right_handed=right_handed,
                        use_boost=True
                    )
                )

            return

        # =========================================================
        # PHASE 1
        #
        # speed_flip is currently on top of the stack.
        #
        # When it finishes, it pops itself and this routine
        # resumes here.
        # =========================================================

        if self.phase == 1:

            # We only get here once speed_flip has popped.
            if not agent.me.airborne:

                self.phase = 2

            return

        # =========================================================
        # PHASE 2
        # Drive toward the ball.
        # =========================================================

        if self.phase == 2:

            local_ball = agent.me.local_location(agent.ball.location)

            defaultPD(agent, local_ball)

            agent.controller.throttle = 1

            speed = agent.me.velocity.magnitude()

            if speed < 2290:
                agent.controller.boost = True

            distance = agent.me.location.flat_dist(
                agent.ball.location
            )

            # Get close enough to perform the actual hit.
            if distance < 1000:

                self.phase = 3
            
                if random.random() < 0.5:

                    local_ball = agent.me.local_location(agent.ball.location)
                    local_ball.z = 0

                    x = local_ball.x
                    y = local_ball.y

                    # Ball is very close and significantly to the left/right.
                    # A forward flip would go past it, so side flip into the ball.
                    if abs(y) > 150 and x < 300:
                        dodge_direction = Vector(
                            y=-1 if y > 0 else 1
                        )

                    # Ball is basically straight ahead.
                    elif abs(y) < 100 and x > 150:
                        dodge_direction = Vector(x=1)

                    # Ball is strongly to the side.
                    elif abs(y) > abs(x) * 0.8:
                        dodge_direction = Vector(
                            y=-1 if y > 0 else 1
                        )

                    # Otherwise, diagonal flip towards the ball.
                    else:
                        dodge_direction = Vector(
                            x=1,
                            y=y / max(abs(x), 1)
                        ).normalize()
                    agent.push(
                        kickoff_dodge(dodge_direction)
                    )
                else:
                    local_ball = agent.me.local_location(agent.ball.location)

                    defaultPD(agent, local_ball)

                    agent.controller.throttle = 1

                    speed = agent.me.velocity.magnitude()
                    return

    
            return

        # =========================================================
        # PHASE 3
        # Final dodge is on the stack.
        # =========================================================

        if self.phase == 3:

            # kickoff_dodge pops itself when complete.
            #
            # Once we're back here, finish the kickoff.
            if not agent.me.airborne:

                agent.kickoff_done = True
                agent.pop()

            return

class generic_kickoff:
    def __init__(self):
        self.start_time = -1
        self.flip = False

    def run(self, agent: VirxERLU):
        if self.start_time == -1:
            self.start_time = agent.time

        if self.flip or agent.time - self.start_time > 3:
            agent.kickoff_done = True
            agent.pop()
            return

        target = agent.ball.location + Vector(y=(200 if agent.gravity.z < -600 and agent.gravity.z > -700 else 50)*side(agent.team))
        local_target = agent.me.local_location(target)

        defaultPD(agent, local_target)
        agent.controller.throttle = 1
        agent.controller.boost = True

        distance = local_target.magnitude()

        if distance < 550:
            self.flip = True
            agent.push(flip(agent.me.local_location(agent.foe_goal.location)))


class recovery:
    # Point towards our velocity vector and land upright, unless we aren't moving very fast
    # A vector can be provided to control where the car points when it lands
    def __init__(self, target=None):
        self.target = target

    def run(self, agent: VirxERLU):
        target = agent.me.velocity.normalize() if self.target is None else (self.target - agent.me.location).normalize()

        landing_plane = virxrlcu.find_landing_plane(tuple(agent.me.location), tuple(agent.me.velocity), agent.gravity.z)

        d_switch = [
            "side wall",
            "side wall",
            "back wall",
            "back wall",
            "ceiling",
            "floor"
        ]

        agent.dbg_2d(f"Recovering towards the {d_switch[landing_plane]}")

        t_switch = [
            Vector(y=target.y, z=-1),
            Vector(y=target.y, z=-1),
            Vector(x=target.x, z=-1),
            Vector(x=target.x, z=-1),
            Vector(x=target.x, y=target.y),
            Vector(x=target.x, y=target.y)
        ]

        r_switch = [
            Vector(x=-1),
            Vector(x=1),
            Vector(y=-1),
            Vector(y=1),
            Vector(z=-1),
            Vector(z=1)
        ]

        defaultPD(agent, agent.me.local(t_switch[landing_plane]), up=agent.me.local(r_switch[landing_plane]))
        agent.controller.throttle = 1
        if not agent.me.airborne:
            agent.pop()


class ball_recovery:
    def __init__(self):
        self.recovery = recovery()

    def run(self, agent: VirxERLU):
        self.recovery.target = agent.ball.location
        self.recovery.target.y = cap(self.recovery.target.y, -5100, 5100)
        self.recovery.run(agent)


class short_shot:
    # This routine drives towards the ball and attempts to hit it towards a given target
    # It does not require ball prediction and kinda guesses at where the ball will be on its own
    def __init__(self, target):
        self.target = target
        self.start_time = None

    def run(self, agent: VirxERLU):
        if not agent.shooting or agent.shot_weight != -1:
            agent.shooting = True
            agent.shot_weight = -1

        if self.start_time is None:
            self.start_time = agent.time

        car_to_ball, distance = (agent.ball.location - agent.me.location).normalize(True)
        ball_to_target = (self.target - agent.ball.location).normalize()

        relative_velocity = car_to_ball.dot(agent.me.velocity-agent.ball.velocity)
        eta = cap(distance / cap(relative_velocity, 400, 2150), 0, 1.5) if relative_velocity != 0 else 1.5

        # If we are approaching the ball from the wrong side the car will try to only hit the very edge of the ball
        left_vector = car_to_ball.cross(Vector(z=1))
        right_vector = car_to_ball.cross(Vector(z=-1))
        target_vector = -ball_to_target.clamp2D(left_vector, right_vector)
        final_target = agent.ball.location + (target_vector*(distance/2))
        angle_to_target = abs(Vector(x=1).angle2D(agent.me.local_location(final_target)))
        distance_remaining = agent.me.location.dist(final_target) - agent.me.hitbox.length / 3

        # Some adjustment to the final target to ensure we don't try to drive through any goalposts to reach it
        final_target = cap_in_field(agent, final_target)
        local_final_target = agent.me.local_location(final_target)

        agent.line(final_target-Vector(z=100), final_target + Vector(z=100), (255, 255, 255))
        direction = 1
        angles, velocity = defaultDrive(agent, 1410 * direction, local_final_target)

        if velocity == 0: velocity = 1

        if abs(angles[1]) < 0.05 and (eta < 0.45 or distance < 150):
            agent.pop()
            agent.shooting = False
            agent.shot_weight = -1
            agent.shot_time = -1
            agent.push(flip(agent.me.local(car_to_ball)))
        elif agent.boost_amount != 'unlimited' and agent.me.location.z < 50 and agent.me.boost < 48 and angle_to_target < 0.03 and velocity > 500 and T > 1 and time >= T + 1.5:
            speed_gain_routine = wave_dash if agent.gravity.z < -450 and time < 3 and time > 1 else flip
            agent.push(speed_gain_routine(agent.me.local_location(self.target)))
        elif agent.boost_amount != 'unlimited' and agent.me.location.z < 50 and direction == -1 and velocity < 200 and time >= T + 1.5:
            agent.push(flip(agent.me.local_location(self.target), True))

class boost_down:
    def __init__(self):
        self.face = ball_recovery()

    def run(self, agent: VirxERLU):
        if agent.me.boost == 0:
            agent.pop()
            agent.push(self.face)

        target = agent.me.local(agent.me.forward.flatten()*100 - Vector(z=100))
        defaultPD(agent, target)
        if not agent.me.airborne:
            agent.pop()
        elif abs(Vector(x=1).angle(target)) < 0.5:
            agent.controller.boost = True