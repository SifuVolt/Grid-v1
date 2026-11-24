"""
Copyright (c) 2023 Eric Veilleux. MIT licensed.

Dubins path planner adapted from VirxEC/dubins_paths (Rust).

The planner connects a car's current 2D position/heading to a target
position/heading using the six standard Dubins path families. It exposes
sampled points so the shot controller can follow a curved approach.
"""

import math


_TAU = math.tau
_PATH_TYPES = ("LSL", "LSR", "RSL", "RSR", "RLR", "LRL")


def _mod2pi(angle):
    result = angle % _TAU
    return result + _TAU if result < 0.0 else result


def _path_parameters(alpha, beta, distance, path_type):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    cab = math.cos(alpha - beta)
    d2 = distance * distance

    if path_type == "LSL":
        p2 = 2.0 * distance * (sa - sb) + 2.0 + d2 - 2.0 * cab
        if p2 < 0.0:
            return None
        tmp0 = distance + sa - sb
        tmp1 = math.atan2(cb - ca, tmp0)
        return (_mod2pi(tmp1 - alpha), math.sqrt(p2),
                _mod2pi(beta - tmp1))

    if path_type == "RSR":
        p2 = 2.0 * distance * (sb - sa) + 2.0 + d2 - 2.0 * cab
        if p2 < 0.0:
            return None
        tmp0 = distance - sa + sb
        tmp1 = math.atan2(ca - cb, tmp0)
        return (_mod2pi(alpha - tmp1), math.sqrt(p2),
                _mod2pi(tmp1 - beta))

    if path_type == "LSR":
        p2 = d2 - 2.0 + 2.0 * cab + 2.0 * distance * (sa + sb)
        if p2 < 0.0:
            return None
        p = math.sqrt(p2)
        tmp0 = math.atan2(-ca - cb, distance + sa + sb)
        tmp1 = math.atan2(-2.0, p)
        value = tmp0 - tmp1
        return (_mod2pi(value - alpha), p,
                _mod2pi(value - beta))

    if path_type == "RSL":
        p2 = d2 - 2.0 + 2.0 * cab - 2.0 * distance * (sa + sb)
        if p2 < 0.0:
            return None
        p = math.sqrt(p2)
        tmp0 = math.atan2(ca + cb, distance - sa - sb)
        tmp1 = math.atan2(2.0, p)
        value = tmp0 - tmp1
        return (_mod2pi(alpha - value), p,
                _mod2pi(beta - value))

    if path_type == "RLR":
        value = (2.0 * distance * (sa - sb) + 2.0 * cab
                 + 6.0 - d2) / 8.0
        if abs(value) > 1.0:
            return None
        p = _mod2pi(_TAU - math.acos(value))
        phi = math.atan2(ca - cb, distance - sa + sb)
        t = _mod2pi(alpha - phi + _mod2pi(p / 2.0))
        return (t, p, _mod2pi(alpha - beta - t + _mod2pi(p)))

    if path_type == "LRL":
        value = (2.0 * distance * (sb - sa) + 2.0 * cab
                 + 6.0 - d2) / 8.0
        if abs(value) > 1.0:
            return None
        p = _mod2pi(_TAU - math.acos(value))
        phi = math.atan2(ca - cb, distance + sa - sb)
        t = _mod2pi(-alpha - phi + p / 2.0)
        return (t, p, _mod2pi(beta - alpha - t + _mod2pi(p)))

    return None


def _segment(distance, pose, segment_type):
    x, y, heading = pose
    sine = math.sin(heading)
    cosine = math.cos(heading)

    if segment_type == "L":
        return (
            x + math.sin(heading + distance) - sine,
            y - math.cos(heading + distance) + cosine,
            heading + distance,
        )
    if segment_type == "R":
        return (
            x - math.sin(heading - distance) + sine,
            y + math.cos(heading - distance) - cosine,
            heading - distance,
        )
    return (
        x + cosine * distance,
        y + sine * distance,
        heading,
    )


class DubinsPath:
    """One shortest feasible Dubins path between two oriented 2D poses."""

    def __init__(self, start, end, radius, path_type, parameters):
        self.start = start
        self.end = end
        self.radius = radius
        self.path_type = path_type
        self.parameters = parameters
        self.length = sum(parameters) * radius

    @classmethod
    def shortest_valid(cls, start, end, radius, validator=None,
                       sample_step=160.0):
        radius = float(radius)
        if radius <= 0.0:
            return None

        sx, sy, start_heading = start
        ex, ey, end_heading = end
        dx = ex - sx
        dy = ey - sy
        distance = math.hypot(dx, dy) / radius
        theta = _mod2pi(math.atan2(dy, dx)) if distance > 0.0 else 0.0
        alpha = _mod2pi(start_heading - theta)
        beta = _mod2pi(end_heading - theta)

        candidates = []
        for path_type in _PATH_TYPES:
            parameters = _path_parameters(alpha, beta, distance, path_type)
            if parameters is None:
                continue
            path = cls(start, end, radius, path_type, parameters)
            if validator is not None:
                samples = path.sample_many(sample_step)
                if not all(validator(x, y) for x, y, _ in samples):
                    continue
            candidates.append(path)

        if not candidates:
            return None
        return min(candidates, key=lambda candidate: candidate.length)

    def sample(self, travel_distance):
        """Return (x, y, heading) at a distance along this path."""
        normalized = max(
            0.0, min(float(travel_distance) / self.radius,
                     sum(self.parameters))
        )
        types = tuple(self.path_type)
        q0 = (0.0, 0.0, self.start[2])
        q1 = _segment(self.parameters[0], q0, types[0])
        q2 = _segment(self.parameters[1], q1, types[1])

        if normalized < self.parameters[0]:
            pose = _segment(normalized, q0, types[0])
        elif normalized < self.parameters[0] + self.parameters[1]:
            pose = _segment(
                normalized - self.parameters[0], q1, types[1]
            )
        else:
            pose = _segment(
                normalized - self.parameters[0] - self.parameters[1],
                q2, types[2]
            )

        return (
            pose[0] * self.radius + self.start[0],
            pose[1] * self.radius + self.start[1],
            _mod2pi(pose[2]),
        )

    def sample_many(self, step_distance=160.0):
        step = max(1.0, float(step_distance))
        samples = [
            self.sample(distance)
            for distance in range(0, int(self.length), int(step))
        ]
        endpoint = self.sample(self.length)
        if not samples or math.hypot(
            samples[-1][0] - endpoint[0],
            samples[-1][1] - endpoint[1]
        ) > 0.5:
            samples.append(endpoint)
        return samples
