"""Noob Black 425 driving utility, adapted to Grid's physics types.

Source: https://github.com/NicEastvillage/Noob-Black-425
Pinned revision: 9c8a02b5a81f1d4854cb2f6474b99dd83d3e0c6e
Original utility: relative_location + atan2(relative.y, relative.x) * 5.
MIT License; Copyright (c) 2025 RLBot. See THIRD_PARTY_NOTICES.md.
"""
from __future__ import annotations

from math import atan2

from physics.math3d import Vec3, clamp
from physics.vehicle import VehicleState


def limit_to_safe_range(value: float) -> float:
    return clamp(float(value), -1.0, 1.0)


def steer_toward_target(car: VehicleState, target: Vec3) -> float:
    """Noob Black's car-relative steering law, using Grid's basis vectors."""
    relative = car.orientation.world_to_local(
        Vec3.from_value(target) - car.location
    )
    angle = atan2(relative.y, relative.x)
    return limit_to_safe_range(angle * 5.0)
