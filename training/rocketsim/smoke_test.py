"""Verify the pinned RocketSim Python binding and local arena assets.

Run from the repository root:
    venv\\Scripts\\python.exe training\\rocketsim\\smoke_test.py

This is an offline simulator test. It does not start RLBot or modify Volt's
live-match behavior.
"""

from __future__ import annotations

import argparse
import math
import os
from pathlib import Path

import RocketSim as rs


HERE = Path(__file__).resolve().parent
DEFAULT_COLLISION_MESHES = HERE / "collision_meshes"


def resolve_collision_meshes(argument: str | None) -> Path:
    candidates = []

    if argument:
        candidates.append(Path(argument).expanduser())

    environment_path = os.environ.get("RS_COLLISION_MESHES")
    if environment_path:
        candidates.append(Path(environment_path).expanduser())

    candidates.append(DEFAULT_COLLISION_MESHES)

    for candidate in candidates:
        resolved = candidate.resolve()
        if resolved.is_dir():
            return resolved

    searched = "\\n".join(f"  - {path.resolve()}" for path in candidates)
    raise FileNotFoundError(
        "RocketSim collision meshes were not found. Dump the arena assets "
        "using https://github.com/ZealanL/RLArenaCollisionDumper and place "
        f"them in {DEFAULT_COLLISION_MESHES}, or pass --collision-meshes. "
        f"Searched:\\n{searched}"
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Smoke-test the RocketSim Python binding used by Volt."
    )
    parser.add_argument(
        "--collision-meshes",
        help="Directory containing RocketSim arena collision mesh assets.",
    )
    args = parser.parse_args()

    collision_meshes = resolve_collision_meshes(args.collision_meshes)

    # The binding's init() accepts one optional path argument. It must be
    # called before constructing an Arena, and only once per Python process.
    rs.init(str(collision_meshes))

    arena = rs.Arena(
        game_mode=rs.GameMode.SOCCAR,
        tick_rate=120.0,
    )

    car = arena.add_car(rs.Team.BLUE, rs.CarConfig.OCTANE)
    car.set_state(
        rs.CarState(
            pos=rs.Vec(x=0.0, y=-2500.0, z=17.0),
            vel=rs.Vec(x=0.0, y=0.0, z=0.0),
            boost=50.0,
            is_on_ground=True,
            wheels_with_contact=(True, True, True, True),
        )
    )
    car.set_controls(rs.CarControls(throttle=1.0))

    # Step one tick at a time so the same loop can later observe transient
    # events such as CarState.got_flip_reset() without missing the event.
    reset_events = 0
    for _ in range(120):
        arena.step(1)
        state = car.get_state()
        if state.got_flip_reset():
            reset_events += 1

    final_state = car.get_state()
    coordinates = (
        final_state.pos.x,
        final_state.pos.y,
        final_state.pos.z,
    )

    if arena.tick_count != 120:
        raise RuntimeError(
            f"Expected 120 simulation ticks, got {arena.tick_count}."
        )

    if not all(math.isfinite(value) for value in coordinates):
        raise RuntimeError(f"RocketSim returned invalid car position: {coordinates}")

    if not callable(getattr(final_state, "has_flip_reset", None)):
        raise RuntimeError("RocketSim CarState.has_flip_reset() is unavailable.")

    if not callable(getattr(final_state, "got_flip_reset", None)):
        raise RuntimeError("RocketSim CarState.got_flip_reset() is unavailable.")

    print("[ROCKETSIM] Python binding import: OK")
    print(f"[ROCKETSIM] Collision meshes: {collision_meshes}")
    print(f"[ROCKETSIM] Arena tick rate: {arena.tick_rate:.0f} Hz")
    print(f"[ROCKETSIM] Simulated ticks: {arena.tick_count}")
    print(
        "[ROCKETSIM] Final car position: "
        f"x={coordinates[0]:.2f}, y={coordinates[1]:.2f}, z={coordinates[2]:.2f}"
    )
    print(f"[ROCKETSIM] Remaining boost: {final_state.boost:.2f}")
    print(f"[ROCKETSIM] Flip-reset events observed: {reset_events}")
    print(f"[ROCKETSIM] Has flip reset at end: {final_state.has_flip_reset()}")
    print("[ROCKETSIM] SMOKE TEST PASSED")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
