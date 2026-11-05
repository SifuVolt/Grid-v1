# Volt offline RocketSim training

This folder is an **offline training and physics-test harness** for Volt. It is deliberately separate from the live RLBot entry point: importing or running RocketSim here does not change Volt's in-match controller.

## Binding selected

The upstream RocketSim project is C++. Its own README lists the unofficial Python binding at:

- https://github.com/mtheall/RocketSim

This setup pins the binding's `python-dev` implementation to commit
`2da51b1dac7b8127127613a5ff30e490bdd70dd8` so that the binding API does not silently move underneath the training code.

The binding's checked-in `RocketSim.pyi` declares the Python interface used here:
- `rs.init(path)` for loading collision meshes;
- `rs.Arena(game_mode=..., tick_rate=...)`;
- `arena.add_car(team, config)`;
- `CarState`, `BallState`, `CarControls`, `car.set_state()`, `car.set_controls()`;
- `arena.step(ticks)`;
- `CarState.got_flip_reset()` and `CarState.has_flip_reset()` for observing reset state.

This is not the Rust binding from VirxEC/rocketsim-rs. That repository is useful for understanding the upstream lifecycle and assets, but this folder imports the Python module named `RocketSim`.

## 1. Native build prerequisites (Windows)

The Python binding builds a C++20 extension with CMake. Before installing it, install:

1. Visual Studio 2022 Build Tools with **Desktop development with C++** and a Windows SDK.
2. CMake 3.26 or newer.
3. A Python interpreter supported by the binding (Python 3.9+).

Open a Developer Command Prompt once after installing the C++ tools, then check:

```cmd
cmake --version
cl
```

The binding's `pyproject.toml` uses the CMake-based `cmeel` build backend and NumPy 2.x for the build.

## 2. Install into Volt's existing virtual environment

From the Volt repository root, use the same virtual environment that can import Volt's dependencies:

```cmd
venv\Scripts\python.exe -m pip install --upgrade pip
venv\Scripts\python.exe -m pip install "git+https://github.com/mtheall/RocketSim.git@2da51b1dac7b8127127613a5ff30e490bdd70dd8"
```

This dependency is intentionally **not** added to the live bot's root `requirements.txt`. RocketSim is only needed when running the offline tools in this folder.

If the repository's local environment uses a different venv path, replace `venv\Scripts\python.exe` with that interpreter.

## 3. Install the arena collision meshes

RocketSim needs Rocket League arena collision mesh assets. They are not included in this repository.

1. Use ZealanL's collision dumper: https://github.com/ZealanL/RLArenaCollisionDumper
2. Follow that project's instructions to dump the arena collision meshes from your local Rocket League installation.
3. Put the resulting `collision_meshes` directory here:

```text
training/
  rocketsim/
    collision_meshes/
      ...
```

Do not commit the dumped assets. The local `.gitignore` excludes this directory.

Alternatively, set `RS_COLLISION_MESHES` to the absolute path of the directory. The Python binding reads that environment variable when no explicit path is supplied; this smoke test passes the path explicitly.

## 4. Run the binding smoke test

From the repository root:

```cmd
venv\Scripts\python.exe training\rocketsim\smoke_test.py
```

Or pass the asset directory explicitly:

```cmd
venv\Scripts\python.exe training\rocketsim\smoke_test.py --collision-meshes "C:\path\to\collision_meshes"
```

A successful run proves that:
- the native `RocketSim` extension imports;
- the arena collision meshes initialize;
- a Soccar arena and Octane car can be created;
- a car state and controls can be set;
- the simulation advances at 120 ticks per second;
- the binding exposes the flip-reset state methods.

It does **not** claim that Volt can perform a flip reset yet. No flip-reset attempt logic has been added to Volt. This is only the simulator installation/API check.

## 5. Next integration stage

Once this smoke test works on the target Windows machine, the next stage is a controlled adapter between Volt's existing controller and RocketSim:

1. Convert RocketSim car/ball states into the observations Volt's controller expects.
2. Feed Volt's chosen controls into `CarControls` each simulation tick.
3. Sample `got_flip_reset()` every tick and record `has_flip_reset()`, wheel contact, contact geometry, boost, and outcome.
4. Run reproducible scenario batches and compare the results.
5. Only after those results are checked against live Rocket League should any learned parameters or new mechanics be moved into Volt's live routines.

RocketSim's own README explicitly notes that its physics are not perfectly identical to Rocket League and that small errors accumulate. Use short, feedback-driven trials and validate promising results in-game.
