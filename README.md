
# Grid v1

Grid is a standalone Rocket League bot for the RLBot v5 Python interface.
It does **not** use VirxERLU and does not share Volt's code or routines.

## Requirements

- Python 3.12 or newer
- RLBot v5 / the RLBot Python interface
- Rocket League and RLBot

## Setup (Windows)

```powershell
py -3.12 -m venv ..\venv
..\venv\Scripts\python.exe -m pip install --upgrade pip
..\venv\Scripts\python.exe -m pip install -r requirements.txt
```

The `bot.toml` run command expects a virtual environment one directory above
this project, matching the common RLBot bot-folder layout. Change that command
if your environment is elsewhere.

## Start

Launch Grid through your RLBot v5 GUI / match configuration using `bot.toml`.
For a direct process smoke test, run:

```powershell
..\venv\Scripts\python.exe main.py
```

The direct process expects the RLBot runner to provide the v5 connection.

## Architecture

- `util/agent.py`: RLBot adapter, state assembly, tactical decision loop
- `util/tools.py`: state models, prediction parsing, arrival estimates,
  intercept evaluation, role and boost selection
- `util/routines.py`: ground driving and persistent mechanic controllers
- `util/utils.py`: vector math, field geometry and coordinate transforms
- `match_comms.py`: versioned, defensive team-message codec
- `gui.py`: telemetry formatting helpers

## Current v1 status

This is a substantial first playable foundation, not a claim that every named
mechanic is already competition-grade. The current implementation includes
ground navigation, prediction-based intercept ranking, basic role assignment,
shadow positioning, boost routing, kickoff phases, jump-shot and aerial
controllers, and a conservative wave-dash trigger.

Advanced dribbling, flick selection, air-dribble continuation, wall reads,
backboard prediction, speed-flip calibration, robust save trajectories,
passing lanes, and full multi-agent coordination are planned follow-on systems.
They need match testing and tuning rather than placeholder code or inflated
line counts.

## Debugging

Grid logs a compact tactical line once per second through the RLBot logger.
The bot deliberately avoids printing a full diagnostic dump every frame.
