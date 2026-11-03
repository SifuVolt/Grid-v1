# PARTY CANNON V@ — Python port

A Python/RLBot v5 port of the Party Cannon bot and its RedUtils support library.
The port follows the upstream C# source at the pinned revision below and keeps
its core strategy, interception calculations, vehicle/field models, action
system, shot planning, math utilities, and debug renderer.

- **Upstream:** [NicEastvillage/PartyCannon](https://github.com/NicEastvillage/PartyCannon)
- **Source revision:** `ea84e4325ddb60565bbac6fb9766dc45f38b58cb`
- **Python runtime:** RLBot v5 (`rlbot==2.0.0b56`)
- **Agent entry point:** `main.py`

The C# project and solution files are intentionally replaced by Python
modules and RLBot v5 configuration. See [PORTING_NOTES.md](PORTING_NOTES.md)
for the source-to-Python file map.

## Setup

Create/activate the bot's virtual environment, then install:

```bash
python -m pip install -r requirements.txt
```

Start it through RLBot v5 using `bot.toml`, or directly run:

```bash
python main.py
```

The bot has been syntax-compiled and its pure-Python suite currently passes 10
unit tests covering vectors, field geometry, drive ETA, prediction search,
aerial initialization, renderer API compatibility, and interception helpers. A local Rocket League match has not yet been run; live
behaviour and RLBot installation compatibility still need in-game validation.
