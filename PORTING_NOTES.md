# Party Cannon C# → Python source map

All 36 C# source files in upstream revision
`ea84e4325ddb60565bbac6fb9766dc45f38b58cb` have Python implementations. To keep
related types together in Python, several original C# files are consolidated
into modules; the original behaviour and type responsibilities are mapped as
follows.

| Upstream C# source | Python destination |
|---|---|
| `Bot/Bot.cs` | `party_bot.py` — `PartyBot` strategy |
| `Bot/InterceptCalculator.cs` | `intercept_calculator.py` — `InterceptCalculator` |
| `Bot/InterceptInfo.cs` | `intercept_calculator.py` — `InterceptInfo` |
| `Bot/Program.cs` | `main.py` — RLBot v5 entry point |
| `RedUtils/Actions/Action.cs` | `redutils/actions.py` — `IAction` |
| `RedUtils/Actions/AerialShot.cs` | `redutils/actions.py` — `AerialShot` |
| `RedUtils/Actions/Arrive.cs` | `redutils/actions.py` — `Arrive` |
| `RedUtils/Actions/Dodge.cs` | `redutils/actions.py` — `Dodge` |
| `RedUtils/Actions/DoubleJumpShot.cs` | `redutils/actions.py` — `DoubleJumpShot` |
| `RedUtils/Actions/Drive.cs` | `redutils/actions.py` — `Drive` |
| `RedUtils/Actions/GetBoost.cs` | `redutils/actions.py` — `GetBoost` |
| `RedUtils/Actions/GroundShot.cs` | `redutils/actions.py` — `GroundShot` |
| `RedUtils/Actions/HalfFlip.cs` | `redutils/actions.py` — `HalfFlip` |
| `RedUtils/Actions/JumpShot.cs` | `redutils/actions.py` — `JumpShot` |
| `RedUtils/Actions/Kickoff.cs` | `redutils/actions.py` — `Kickoff` |
| `RedUtils/Actions/QuickShot.cs` | `redutils/actions.py` — `QuickShot` |
| `RedUtils/Actions/Shot.cs` | `redutils/actions.py` — `Shot` |
| `RedUtils/Actions/SpeedFlip.cs` | `redutils/actions.py` — `SpeedFlip` |
| `RedUtils/Actions/Wavedash.cs` | `redutils/actions.py` — `Wavedash` |
| `RedUtils/Bot.cs` | `redutils/bot.py` — `RUBot` RLBot v5 lifecycle |
| `RedUtils/ExtendedRenderer.cs` | `redutils/renderer.py` — `ExtendedRenderer` |
| `RedUtils/Math/Mat3x3.cs` | `redutils/math3d.py` — `Mat3x3` |
| `RedUtils/Math/Vec3.cs` | `redutils/math3d.py` — `Vec3` |
| `RedUtils/Objects/Ball.cs` | `redutils/objects.py` — `Ball` |
| `RedUtils/Objects/BallPrediction.cs` | `redutils/objects.py` — `BallPrediction` |
| `RedUtils/Objects/BallSlice.cs` | `redutils/objects.py` — `BallSlice` |
| `RedUtils/Objects/BallTouch.cs` | `redutils/objects.py` — `BallTouch` |
| `RedUtils/Objects/Boost.cs` | `redutils/objects.py` — `Boost` |
| `RedUtils/Objects/Car.cs` | `redutils/objects.py` — `Car` |
| `RedUtils/Objects/Cars.cs` | `redutils/objects.py` — `Cars` |
| `RedUtils/Objects/Field.cs` | `redutils/objects.py` — `Field` and `Surface` |
| `RedUtils/Objects/Game.cs` | `redutils/objects.py` — `Game` |
| `RedUtils/Objects/Goal.cs` | `redutils/objects.py` — `Goal` |
| `RedUtils/Objects/Hitbox.cs` | `redutils/objects.py` — `Hitbox` |
| `RedUtils/Objects/Target.cs` | `redutils/objects.py` — `Target` |
| `RedUtils/Tools.cs` | `redutils/tools.py` — `ToolsMixin` |
| `RedUtils/Utils.cs` | `redutils/utils.py` — physics/math helpers |

## Port notes

- RLBot v4/C# packet and controller access is adapted to RLBot v5's Python
  `GamePacket`, `ControllerState`, `Bot`, and renderer interface.
- The original RedUtils action set and Party Cannon strategy are retained.
- The original .NET solution/project files and C# launch command are replaced
  with `main.py`, `requirements.txt`, and a Python `bot.toml`.
- The upstream MIT license notice is retained verbatim in `LICENSE`.
- The upstream logo image and .NET-only solution/project files are not needed
  for the Python agent runtime and are not included in this port branch.
