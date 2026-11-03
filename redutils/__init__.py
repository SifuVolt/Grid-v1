"""Party Cannon RedUtils Python port."""
from .math3d import Vec3, Mat3x3
from .objects import Game, Field, Surface, Goal, Boost, Ball, BallSlice, BallPrediction, BallTouch, Car, Cars, Hitbox, Target
from .actions import IAction, Shot, Drive, Arrive, Dodge, HalfFlip, SpeedFlip, Wavedash, GetBoost, GroundShot, JumpShot, DoubleJumpShot, AerialShot, QuickShot, Kickoff

try:
    from .bot import RUBot
except ModuleNotFoundError as exc:
    if exc.name not in {"rlbot", "rlbot.flat", "rlbot.managers"}:
        raise
    RUBot = None  # Runtime adapter requires the RLBot v5 Python package.
