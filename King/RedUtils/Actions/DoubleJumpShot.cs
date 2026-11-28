using System;
using RedUtils.Math;

namespace RedUtils
{
public class DoubleJumpShot : Shot
	{
		private readonly int _startBoostAmount;

		private readonly float _updateInterval = 0.2f;

		private float _updateTimer;

		private bool _jumped;

		private bool _leftGround;

		private float _jumpElapsed;

		private float _latestTouchTime = -1f;

		private int _step;

		public override bool Finished { get; internal set; }

		public override bool Interruptible { get; internal set; }

		public override BallSlice Slice { get; internal set; }

		public override Vec3 ShotTarget { get; internal set; }

		public override Vec3 TargetLocation { get; internal set; }

		public override Vec3 ShotDirection { get; internal set; }

		public Arrive ArriveAction { get; internal set; }

		public DoubleJumpShot(Car car, BallSlice slice, Vec3 shotTarget)
		{
			Finished = false;
			Interruptible = true;
			Slice = slice;
			ShotTarget = shotTarget;
			_startBoostAmount = car.Boost;
			SetTargetLocation(car, init: true);
		}

		private void SetTargetLocation(Car car, bool init = false)
		{
			float num = Slice.Time - Game.Time;
			Ball ball = Slice.ToBall();
			Vec3 vec = (((init ? Slice.Location : TargetLocation) - car.Location) / num).Cap(0f, 2300f);
			ball.velocity = (vec * 6f + Slice.Velocity) / 7f;
			Vec3 v = Slice.Location.FlatDirection(ShotTarget);
			float num2 = (vec - Slice.Velocity).Length();
			float time = Slice.Location.FlatDist(ShotTarget) / Utils.Cap(num2 * Utils.ShotPowerModifier(num2) + ball.velocity.Dot(v), 500f, 6000f);
			ball.velocity = (vec + Slice.Velocity * 2f) / 3f;
			ShotDirection = ball.PredictLocation(time).Direction(ShotTarget);
			float x = MathF.Min(MathF.Asin(ShotDirection.z), 1.0995574f);
			ShotDirection = ShotDirection.FlatNorm() * MathF.Cos(x) + Vec3.Up * MathF.Sin(x);
			TargetLocation = Slice.Location - ShotDirection * 160f;
			Surface surface = Field.NearestSurface(Slice.Location);
			Vec3 normal = surface.Normal;
			float num3 = surface.Limit(TargetLocation).Dist(TargetLocation);
			if (num3 < 50f)
			{
				x = MathF.Asin(Utils.Cap((num3 - 50f) / 160f, -1f, 1f));
				ShotDirection = ShotDirection.FlatNorm(normal) * MathF.Cos(x) + normal * MathF.Sin(x);
				TargetLocation = Slice.Location - ShotDirection * 160f;
			}
			if (init)
			{
				ArriveAction = new Arrive(car, TargetLocation.Flatten(), Slice.Time)
				{
					AllowFlipping = false,
					RecoveryTime = Utils.TimeToJump(Vec3.Up, TargetLocation.z - 17f, doubleJump: true)
				};
			}
			else
			{
				ArriveAction.Target = TargetLocation.Flatten();
				ArriveAction.RecoveryTime = Utils.TimeToJump(Vec3.Up, TargetLocation.z - 17f, doubleJump: true);
			}
		}

		public override void Run(RUBot bot)
		{
			float num = Slice.Time - Game.Time;
			if (_latestTouchTime < 0f && Ball.LatestTouch != null)
			{
				_latestTouchTime = Ball.LatestTouch.Time;
			}
			if (!_jumped)
			{
				ArriveAction.Run(bot);
				Interruptible = ArriveAction.Interruptible;
				_leftGround = _leftGround || !bot.Me.IsGrounded;
				float num2 = MathF.Max(Utils.TimeToJump(Vec3.Up, TargetLocation.z - 17f, doubleJump: true), 0.2f);
				float num3 = ArriveAction.Eta(bot.Me);
				_updateTimer += bot.DeltaTime;
				if (Interruptible && (num < num2 - 0.05f || (_leftGround && bot.Me.IsGrounded) || !ShotValid() || bot.Me.Boost > _startBoostAmount || (_updateTimer > _updateInterval && !ArriveAction.OnTime) || (num3 < num - 0.1f && bot.Me.Boost > 5 && _updateTimer > _updateInterval)))
				{
					Finished = true;
					return;
				}
				if (_updateTimer > _updateInterval && num > num2)
				{
					SetTargetLocation(bot.Me);
					_updateTimer = 0f;
					return;
				}
				Vec3 vec = bot.Me.LocationAfterDoubleJump(num, 0f);
				if (num < num2 && Interruptible && !Field.OnSeam(bot.Me.Location, 100f) && vec.FlatDist(TargetLocation) < 80f && vec.Dist(Slice.Location) < 165f)
				{
					_jumped = true;
				}
				return;
			}
			Interruptible = false;
			_jumpElapsed += bot.DeltaTime;
			if (num < 0f || (Ball.LatestTouch != null && _latestTouchTime != Ball.LatestTouch.Time && Ball.LatestTouch.PlayerIndex != bot.Index && Ball.Location.Dist(bot.Me.Location) > 200f))
			{
				Finished = true;
				return;
			}
			if (_jumpElapsed < 0.2f)
			{
				bot.Controller.Jump = true;
				return;
			}
			if (_step < 3)
			{
				bot.Controller.Jump = false;
				_step++;
				return;
			}
			if (_step < 6)
			{
				bot.Controller.Jump = true;
				_step++;
				return;
			}
			Vec3 vec2 = bot.Me.PredictLocation(num);
			Vec3 vec3 = TargetLocation - vec2;
			Vec3 vec4 = vec2.Direction(Slice.Location);
			bot.Controller.Boost = vec3.Dot(bot.Me.Forward) / num >= 1058.334f * MathF.Max(bot.DeltaTime, 13f / 120f) && vec3.Normalize().Dot(bot.Me.Forward) > 0.75f;
			bot.Controller.Throttle = ((vec3.Normalize().Dot(bot.Me.Forward) > 0.5f) ? 1 : 0);
			bot.AimAt(bot.Me.Location + vec4);
		}

		public override bool IsValid(Car car)
		{
			float num = Slice.Time - Game.Time;
			if (ArriveAction.Eta(car) < num && TargetLocation.z >= 300f && TargetLocation.z < 510f)
			{
				return num > Utils.TimeToJump(Vec3.Up, TargetLocation.z - 17f, doubleJump: true);
			}
			return false;
		}
	}
}
