using System;
using RedUtils.Math;

namespace RedUtils
{
public class GroundShot : Shot
	{
		private readonly int _startBoostAmount;

		private readonly float _updateInterval = 0.2f;

		private float _updateTimer;

		private bool _leftGround;

		public override bool Finished { get; internal set; }

		public override bool Interruptible { get; internal set; }

		public override BallSlice Slice { get; internal set; }

		public override Vec3 ShotTarget { get; internal set; }

		public override Vec3 TargetLocation { get; internal set; }

		public override Vec3 ShotDirection { get; internal set; }

		public Arrive ArriveAction { get; internal set; }

		public GroundShot(Car car, BallSlice slice, Vec3 shotTarget)
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
			Surface surface = Field.NearestSurface(Slice.Location);
			Vec3 normal = surface.Normal;
			Ball ball = Slice.ToBall();
			Vec3 vec = (((init ? Slice.Location : TargetLocation) - car.Location).Flatten(normal) / num).Cap(0f, 2300f);
			ball.velocity = (vec * 6f + Slice.Velocity) / 7f;
			Vec3 v = Slice.Location.FlatDirection(ShotTarget);
			float num2 = (vec - Slice.Velocity).Length();
			float time = Slice.Location.FlatDist(ShotTarget) / Utils.Cap(num2 * Utils.ShotPowerModifier(num2) + ball.velocity.Dot(v), 500f, 6000f);
			ball.velocity = (vec + Slice.Velocity * 2f) / 3f;
			ShotDirection = ball.PredictLocation(time).Direction(ShotTarget);
			float x = MathF.Asin(Utils.Cap(((Slice.Location - surface.Limit(Slice.Location)).Dot(normal) - 17f) / 130f, -1f, 1f));
			ShotDirection = ShotDirection.FlatNorm(normal) * MathF.Cos(x) + normal * MathF.Sin(x);
			TargetLocation = Slice.Location - ShotDirection * 130f;
			if (init)
			{
				ArriveAction = new Arrive(car, TargetLocation, ShotDirection.FlatNorm(normal), Slice.Time, allowFlipping: false, 0.1f);
			}
			else
			{
				ArriveAction.Target = TargetLocation;
			}
		}

		public override void Run(RUBot bot)
		{
			float num = Slice.Time - Game.Time;
			ArriveAction.Run(bot);
			Interruptible = ArriveAction.Interruptible;
			_leftGround = _leftGround || !bot.Me.IsGrounded;
			float num2 = ArriveAction.Eta(bot.Me);
			_updateTimer += bot.DeltaTime;
			if (Interruptible && (num < 0f || (_leftGround && bot.Me.IsGrounded) || !ShotValid() || bot.Me.Boost > _startBoostAmount || (num > 0.5f && (!ArriveAction.OnTime || (num2 < num - 0.25f && _updateTimer > _updateInterval)))))
			{
				Finished = true;
			}
			else if (_updateTimer > _updateInterval)
			{
				SetTargetLocation(bot.Me);
				_updateTimer = 0f;
			}
		}

		public override bool IsValid(Car car)
		{
			float num = Slice.Time - Game.Time;
			Surface surface = Field.NearestSurface(TargetLocation);
			float num2 = (Slice.Location - surface.Limit(Slice.Location)).Dot(surface.Normal);
			if (ArriveAction.Eta(car) < num)
			{
				return num2 < 130f;
			}
			return false;
		}
	}
}
