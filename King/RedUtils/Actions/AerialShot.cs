using System;
using RedUtils.Math;

namespace RedUtils
{
public class AerialShot : Shot
	{
		private readonly bool _startedOnGround;

		private readonly int _startBoostAmount;

		private bool _currentlyDoubleJumping;

		private bool _aerialing;

		private bool _jumped;

		private float _elapsedTime;

		private int _step;

		public override bool Finished { get; internal set; }

		public override bool Interruptible { get; internal set; }

		public override BallSlice Slice { get; internal set; }

		public override Vec3 ShotTarget { get; internal set; }

		public override Vec3 TargetLocation { get; internal set; }

		public override Vec3 ShotDirection { get; internal set; }

		public bool DoubleJumping { get; internal set; }

		public AerialShot(Car car, BallSlice slice, Vec3 shotTarget)
		{
			Finished = false;
			Interruptible = true;
			Slice = slice;
			ShotTarget = shotTarget;
			_startBoostAmount = car.Boost;
			_startedOnGround = car.IsGrounded;
			SetTargetLocation(car);
			CanHit(car, out var doubleJumping);
			DoubleJumping = doubleJumping;
		}

		private void SetTargetLocation(Car car)
		{
			float num = Slice.Time - Game.Time;
			Ball ball = Slice.ToBall();
			Vec3 vec = ((Slice.Location - car.Location) / num).Cap(0f, 2300f);
			ball.velocity = (vec * 6f + Slice.Velocity) / 7f;
			Vec3 v = Slice.Location.FlatDirection(ShotTarget);
			float num2 = (vec - Slice.Velocity).Length();
			float time = Slice.Location.FlatDist(ShotTarget) / Utils.Cap(num2 + ball.velocity.Dot(v), 500f, 6000f);
			ball.velocity = (vec + Slice.Velocity * 2f) / 3f;
			ShotDirection = ball.PredictLocation(time).Direction(ShotTarget);
			TargetLocation = Slice.Location - ShotDirection * 155f;
			Vec3 normal = Field.NearestSurface(TargetLocation).Normal;
			float num3 = (TargetLocation - Field.LimitToNearestSurface(TargetLocation)).Dot(normal);
			if (num3 < 50f)
			{
				float x = MathF.Asin(Utils.Cap((num3 - 50f) / 155f, -1f, 1f));
				ShotDirection = (ShotDirection.FlatNorm(normal) * MathF.Cos(x) + normal * MathF.Sin(x)).Normalize();
				TargetLocation = Slice.Location - ShotDirection * 155f;
			}
		}

		public override void Run(RUBot bot)
		{
			float num = Slice.Time - Game.Time;
			Interruptible = false;
			_elapsedTime += bot.DeltaTime;
			_currentlyDoubleJumping = false;
			if (!_jumped)
			{
				if (_elapsedTime <= 0.2f)
				{
					bot.Controller.Jump = true;
				}
				else if (_step < 3 && DoubleJumping)
				{
					bot.Controller.Jump = false;
					_step++;
				}
				else if (_step < 6 && DoubleJumping)
				{
					bot.Controller.Jump = true;
					_currentlyDoubleJumping = true;
					_step++;
				}
				else
				{
					_jumped = true;
				}
			}
			Vec3 vec;
			if (_jumped)
			{
				vec = bot.Me.PredictLocation(num);
			}
			else
			{
				vec = (DoubleJumping ? bot.Me.LocationAfterDoubleJump(num, _elapsedTime) : bot.Me.LocationAfterJump(num, _elapsedTime));
			}
			Vec3 vec2 = TargetLocation - vec;
			Vec3 vec3 = Slice.Location + Slice.Location.Direction(vec) * MathF.Min(155f, Slice.Location.Dist(vec)) - vec;
			float num2 = 2f * vec3.Length() / MathF.Pow(num, 2f);
			if (num2 > 1058.334f)
			{
				vec2 = Slice.Location - vec.Direction(Slice.Location) * 160f - vec;
				num2 = 2f * vec2.Length() / MathF.Pow(num, 2f);
			}
			bot.AimAt(bot.Me.Location + vec2, _jumped ? bot.Me.Location.Direction(Slice.Location) : Vec3.Up);
			bot.Controller.Boost = vec2.Dot(bot.Me.Forward) / num >= 1058.334f * MathF.Max(bot.DeltaTime, 13f / 120f) && vec2.Angle(bot.Me.Forward) < ((bot.Me.Boost > 10) ? 0.4f : 0.3f);
			bot.Controller.Throttle = Utils.Cap(vec2.Dot(bot.Me.Forward) / num / (66.667f * MathF.Max(bot.DeltaTime, 1f / 120f)), -1f, 1f);
			if (_currentlyDoubleJumping)
			{
				bot.Controller.Steer = 0f;
				bot.Controller.Yaw = 0f;
				bot.Controller.Pitch = 0f;
				bot.Controller.Roll = 0f;
			}
			if (num <= 0f || (_jumped && vec3.Length() > 50f && num > 0.8f && num2 * 0.9f > 66.667f && (bot.Me.Boost == 0 || num2 * 0.95f > 1058.334f)) || (!ShotValid() && num > 0.3f) || (bot.Me.IsGrounded && _jumped))
			{
				Finished = true;
			}
			else if (((!DoubleJumping && _elapsedTime < 1.45f) || (!bot.Me.HasJumped && _jumped)) && num < 0.05f && vec2.FlatLen() < 50f && ShotDirection.z < 0.7f && _startedOnGround)
			{
				bot.Action = new Dodge(ShotDirection.FlatNorm(), 0.1f);
			}
			else if (vec2.Length() < 50f && !_currentlyDoubleJumping)
			{
				bot.AimAt(bot.Me.Location + ShotDirection, bot.Me.Location.Direction(Slice.Location));
			}
		}

		public override bool IsValid(Car car)
		{
			if (TargetLocation.z > 300f)
			{
				return CanHit(car);
			}
			return false;
		}

		private bool CanHit(Car car)
		{
			float boostEstimate = GetBoostEstimate(car, doubleJumping: false);
			float boostEstimate2 = GetBoostEstimate(car, doubleJumping: true);
			if (!(boostEstimate >= 0f))
			{
				return boostEstimate2 >= 0f;
			}
			return true;
		}

		private bool CanHit(Car car, out bool doubleJumping)
		{
			float boostEstimate = GetBoostEstimate(car, doubleJumping: false);
			float boostEstimate2 = GetBoostEstimate(car, doubleJumping: true);
			doubleJumping = boostEstimate2 >= 0f && (boostEstimate2 < boostEstimate || boostEstimate < 0f);
			if (!(boostEstimate >= 0f))
			{
				return boostEstimate2 >= 0f;
			}
			return true;
		}

		private float GetBoostEstimate(Car car, bool doubleJumping)
		{
			float num = Slice.Time - Game.Time;
			Vec3 vec;
			if (car.IsGrounded)
			{
				vec = (doubleJumping ? car.LocationAfterDoubleJump(num, 0f) : car.LocationAfterJump(num, 0f));
			}
			else
			{
				vec = car.PredictLocation(num);
			}
			Vec3 vec2;
			if (car.IsGrounded)
			{
				vec2 = (doubleJumping ? car.VelocityAfterDoubleJump(num, 0f) : car.VelocityAfterJump(num, 0f));
			}
			else
			{
				vec2 = car.PredictVelocity(num);
			}
			_ = Slice.Location + Slice.Location.Direction(vec) * MathF.Min(155f, Slice.Location.Dist(vec));
			Vec3 vec3 = TargetLocation - vec;
			Vec3 vec4 = vec3.Normalize();
			float num2 = vec4.Angle(car.Forward);
			num2 = Utils.Cap(num2, 0.0001f, num2);
			float num3 = (car.IsGrounded ? 0.6f : 0.1f) * (2f * MathF.Sqrt(num2 / 9f)) * Utils.Cap(1f - 0.4f / num2, 0f, 1f);
			float num4 = 2f * vec3.Length() / MathF.Pow(num - num3, 2f) / 1058.334f;
			float num5 = num - (num - num3) * MathF.Sqrt(1f - Utils.Cap(num4, 0f, 1f));
			Vec3 vec5 = vec2 + 1058.334f * (num5 - num3) * vec4;
			float num6 = (num5 - num3) * 33.3f;
			bool flag = num6 < (float)car.Boost * 0.9f;
			bool flag2 = MathF.Abs(num4) < 0.9f;
			if (!((vec5.Length() < 2070f) & flag & flag2))
			{
				return -1f;
			}
			return num6;
		}
	}
}
