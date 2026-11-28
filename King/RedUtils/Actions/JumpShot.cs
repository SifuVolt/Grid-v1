using System;
using RedUtils.Math;

namespace RedUtils
{
public class JumpShot : Shot
	{
		private float _lastBoostAmount;

		private readonly float _updateInterval = 0.2f;

		private float _updateTimer;

		private bool _jumped;

		private bool _leftGround;

		private float _jumpElapsed;

		private float _latestTouchTime = -1f;

		private int _step;

		private float _dodgeAngle;

		private Surface _jumpedSurface;

		public override bool Finished { get; internal set; }

		public override bool Interruptible { get; internal set; }

		public override BallSlice Slice { get; internal set; }

		public override Vec3 ShotTarget { get; internal set; }

		public override Vec3 TargetLocation { get; internal set; }

		public override Vec3 ShotDirection { get; internal set; }

		public Vec3 DodgeDirection { get; internal set; }

		public Arrive ArriveAction { get; internal set; }

		public JumpShot(Car car, BallSlice slice, Vec3 shotTarget)
		{
			Finished = false;
			Interruptible = true;
			Slice = slice;
			ShotTarget = shotTarget;
			_lastBoostAmount = car.Boost;
			ArriveAction = null;
			SetTargetLocation(car, init: true);
		}

		private void SetTargetLocation(Car car, bool init = false)
		{
			float num = Slice.Time - Game.Time;
			Ball ball = Slice.ToBall();
			Vec3 vec = (((init ? Slice.Location : TargetLocation) - car.Location) / num + (init ? (car.Location.FlatDirection(Slice.Location) * 500f) : (ShotDirection.FlatNorm() * 500f))).Cap(0f, 2300f);
			ball.velocity = (vec * 6f + Slice.Velocity) / 7f;
			Vec3 v = Slice.Location.FlatDirection(ShotTarget);
			float num2 = (vec - Slice.Velocity).Length();
			float time = Slice.Location.FlatDist(ShotTarget) / Utils.Cap(num2 * Utils.ShotPowerModifier(num2) + ball.velocity.Dot(v), 500f, 6000f);
			ball.velocity = (vec + Slice.Velocity * 2f) / 3f;
			ShotDirection = ball.PredictLocation(time).Direction(ShotTarget);
			TargetLocation = Slice.Location - ShotDirection * 165f;
			Surface surface = Field.BestJumpingSurface(car.Location, Slice.Location);
			Vec3 normal = surface.Normal;
			float x = (TargetLocation - surface.Limit(TargetLocation)).Dot(normal);
			float num3 = MathF.Asin(ShotDirection.Dot(normal));
			_dodgeAngle = MathF.Min(MathF.Min(num3 + 0.25f, 0.9424779f), MathF.Max(Utils.PredictRotation(12.46f, Utils.TimeToJump(normal, MathF.Max(x, 100f) - 17f)) + 0.4f, 0.4f));
			num3 = _dodgeAngle - 0.25f;
			ShotDirection = ShotDirection.FlatNorm(normal) * MathF.Cos(num3) + normal * MathF.Sin(num3);
			DodgeDirection = ShotDirection.FlatNorm(normal) * MathF.Cos(_dodgeAngle) + normal * MathF.Sin(_dodgeAngle);
			TargetLocation = Slice.Location - ShotDirection * 165f;
			float num4 = (Slice.Location - surface.Limit(Slice.Location)).Dot(normal);
			x = (TargetLocation - surface.Limit(TargetLocation)).Dot(normal);
			if (x < 30f)
			{
				num3 = MathF.Asin(Utils.Cap((num4 - 30f) / 165f, -1f, 1f));
				ShotDirection = (ShotDirection.FlatNorm(normal) * MathF.Cos(num3) + normal * MathF.Sin(num3)).Normalize();
				DodgeDirection = (ShotDirection.FlatNorm(normal) * MathF.Cos(num3 + 0.25f) + normal * MathF.Sin(num3 + 0.25f)).Normalize();
				TargetLocation = Slice.Location - ShotDirection * 165f;
			}
			float num5 = Utils.TimeToJump(normal, MathF.Max(x, 30f) - 17f);
			if (normal.Dot(Vec3.Up) < 0.95f)
			{
				TargetLocation = TargetLocation.Flatten() + Vec3.Up * (Slice.Location.z + Utils.Cap(TargetLocation.z - Slice.Location.z, -80f, 80f));
				TargetLocation -= Game.Gravity / 2f * MathF.Pow(num5, 2f);
			}
			if (init)
			{
				ArriveAction = new Arrive(car, surface.Limit(TargetLocation), ShotDirection.FlatNorm(normal), Slice.Time, allowFlipping: false, num5);
				return;
			}
			ArriveAction.Target = surface.Limit(TargetLocation);
			ArriveAction.RecoveryTime = num5;
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
				_jumpedSurface = Field.NearestSurface(bot.Me.Location);
				float height = Utils.Cap((TargetLocation - _jumpedSurface.Limit(TargetLocation)).Dot(_jumpedSurface.Normal) - 17f, 1f, 270f);
				float num2 = MathF.Max(Utils.TimeToJump(_jumpedSurface.Normal, height), 0.15f);
				float num3 = ArriveAction.Eta(bot.Me);
				_updateTimer += bot.DeltaTime;
				if (Interruptible && (num < num2 - 0.025f || (_leftGround && bot.Me.IsGrounded) || !ShotValid() || bot.Me.Boost > _lastBoostAmount || !ArriveAction.OnTime || (num3 < num - 0.25f && _updateTimer > _updateInterval)))
				{
					Finished = true;
				}
				else if (_updateTimer > _updateInterval && num > num2 + _updateInterval)
				{
					SetTargetLocation(bot.Me);
					_updateTimer = 0f;
				}
				else if (ReadyToJump(bot.Me, num2))
				{
					_jumped = true;
				}
			}
			else
			{
				Interruptible = false;
				_jumpElapsed += bot.DeltaTime;
				Vec3 location = bot.Me.LocationAfterJump(0.0333f, _jumpElapsed);
				Vec3 vec = bot.Me.LocationAfterJump(num, _jumpElapsed);
				Vec3 vec2 = TargetLocation - vec + 0f * Game.Gravity * MathF.Pow(num, 2f);
				Car car = new Car(bot.Me)
				{
					Location = location
				};
				Vec3 pos = ((Ball.Prediction[1] != null) ? Ball.Prediction[1].Location : Ball.Location);
				bool flag = car.Hitbox.IsTouching(pos);
				Vec3 vec3 = location.Direction(Slice.Location);
				if (!bot.Me.IsGrounded)
				{
					bot.AimAt(bot.Me.Location + DodgeDirection, DodgeDirection.Cross(DodgeDirection.Cross(-_jumpedSurface.Normal)).Normalize(), backwards: false);
					bot.Controller.Boost = vec2.Dot(bot.Me.Forward) / num >= 1058.334f * MathF.Max(bot.DeltaTime, 13f / 120f) && vec2.Normalize().Dot(bot.Me.Forward) > 0.25f;
					bot.Controller.Throttle = ((vec2.Normalize().Dot(bot.Me.Forward) > 0.5f) ? 1 : 0);
				}
				if (num < -0.1f || (num > 0.4f && !ShotValid()) || (Ball.LatestTouch != null && _latestTouchTime != Ball.LatestTouch.Time && Ball.LatestTouch.PlayerIndex != bot.Index && Ball.Location.Dist(vec) > 300f))
				{
					Finished = true;
				}
				else if ((num > 0.075f && !flag) || _jumpElapsed < 0.025f)
				{
					bot.Controller.Jump = true;
				}
				else if (_step < 3 || (num > 0.05f && !flag))
				{
					bot.Controller.Jump = false;
					_step++;
				}
				else
				{
					Vec3 direction;
					if (vec3.FlatAngle(ShotDirection) > 0.8f)
					{
						direction = ((vec3.FlatAngle(ShotDirection) > 2f) ? vec3.FlatNorm() : (vec3.FlatNorm() + ShotDirection.FlatNorm()).Normalize());
					}
					else
					{
						direction = ShotDirection.FlatNorm();
					}
					bot.Action = new Dodge(direction, 0.1f, willJump: true);
				}
			}
			_lastBoostAmount = bot.Me.Boost;
		}

		public override bool IsValid(Car car)
		{
			float num = Slice.Time - Game.Time;
			Surface surface = Field.BestJumpingSurface(car.Location, TargetLocation);
			float num2 = MathF.Max((TargetLocation - surface.Limit(TargetLocation)).Dot(surface.Normal), 20f);
			float num3 = Utils.TimeToJump(surface.Normal, num2 - 17f);
			if (ArriveAction.Eta(car) < num && num2 < 270f && num > num3)
			{
				return Field.InField(TargetLocation, 50f);
			}
			return false;
		}

		private bool ReadyToJump(Car car, float timeToJump)
		{
			float num = Slice.Time - Game.Time;
			Vec3 vec = car.LocationAfterJump(num, 0f) - 0f * Game.Gravity * MathF.Pow(num, 2f);
			Vec3 normal = Field.NearestSurface(car.Location).Normal;
			bool flag = (vec.Dist(Slice.Location) < 180f || vec.FlatDist(TargetLocation) < 50f || vec.Length() < MathF.Min(num, (float)car.Boost / 33.3f) * 991.667f) && vec.Direction(Slice.Location).Dot(ShotDirection.FlatNorm(normal)) > -0.2f;
			return (num < timeToJump + 0.025f && !Field.OnSeam(car.Location, 100f) && Interruptible) & flag;
		}
	}
}
