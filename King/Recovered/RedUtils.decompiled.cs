using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Numerics;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.Versioning;
using RLBotDotNet;
using RLBotDotNet.Renderer;
using RedUtils.Math;
using rlbot.flat;

[assembly: CompilationRelaxations(8)]
[assembly: RuntimeCompatibility(WrapNonExceptionThrows = true)]
[assembly: Debuggable(DebuggableAttribute.DebuggingModes.IgnoreSymbolStoreSequencePoints)]
[assembly: TargetFramework(".NETCoreApp,Version=v6.0", FrameworkDisplayName = "")]
[assembly: AssemblyCompany("RedUtils")]
[assembly: AssemblyConfiguration("Release")]
[assembly: AssemblyFileVersion("1.0.0.0")]
[assembly: AssemblyInformationalVersion("1.0.0")]
[assembly: AssemblyProduct("RedUtils")]
[assembly: AssemblyTitle("RedUtils")]
[assembly: AssemblyVersion("1.0.0.0")]
namespace RedUtils
{
	public interface IAction
	{
		bool Finished { get; }

		bool Interruptible { get; }

		void Run(RUBot bot);
	}
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
	public class Arrive : IAction
	{
		public Vec3 Target;

		public Vec3 Direction;

		public float ArrivalTime;

		public bool AllowFlipping;

		public float RecoveryTime;

		public Drive Drive;

		public bool OnTime;

		public bool Finished { get; private set; }

		public bool Interruptible { get; private set; }

		public float TimeRemaining { get; private set; }

		public Arrive(Car car, Vec3 target, Vec3 direction)
		{
			Interruptible = true;
			Finished = false;
			Target = target;
			Direction = direction;
			ArrivalTime = -1f;
			AllowFlipping = true;
			Drive = new Drive(car, Target, 2300f, AllowFlipping, wasteBoost: false);
			RecoveryTime = 0f;
		}

		public Arrive(Car car, Vec3 target, float arrivalTime)
		{
			Interruptible = true;
			Finished = false;
			Target = target;
			Direction = Vec3.Zero;
			ArrivalTime = arrivalTime;
			AllowFlipping = true;
			Drive = new Drive(car, Target, 2300f, AllowFlipping, wasteBoost: true);
			RecoveryTime = 0f;
		}

		public Arrive(Car car, Vec3 target, Vec3 direction, float arrivalTime)
		{
			Interruptible = true;
			Finished = false;
			Target = target;
			Direction = direction;
			ArrivalTime = arrivalTime;
			AllowFlipping = true;
			Drive = new Drive(car, Target, 2300f, AllowFlipping, wasteBoost: true);
			RecoveryTime = 0f;
		}

		public Arrive(Car car, Vec3 target, Vec3 direction, float arrivalTime, bool allowFlipping)
		{
			Interruptible = true;
			Finished = false;
			Target = target;
			Direction = direction;
			ArrivalTime = arrivalTime;
			AllowFlipping = allowFlipping;
			Drive = new Drive(car, Target, 2300f, AllowFlipping, wasteBoost: true);
			RecoveryTime = 0f;
		}

		public Arrive(Car car, Vec3 target, Vec3 direction, float arrivalTime, bool allowFlipping, float recoveryTime)
		{
			Interruptible = true;
			Finished = false;
			Target = target;
			Direction = direction;
			ArrivalTime = arrivalTime;
			AllowFlipping = allowFlipping;
			Drive = new Drive(car, Target, 2300f, AllowFlipping, wasteBoost: true);
			RecoveryTime = recoveryTime;
		}

		public void Run(RUBot bot)
		{
			float num = Distance(bot.Me);
			float num2 = bot.Me.Velocity.Length();
			TimeRemaining = ((ArrivalTime < 0f) ? (num / 2300f) : MathF.Max(ArrivalTime - Game.Time, 0.001f));
			float num3 = TimeRemaining - Eta(bot.Me);
			Vec3 target;
			Vec3 vec2;
			if (Direction.Length() > 0f)
			{
				Vec3 normal = Field.NearestSurface(Target).Normal;
				Vec3 vec = bot.Me.Location.FlatDirection(Target, normal);
				float num4 = MathF.Min(Field.DistanceBetweenPoints(Target, bot.Me.Location) * 0.4f, Utils.Cap(num2, 1410f, 2300f) * 0.8f);
				float num5 = Utils.Cap(num2, 500f, 2300f);
				float num6 = Drive.TurnRadius(num5);
				float num7 = num6 * bot.Me.Forward.FlatAngle(Target, bot.Me.Up) / num5;
				bool flag = Drive.TurnRadius(bot.Me, Target) - 100f < num6;
				num4 *= (((TimeRemaining > RecoveryTime + num7 + 0.1f && ((!flag && num3 > 0.1f) || vec.Angle(Direction) > 1.6f)) || ArrivalTime < 0f) ? Utils.Cap(num4 / (num6 * 1.5f), 0f, 1f) : 0f);
				Vec3 start = vec.Cross(normal).Normalize();
				Vec3 end = vec.Cross(-normal).Normalize();
				target = Field.LimitToNearestSurface(Target - Direction.Clamp(start, end, normal).Normalize() * num4);
				vec2 = Field.LimitToNearestSurface(Target - Direction.Clamp(start, end, normal).Normalize() * num4 * 0.5f);
			}
			else
			{
				target = Target;
				vec2 = Target;
			}
			float num8 = bot.Me.Location.FlatDist(Target) / MathF.Max(num2 + 500f, 1410f);
			float targetSpeed = (Drive.GetDistance(bot.Me, vec2) + Field.DistanceBetweenPoints(vec2, Target)) / TimeRemaining;
			Vec3 vec3 = bot.Me.LocationAfterSpeedFlip(bot.Me.Boost > 0);
			Drive.AllowDodges = MathF.Sign(vec3.FlatDirection(Target).Dot(Direction.Cross())) == MathF.Sign(bot.Me.Location.FlatDirection(Target).Dot(Direction.Cross())) && num8 > 1.4f + RecoveryTime;
			Drive.Target = target;
			Drive.TargetSpeed = targetSpeed;
			Drive.Run(bot);
			OnTime = ArrivalTime < 0f || MathF.Max(TimeRemaining * 1.05f, TimeRemaining + 0.1f) > Eta(bot.Me);
			Interruptible = Drive.Interruptible;
			if (Field.LimitToNearestSurface(bot.Me.Location).Dist(Field.LimitToNearestSurface(Target)) < 100f || (ArrivalTime < Game.Time && ArrivalTime > 0f))
			{
				Finished = true;
			}
		}

		public float Distance(Car car)
		{
			return Drive.GetDistance(car, Target);
		}

		public float Eta(Car car)
		{
			return Drive.GetEta(car, Target, Drive.Backwards, AllowFlipping);
		}
	}
	public class Dodge : IAction
	{
		public bool WillJump = true;

		public bool CancelFlip;

		private bool _jumping = true;

		private float _startTime = -1f;

		private Vec3 _input = Vec3.Zero;

		private int _step;

		public bool Finished { get; private set; }

		public bool Interruptible { get; private set; }

		public Vec3 Direction { get; set; }

		public float JumpTime { get; set; }

		public float Duration => JumpTime + 1.1f;

		public Dodge(Vec3 direction)
		{
			Interruptible = false;
			Finished = false;
			Direction = direction;
			JumpTime = 0.1f;
		}

		public Dodge(Vec3 direction, float jumpTime)
		{
			Interruptible = false;
			Finished = false;
			Direction = direction;
			JumpTime = jumpTime;
		}

		public Dodge(Vec3 direction, float jumpTime, bool willJump)
		{
			Interruptible = false;
			Finished = false;
			Direction = direction;
			JumpTime = jumpTime;
			WillJump = willJump;
		}

		public Dodge(Vec3 direction, float jumpTime, bool willJump, bool cancelFlip)
		{
			Interruptible = false;
			Finished = false;
			Direction = direction;
			JumpTime = jumpTime;
			WillJump = willJump;
			CancelFlip = cancelFlip;
		}

		public static Vec3 GetDodgeImpulse(Mat3x3 orientation, Vec3 vel, Vec3 dodgeInput)
		{
			float num = vel.Dot(orientation.Forward);
			float num2 = MathF.Abs(num) / 2300f;
			dodgeInput = new Vec3(0f - dodgeInput[1], dodgeInput[0]);
			dodgeInput[0] = ((MathF.Abs(dodgeInput[0]) < 0.1f) ? 0f : dodgeInput[0]);
			dodgeInput[1] = ((MathF.Abs(dodgeInput[1]) < 0.1f) ? 0f : dodgeInput[1]);
			bool flag = ((!(num2 * 2300f < 100f)) ? (dodgeInput[0] >= 0f != num > 0f) : (dodgeInput[0] < 0f));
			Vec3 vec = 500f * dodgeInput;
			vec[0] *= (flag ? (1.0666667f * (1f + 1.5f * num2)) : 1f);
			vec[1] *= 1f + 0.9f * num2;
			vec = orientation.Forward.FlatNorm() * vec[0] - orientation.Forward.Flatten().Cross(Vec3.Up).Normalize() * vec[1];
			return (vel + vec).Cap(0f, 2300f) - vel;
		}

		public static Vec3 GetDodgeInput(Car car, Vec3 direction)
		{
			Vec3 vec = new Vec3(car.Forward.FlatNorm().Dot(direction), 0f - car.Forward.FlatNorm().Cross().Dot(direction));
			float num = car.Forward.Dot(car.Velocity);
			float num2 = MathF.Abs(num) / 2300f;
			bool flag = ((MathF.Abs(num) < 100f) ? (vec[0] < 0f) : (vec[0] >= 0f != num > 0f));
			vec[0] /= (flag ? (1.0666667f * (1f + 1.5f * num2)) : 1f);
			vec[1] /= 1f + 0.9f * num2;
			vec = vec.Normalize();
			return new Vec3(vec[1], 0f - vec[0]);
		}

		public void Run(RUBot bot)
		{
			if (_startTime == -1f)
			{
				_startTime = Game.Time;
				_jumping = bot.Me.IsGrounded && WillJump;
			}
			float num = Game.Time - _startTime;
			if (bot.Me.IsGrounded && num > (_jumping ? JumpTime : 0f) + 0.1f)
			{
				Finished = true;
			}
			else if (num < JumpTime && _jumping)
			{
				bot.Controller.Jump = true;
			}
			else if (_step < 3 && _jumping)
			{
				bot.Controller.Jump = false;
				_step++;
			}
			else if (num < (_jumping ? JumpTime : 0f) + 0.1f)
			{
				if (_input.Length() == 0f)
				{
					_input = GetDodgeInput(bot.Me, Direction);
				}
				bot.Controller.Yaw = _input[0];
				bot.Controller.Pitch = _input[1];
				bot.Controller.Jump = true;
			}
			else if (!(num < (_jumping ? JumpTime : 0f) + 0.6f) || CancelFlip)
			{
				Finished = true;
			}
		}
	}
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
	public class Drive : IAction
	{
		public Vec3 Target;

		public float TargetSpeed;

		public bool Backwards;

		public bool AllowDodges;

		public bool WasteBoost;

		public IAction Action;

		private float timeOnGround;

		public bool Finished { get; private set; }

		public bool Interruptible { get; private set; }

		public float TimeRemaining { get; private set; }

		public Drive(Car car, Vec3 target)
		{
			Interruptible = true;
			Finished = false;
			Target = target;
			TargetSpeed = 2300f;
			Backwards = ShouldDriveBackwrds(car, target);
			AllowDodges = true;
			WasteBoost = false;
			Action = null;
		}

		public Drive(Car car, Vec3 target, float targetSpeed)
		{
			Interruptible = true;
			Finished = false;
			Target = target;
			TargetSpeed = targetSpeed;
			Backwards = ShouldDriveBackwrds(car, target);
			AllowDodges = true;
			WasteBoost = false;
			Action = null;
		}

		public Drive(Car car, Vec3 target, float targetSpeed, bool allowDodges)
		{
			Interruptible = true;
			Finished = false;
			Target = target;
			TargetSpeed = targetSpeed;
			Backwards = ShouldDriveBackwrds(car, target, allowDodges);
			AllowDodges = allowDodges;
			WasteBoost = false;
			Action = null;
		}

		public Drive(Car car, Vec3 target, float targetSpeed, bool allowDodges, bool wasteBoost)
		{
			Interruptible = true;
			Finished = false;
			Target = target;
			TargetSpeed = targetSpeed;
			Backwards = ShouldDriveBackwrds(car, target, allowDodges);
			AllowDodges = allowDodges;
			WasteBoost = wasteBoost;
			Action = null;
		}

		public void Run(RUBot bot)
		{
			TimeRemaining = Distance(bot.Me) / TargetSpeed;
			Surface surface = Field.NearestSurface(Target);
			if (Action == null)
			{
				timeOnGround += bot.DeltaTime;
				Surface surface2 = surface;
				Surface surface3 = Field.NearestSurface(bot.Me.Location);
				float num = bot.Me.Velocity.Length();
				float num2 = bot.Me.Velocity.Dot(bot.Me.Forward);
				Vec3 vec = Field.LimitToNearestSurface(Target);
				if (surface3.Normal.Dot(surface.Normal) < 0.95f)
				{
					surface2 = FindNextSurface(Field.LimitToNearestSurface(bot.Me.Location), vec);
					vec = surface2.Limit(vec);
					Vec3 vec2 = surface3.Limit(vec);
					vec = vec2 - surface2.Normal.FlatNorm(surface3.Normal) * MathF.Max(vec2.Dist(vec) - 75f, 0f);
				}
				if (surface3.Key != surface.Key)
				{
					vec = FindTargetAroundCorner(bot, vec, surface2);
				}
				float num3 = TurnRadius(MathF.Abs(num2));
				Vec3 vec3 = surface3.Limit(bot.Me.Location) + bot.Me.Right.FlatNorm(surface3.Normal) * MathF.Sign(bot.Me.Right.Dot(vec - bot.Me.Location)) * num3;
				if (Field.DistanceBetweenPoints(vec3, Target) > num3 - 40f)
				{
					bot.Throttle(TargetSpeed, Backwards);
				}
				else
				{
					bot.Throttle(MathF.Max(SpeedFromTurnRadius(TurnRadius(bot.Me, Target)), 400f), Backwards);
				}
				Vec3 targetLocation = vec;
				bool backwards = Backwards;
				float num4 = bot.AimAt(targetLocation, default, backwards)[1];
				bot.Controller.Boost = bot.Controller.Boost && num4 < 0.2f && !Backwards && WasteBoost;
				bot.Controller.Handbrake = (MathF.Abs(num4) > 2f || Field.InGoal(vec3, num3) || (Field.DistanceBetweenPoints(vec3, Target) < num3 - 40f && bot.Me.Velocity.Length() > 800f)) && surface3.Normal.Dot(Vec3.Up) > 0.9f && bot.Me.Velocity.Normalize().Dot(bot.Me.Forward) > 0.97f && MathF.Abs(bot.Me.LocalAngularVelocity.z) > 1f;
				bot.Renderer.Line3D(vec, vec + Field.NearestSurface(vec).Normal * 200f, System.Drawing.Color.LimeGreen);
				Vec3 pos = bot.Me.LocationAfterSpeedFlip(bot.Me.Boost > 0 && WasteBoost);
				float num5 = bot.Me.Location.FlatDist(vec) / MathF.Max(num + 500f, 1410f);
				float num6 = bot.Me.Location.FlatDist(vec) / MathF.Max(num + 500f + MathF.Min(bot.Me.Boost, 40f) / 33.3f * 991.667f / 2f, 1410f);
				if (!bot.Me.IsGrounded)
				{
					Action = new Recovery(WasteBoost, Target);
				}
				else if (AllowDodges && (!Field.InGoal(pos, 50f) || Field.InGoal(Target, 50f)) && num < 2000f && timeOnGround > 0.2f && Game.Gravity.z < -500f)
				{
					float num7 = bot.Me.Forward.FlatAngle(bot.Me.Location.Direction(vec), bot.Me.Up);
					float num8 = 0f - bot.Me.Forward.FlatAngle(bot.Me.Location.Direction(vec), bot.Me.Up);
					if (num2 > 0f)
					{
						if (Field.InField(pos, 50f) && TargetSpeed > 100f + num2 && num > 1000f && num7 < 0.1f && bot.Me.Location.z < 100f)
						{
							Dodge dodge = new Dodge(bot.Me.Location.FlatDirection(Target));
							if (num6 > 1.35f && bot.Me.Boost > 0 && Field.InField(pos, 500f) && WasteBoost)
							{
								Action = new SpeedFlip(bot.Me.Location.FlatDirection(Target));
							}
							else if (num5 > dodge.Duration)
							{
								Action = dodge;
							}
						}
						else if (num > 800f && num7 < 1f && bot.Me.Location.z + 2f * bot.Me.Velocity.z < 800f && bot.Me.Location.z > 100f)
						{
							Wavedash wavedash = new Wavedash(bot.Me.Location.FlatDirection(Target));
							if (num5 > wavedash.Duration)
							{
								Action = wavedash;
							}
						}
					}
					else if (num > 500f && num8 < 0.1f && num5 > 0.7f)
					{
						Action = new HalfFlip();
					}
				}
			}
			else if (Action != null && Action.Finished)
			{
				Action = null;
				Backwards = false;
				timeOnGround = 0f;
			}
			else if (Action != null)
			{
				Action.Run(bot);
				if (Action is SpeedFlip)
				{
					bot.Controller.Boost = WasteBoost && bot.Me.PredictLandingPosition().FlatDirection(Target).Dot(bot.Me.Location.FlatDirection(Target)) > 0.5f;
				}
			}
			bot.Renderer.Line3D(Field.LimitToNearestSurface(Target), Field.LimitToNearestSurface(Target) + surface.Normal * 200f, System.Drawing.Color.LimeGreen);
			Interruptible = Action == null || Action.Interruptible;
			if (Field.LimitToNearestSurface(bot.Me.Location).Dist(Field.LimitToNearestSurface(Target)) < 100f)
			{
				Finished = true;
			}
		}

		public float Distance(Car car)
		{
			return GetDistance(car, Target, Backwards);
		}

		public float Eta(Car car)
		{
			return GetEta(car, Target, Backwards, AllowDodges);
		}

		private static Surface FindNextSurface(Vec3 start, Vec3 target)
		{
			Vec3 vec = Field.LimitToNearestSurface((start + target) / 2f);
			for (float num = 0f; num < 2f; num += 0.25f)
			{
				Vec3 pos = Field.LimitToNearestSurface(start + (vec - start) * Utils.Cap(num, 0f, 1f) + (target - vec) * Utils.Cap(num - 1f, 0f, 1f));
				if (Field.NearestSurface(pos).Normal.Dot(Field.NearestSurface(start).Normal) < 0.95f)
				{
					return Field.NearestSurface(pos);
				}
			}
			return Field.NearestSurface(target);
		}

		private static Vec3 FindTargetAroundCorner(RUBot bot, Vec3 finalTarget, Surface nextSurface)
		{
			Surface surface = Field.NearestSurface(bot.Me.Location);
			if (surface.Key == "Ground")
			{
				Goal goal = ((Field.Side(bot.Team) == MathF.Sign(finalTarget.y)) ? bot.OurGoal : bot.TheirGoal);
				Vec3 start = bot.Me.Location.Direction(goal.LeftPost - new Vec3(MathF.Sign(goal.LeftPost.x) * 100, MathF.Sign(goal.LeftPost.y) * 50));
				Vec3 end = bot.Me.Location.Direction(goal.RightPost - new Vec3(MathF.Sign(goal.RightPost.x) * 100, MathF.Sign(goal.RightPost.y) * 50));
				Vec3 end2 = bot.Me.Location.Direction(goal.LeftPost + new Vec3(MathF.Sign(goal.LeftPost.x) * 60, -MathF.Sign(goal.LeftPost.y) * 50));
				Vec3 start2 = bot.Me.Location.Direction(goal.RightPost + new Vec3(MathF.Sign(goal.RightPost.x) * 60, -MathF.Sign(goal.RightPost.y) * 50));
				Vec3 vec = (nextSurface.Key.Contains("Goal Ground") ? bot.Me.Location.Direction(finalTarget).Clamp(start, end, surface.Normal) : bot.Me.Location.Direction(finalTarget).Clamp(start2, end2, surface.Normal));
				return bot.Me.Location + vec.Rescale(bot.Me.Location.Dist(finalTarget));
			}
			if (surface.Key.Contains("Goal Ground"))
			{
				Goal goal2 = ((Field.Side(bot.Team) == MathF.Sign(bot.Me.Location.y)) ? bot.OurGoal : bot.TheirGoal);
				Vec3 end3 = bot.Me.Location.Direction(goal2.LeftPost - new Vec3(MathF.Sign(goal2.LeftPost.x) * 100, MathF.Sign(goal2.LeftPost.y) * 50));
				Vec3 start3 = bot.Me.Location.Direction(goal2.RightPost - new Vec3(MathF.Sign(goal2.RightPost.x) * 100, MathF.Sign(goal2.RightPost.y) * 50));
				Vec3 vec2 = bot.Me.Location.Direction(finalTarget).Clamp(start3, end3, surface.Normal);
				return bot.Me.Location + vec2.Rescale(bot.Me.Location.Dist(finalTarget));
			}
			if (surface.Key.Contains("Backboard") || surface.Key.Contains("Backwall"))
			{
				Goal goal3 = ((Field.Side(bot.Team) == MathF.Sign(bot.Me.Location.y)) ? bot.OurGoal : bot.TheirGoal);
				Vec3 start4;
				Vec3 end4;
				if (surface.Key.Contains("Left Backwall"))
				{
					start4 = bot.Me.Location.Direction(goal3.TopRightCorner + new Vec3(MathF.Sign(goal3.TopRightCorner.x) * 50, 0f, 50f));
					end4 = bot.Me.Location.Direction(goal3.BottomRightCorner + new Vec3(MathF.Sign(goal3.BottomRightCorner.x) * 50, 0f, -50f));
				}
				else if (surface.Key.Contains("Right Backwall"))
				{
					start4 = bot.Me.Location.Direction(goal3.BottomLeftCorner + new Vec3(MathF.Sign(goal3.BottomLeftCorner.x) * 50, 0f, -50f));
					end4 = bot.Me.Location.Direction(goal3.TopLeftCorner + new Vec3(MathF.Sign(goal3.TopLeftCorner.x) * 50, 0f, 50f));
				}
				else
				{
					start4 = bot.Me.Location.Direction(goal3.TopLeftCorner + new Vec3(MathF.Sign(goal3.TopLeftCorner.x) * 50, 0f, 50f));
					end4 = bot.Me.Location.Direction(goal3.TopRightCorner + new Vec3(MathF.Sign(goal3.TopRightCorner.x) * 50, 0f, 50f));
				}
				Vec3 vec3 = bot.Me.Location.Direction(finalTarget).Clamp(start4, end4, surface.Normal);
				return bot.Me.Location + vec3.Rescale(bot.Me.Location.Dist(finalTarget));
			}
			return finalTarget;
		}

		public static float GetDistance(Car car, Vec3 target)
		{
			return GetDistance(car, target, ShouldDriveBackwrds(car, target));
		}

		public static float GetDistance(Car car, Vec3 target, bool backwards)
		{
			float angle;
			float radius;
			return GetDistance(car, target, backwards, out angle, out radius);
		}

		public static float GetDistance(Car car, Vec3 target, bool backwards, out float angle, out float radius)
		{
			target = Field.LimitToNearestSurface(target);
			Vec3 vec = car.PredictLandingPosition();
			Surface surface = Field.FindLandingSurface(car);
			Vec3 normal = surface.Normal;
			Vec3 vec2;
			if (car.IsGrounded)
			{
				vec2 = car.Forward;
			}
			else
			{
				vec2 = ((car.Velocity.FlatLen() > 500f) ? car.Velocity.FlatNorm(normal) : car.Location.FlatDirection(target, normal));
			}
			Vec3 vec3 = vec2.Cross(car.IsGrounded ? (-car.Up) : (-normal)).Normalize();
			float num = car.Velocity.Dot(vec2);
			angle = (backwards ? (-vec2) : vec2).FlatAngle(target - vec, normal);
			float speed = (backwards ? SpeedAfterTurn(0f - num, angle, 0.4f) : SpeedAfterTurn(num, angle, 0.5f));
			radius = TurnRadius(speed);
			Vec3 vec4 = vec + vec3 * MathF.Sign(vec3.Dot(target - vec)) * radius;
			Vec3 vec5 = surface.Limit(vec4);
			if (vec4.Dist(vec5) > 1f)
			{
				Vec3 vec6 = vec5.Direction(Field.LimitToNearestSurface(vec4 + surface.Normal * 500f));
				vec4 = vec5 + vec6 * vec4.Dist(vec5);
			}
			float num2 = Field.DistanceBetweenPoints(vec4, target);
			if (num2 + 50f < radius)
			{
				radius = TurnRadius(car, target);
				vec4 = vec + vec3 * MathF.Sign(vec3.Dot(target - vec)) * radius;
				num2 = Field.DistanceBetweenPoints(vec4, target);
			}
			angle = MathF.Abs((vec - vec4).FlatAngle(target - vec4, normal) - (((target - vec).Dot(backwards ? (-vec2) : vec2) < 0f) ? ((float)System.Math.PI * 2f) : 0f));
			angle -= MathF.Acos(Utils.Cap(radius / num2, 0f, 1f));
			angle = Utils.Cap(angle, 0f, (float)System.Math.PI * 2f);
			return MathF.Sqrt(MathF.Max(MathF.Pow(num2, 2f) - MathF.Pow(radius, 2f), 0f)) + radius * angle;
		}

		public static float GetEta(Car car, Vec3 target)
		{
			return GetEta(car, target, ShouldDriveBackwrds(car, target), allowDodges: true);
		}

		public static float GetEta(Car car, Vec3 target, bool allowDodges)
		{
			return GetEta(car, target, ShouldDriveBackwrds(car, target, allowDodges), allowDodges);
		}

		public static float GetEta(Car car, Vec3 target, bool backwards, bool allowDodges)
		{
			float boostUsed;
			return GetEta(car, target, backwards, allowDodges, out boostUsed);
		}

		public static float GetEta(Car car, Vec3 target, bool backwards, bool allowDodges, out float boostUsed)
		{
			float distance = GetDistance(car, target, backwards, out var angle, out var radius);
			float num = angle * radius;
			distance -= num;
			Vec3 up = (car.IsGrounded ? Field.NearestSurface(car.Location).Normal : Field.FindLandingSurface(car).Normal);
			float num2 = (car.IsGrounded ? car.Forward : ((car.Velocity.FlatLen() > 500f) ? car.Velocity.FlatNorm(up) : car.Location.FlatDirection(target, up))).Dot(car.Velocity);
			float num3 = car.PredictLandingTime();
			if (backwards)
			{
				float num4 = MathF.Max(SpeedAfterTurn(0f - num2, angle, 0.8f), 1400f);
				boostUsed = 0f;
				return num3 + num / MathF.Max(SpeedFromTurnRadius(radius), 400f) + distance / num4;
			}
			float num5 = MathF.Max(SpeedAfterTurn(num2, angle), 1400f);
			float num6 = Utils.Cap(num5 + 991.667f * (float)car.Boost / 33.3f, 1400f, 2300f);
			float num7 = (MathF.Pow(num6, 2f) - MathF.Pow(num5, 2f)) / 1983.334f;
			if (distance < num7)
			{
				num6 = Utils.Cap(MathF.Sqrt(MathF.Max(MathF.Pow(num5, 2f) + 1983.334f * distance, 0f)), 1400f, 2300f);
				boostUsed = 33.3f * (distance / ((num5 + num6) / 2f));
				return num3 + num / MathF.Max(SpeedFromTurnRadius(radius), 400f) + distance / ((num5 + num6) / 2f);
			}
			boostUsed = car.Boost;
			if (allowDodges && distance / num6 > 1.25f)
			{
				return num3 + num / MathF.Max(SpeedFromTurnRadius(radius), 400f) + num7 / ((num5 + num6) / 2f) + (distance - num7) / (num6 + 500f);
			}
			return num3 + num / MathF.Max(SpeedFromTurnRadius(radius), 400f) + num7 / ((num5 + num6) / 2f) + (distance - num7) / num6;
		}

		public static float TurnRadius(Car car, Vec3 target)
		{
			return Field.DistanceBetweenPoints(car.Location, target) / 2f / (car.Right * MathF.Sign(car.Right.Dot(target - car.Location))).Dot(car.Location.FlatDirection(target, car.Up));
		}

		public static bool ShouldDriveBackwrds(Car car, Vec3 target, bool isFlipping = false)
		{
			float eta = GetEta(car, target, backwards: false, isFlipping);
			if (GetEta(car, target, backwards: true, isFlipping) + 0.5f < eta && car.Forward.Dot(car.Velocity) < 0f)
			{
				return car.Forward.FlatAngle(car.Location.Direction(target), car.Up) > 1.8849558f;
			}
			return false;
		}

		public static float TurnRadius(float speed)
		{
			speed = Utils.Cap(speed, 0.01f, 2300f);
			if (speed <= 500f)
			{
				return Utils.Lerp(speed / 500f, 145f, 251f);
			}
			if (speed <= 1000f)
			{
				return Utils.Lerp((speed - 500f) / 500f, 251f, 425f);
			}
			if (speed <= 1500f)
			{
				return Utils.Lerp((speed - 1000f) / 500f, 425f, 727f);
			}
			if (speed <= 1750f)
			{
				return Utils.Lerp((speed - 1500f) / 250f, 727f, 909f);
			}
			return Utils.Lerp((speed - 1750f) / 550f, 909f, 1136f);
		}

		public static float SpeedFromTurnRadius(float radius)
		{
			radius = Utils.Cap(radius, 145f, 1136f);
			if (radius <= 251f)
			{
				return Utils.Lerp((radius - 145f) / 106f, 0f, 500f);
			}
			if (radius <= 425f)
			{
				return Utils.Lerp((radius - 251f) / 174f, 500f, 1000f);
			}
			if (radius <= 727f)
			{
				return Utils.Lerp((radius - 425f) / 302f, 1000f, 1500f);
			}
			if (radius <= 909f)
			{
				return Utils.Lerp((radius - 727f) / 182f, 1500f, 1750f);
			}
			return Utils.Lerp((radius - 909f) / 227f, 1750f, 2300f);
		}

		public static float SpeedAfterTurn(float currentSpeed, float angle, float modifier = 1f)
		{
			return Utils.Cap((1234f * (MathF.Exp(angle * 0.49f * modifier) * angle * 0.49f * modifier * ((currentSpeed > 1234f) ? 0.2f : 1f)) + currentSpeed) / (MathF.Exp(angle * 0.49f * modifier) * angle * 0.49f * modifier * ((currentSpeed > 1234f) ? 0.2f : 1f) + 1f), 0f, 2300f);
		}
	}
	public class GetBoost : IAction
	{
		public int BoostIndex;

		public Drive DriveAction;

		public Boost ChosenBoost;

		public float Eta;

		private readonly bool _initiallyInterruptible = true;

		public bool Finished { get; set; }

		public bool Interruptible { get; set; }

		public GetBoost(Car car)
		{
			Finished = false;
			Interruptible = true;
			float num = 99f;
			foreach (Boost boost in Field.Boosts)
			{
				if (boost.IsLarge)
				{
					float eta = Drive.GetEta(car, boost.Location);
					if (eta < num && (boost.IsActive || boost.TimeUntilActive < eta))
					{
						num = eta;
						BoostIndex = boost.Index;
					}
				}
			}
			ChosenBoost = Field.Boosts[BoostIndex];
			DriveAction = new Drive(car, ChosenBoost.Location, 2300f, allowDodges: true, wasteBoost: true);
		}

		public GetBoost(Car car, bool interruptible)
		{
			Finished = false;
			Interruptible = interruptible;
			_initiallyInterruptible = interruptible;
			float num = 99f;
			foreach (Boost boost in Field.Boosts)
			{
				if (boost.IsLarge)
				{
					float eta = Drive.GetEta(car, boost.Location);
					if (eta < num && (boost.IsActive || boost.TimeUntilActive < eta))
					{
						num = eta;
						BoostIndex = boost.Index;
					}
				}
			}
			ChosenBoost = Field.Boosts[BoostIndex];
			DriveAction = new Drive(car, ChosenBoost.Location, 2300f, allowDodges: true, wasteBoost: true);
		}

		public GetBoost(Car car, int boostIndex)
		{
			Finished = false;
			Interruptible = true;
			BoostIndex = boostIndex;
			ChosenBoost = Field.Boosts[BoostIndex];
			DriveAction = new Drive(car, ChosenBoost.Location, 2300f, allowDodges: true, wasteBoost: true);
		}

		public GetBoost(Car car, int boostIndex, bool interruptible)
		{
			Finished = false;
			Interruptible = interruptible;
			_initiallyInterruptible = interruptible;
			BoostIndex = boostIndex;
			ChosenBoost = Field.Boosts[BoostIndex];
			DriveAction = new Drive(car, ChosenBoost.Location, 2300f, allowDodges: true, wasteBoost: true);
		}

		public void Run(RUBot bot)
		{
			DriveAction.Run(bot);
			ChosenBoost = Field.Boosts[BoostIndex];
			Eta = Drive.GetEta(bot.Me, ChosenBoost.Location);
			Interruptible = _initiallyInterruptible && DriveAction.Interruptible;
			Finished = (!ChosenBoost.IsActive && (ChosenBoost.TimeUntilActive > Drive.GetEta(bot.Me, ChosenBoost.Location) || DriveAction.Finished)) || bot.Me.Boost > 90;
		}
	}
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
	public class HalfFlip : IAction
	{
		public const float Duration = 0.7f;

		private bool _jumping = true;

		private float _startTime = -1f;

		private int _step;

		public bool Finished { get; private set; }

		public bool Interruptible { get; private set; }

		public HalfFlip()
		{
			Interruptible = false;
			Finished = false;
		}

		public void Run(RUBot bot)
		{
			if (_startTime == -1f)
			{
				_startTime = Game.Time;
				_jumping = bot.Me.IsGrounded;
			}
			float num = Game.Time - _startTime;
			if (num < 0.1f && _jumping)
			{
				bot.Controller.Jump = true;
			}
			else if (_step < 3 && _jumping)
			{
				bot.Controller.Jump = false;
				_step++;
			}
			else if (num < (_jumping ? 0.1f : 0f) + 0.2f)
			{
				bot.Controller.Yaw = 0f;
				bot.Controller.Pitch = 1f;
				bot.Controller.Jump = true;
			}
			else if (num < (_jumping ? 0.1f : 0f) + 1f)
			{
				bot.AimAt(bot.Me.Location + bot.Me.Velocity.Flatten());
			}
			else
			{
				Finished = true;
			}
		}
	}
	public class JumpShot : Shot
	{
		private int _lastBoostAmount;

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
					bot.AimAt(bot.Me.Location + DodgeDirection, DodgeDirection.Cross(DodgeDirection.Cross(-_jumpedSurface.Normal)).Normalize(), backwards: false, prioritizeForward: false);
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
	public class Kickoff : IAction
	{
		private bool _isDiagonal;

		private float _timeOnGround;

		private bool _speedFlipped;

		private SpeedFlip _speedFlip;

		public bool Interruptible { get; set; }

		public bool Finished { get; set; }

		public Kickoff()
		{
			Interruptible = false;
			Finished = false;
		}

		public void Run(RUBot bot)
		{
			if (bot.Me.Velocity.Length() < 200f)
			{
				_isDiagonal = MathF.Abs(bot.Me.Location.x) > 1000f;
			}
			if (_speedFlipped && bot.Me.IsGrounded)
			{
				_timeOnGround += bot.DeltaTime;
			}
			if (_speedFlip != null && !_speedFlip.Finished)
			{
				bot.Controller.Boost = true;
				_speedFlip.Run(bot);
				return;
			}
			bot.Throttle(2300f);
			if (!_isDiagonal || _speedFlipped)
			{
				bot.AimAt(Ball.Location - Ball.Location.Direction(bot.TheirGoal.Location) * ((!_speedFlipped) ? 2600 : 120));
				bot.Controller.Steer *= ((!_isDiagonal && !_speedFlipped) ? 0.4f : 1f);
			}
			else if (bot.Me.Velocity.Length() > 500f)
			{
				bot.AimAt(Ball.Location);
			}
			if (!bot.IsKickoff)
			{
				Finished = true;
			}
			else if (bot.Me.Velocity.Length() > (_isDiagonal ? 600f : (700f + MathF.Abs(bot.Me.Location.x) * 3f)) && !_speedFlipped)
			{
				_speedFlipped = true;
				_speedFlip = new SpeedFlip(bot.Me.Location.FlatDirection(Ball.Location - Ball.Location.Direction(bot.TheirGoal.Location) * (_isDiagonal ? 250 : (-1000))));
			}
			else if (bot.Me.Location.Dist(Ball.Location) < 950f && _timeOnGround > 0.1f)
			{
				bot.Action = new Dodge(Ball.Location.Direction(bot.TheirGoal.Location), 0.15f);
			}
		}
	}
	public class QuickShot : IAction
	{
		public Vec3 Target;

		public Arrive ArriveAction;

		private float _latestTouchTime = -1f;

		public bool Finished { get; private set; }

		public bool Interruptible { get; private set; }

		public QuickShot(Car car, Vec3 target)
		{
			Interruptible = true;
			Target = target;
			ArriveAction = new Arrive(car, target, car.Location.Direction(target));
		}

		public void Run(RUBot bot)
		{
			if (_latestTouchTime < 0f && Ball.LatestTouch != null)
			{
				_latestTouchTime = Ball.LatestTouch.Time;
			}
			Vec3 direction = Ball.Location.Direction(Target);
			Vec3 normal = Field.NearestSurface(Ball.Location).Normal;
			ArriveAction.Target = Ball.Location;
			ArriveAction.Direction = direction;
			ArriveAction.Run(bot);
			Interruptible = ArriveAction.Interruptible;
			if (Interruptible && Ball.LatestTouch != null && _latestTouchTime != Ball.LatestTouch.Time && Ball.LatestTouch.PlayerIndex == bot.Index)
			{
				Finished = true;
			}
			else if ((ArriveAction.TimeRemaining < 0.2f || bot.Me.Location.Dist(Ball.Location) < 300f) && Interruptible)
			{
				bot.Action = new Dodge(bot.Me.Location.FlatDirection(Ball.Location, normal));
			}
		}
	}
	public class Recovery : IAction
	{
		public Vec3 Target;

		public bool WasteBoost;

		public Wavedash wavedashSubAction;

		public bool Finished { get; private set; }

		public bool Interruptible { get; private set; }

		public Recovery(bool wasteBoost = false, Vec3 target = default(Vec3))
		{
			Finished = false;
			Interruptible = true;
			Target = target;
			WasteBoost = wasteBoost;
		}

		public void Run(RUBot bot)
		{
			if (wavedashSubAction != null)
			{
				wavedashSubAction.Run(bot);
				Finished = wavedashSubAction.Finished;
				return;
			}
			float num = bot.Me.Velocity.Length();
			float num2 = bot.Me.PredictLandingTime();
			Vec3 normal = Field.FindLandingSurface(bot.Me).Normal;
			Vec3 vec = bot.Me.Velocity.FlatNorm(normal);
			if (Target != default(Vec3) && bot.Me.Location.Direction(Target).FlatAngle(bot.Me.Velocity, normal) > 1f)
			{
				vec = Utils.Lerp(num / 2300f, bot.Me.Location.FlatDirection(Target, normal), vec);
			}
			Vec3 vec2 = Utils.Lerp(Utils.Cap(num2 * 1.2f - 0.6f, 0f, 0.8f), vec, -Vec3.Up);
			Vec3 up = vec2.Cross(vec2.Cross(-normal)).Normalize();
			bool flag = bot.Me.Velocity.Flatten(bot.Me.Forward).Length() > 500f && bot.Me.Forward.Angle(vec) > 0.7f && bot.Me.Velocity.Length() > 1000f;
			bot.Controller.Boost = WasteBoost && num2 > 0.8f && bot.Me.Forward.z < -0.2f;
			bot.Controller.Throttle = 1f;
			bot.Controller.Handbrake = flag;
			bot.AimAt(bot.Me.Location + vec2, up, backwards: false, prioritizeForward: false);
			if (num < 2000f && num2 < 0.8f && Game.Gravity.z < -500f && bot.Me.Location.z > 100f && !bot.Me.HasDoubleJumped && (!bot.Me.IsGrounded || bot.Me.Velocity.Dot(Vec3.Up) < 200f))
			{
				wavedashSubAction = new Wavedash(vec);
			}
			Finished = bot.Me.IsGrounded && !flag;
		}
	}
	public abstract class Shot : IAction
	{
		public abstract bool Finished { get; internal set; }

		public abstract bool Interruptible { get; internal set; }

		public abstract BallSlice Slice { get; internal set; }

		public abstract Vec3 ShotTarget { get; internal set; }

		public abstract Vec3 TargetLocation { get; internal set; }

		public abstract Vec3 ShotDirection { get; internal set; }

		public abstract bool IsValid(Car car);

		internal bool ShotValid(float threshold = 60f)
		{
			BallSlice[] slices = Ball.Prediction.Slices;
			int num = 0;
			int num2 = slices.Length - 1;
			while (num2 + 1 - num > 2)
			{
				int num3 = (num + num2) / 2;
				if (slices[num3].Time > Slice.Time)
				{
					num2 = num3;
				}
				else
				{
					num = num3;
				}
			}
			float num4 = slices[num2].Time - slices[num].Time;
			float num5 = Slice.Time - slices[num].Time;
			Vec3 vec = (slices[num2].Location - slices[num].Location) * (1f / num4);
			Vec3 vec2 = slices[num].Location + vec * num5;
			return (Slice.Location - vec2).Length() < threshold;
		}

		public abstract void Run(RUBot bot);
	}
	internal class SpeedFlip : IAction
	{
		public const float Duration = 1.35f;

		public Vec3 Direction;

		private float _startTime = -1f;

		private int _side;

		public bool Interruptible { get; set; }

		public bool Finished { get; set; }

		public SpeedFlip(Vec3 direction)
		{
			Interruptible = false;
			Finished = false;
			Direction = direction.Normalize();
		}

		public void Run(RUBot bot)
		{
			bot.Controller.Throttle = 1f;
			if (_startTime < 0f)
			{
				float num = 0.06f * bot.Me.Velocity.Dot(bot.Me.Forward) / Drive.TurnRadius(bot.Me.Velocity.Dot(bot.Me.Forward));
				Vec3 vec = Direction.Rotate(num).Flatten().Normalize();
				Vec3 vec2 = Direction.Rotate(0f - num).Flatten().Normalize();
				if (bot.Me.Velocity.Angle(vec) < bot.Me.Velocity.Angle(vec2))
				{
					bot.AimAt(bot.Me.Location + vec);
					if (bot.Me.Velocity.FlatAngle(vec) < 0.05f)
					{
						_startTime = Game.Time;
						_side = 1;
					}
				}
				else
				{
					bot.AimAt(bot.Me.Location + vec2);
					if (bot.Me.Velocity.FlatAngle(vec2) < 0.05f)
					{
						_startTime = Game.Time;
						_side = -1;
					}
				}
			}
			else
			{
				float num2 = Game.Time - _startTime;
				if (0f < num2 && (double)num2 < 0.1)
				{
					bot.Controller.Jump = true;
				}
				else if (0.12 < (double)num2 && (double)num2 < 0.15)
				{
					bot.Controller.Jump = true;
					bot.Controller.Pitch = -1f;
					bot.Controller.Roll = (float)_side * 0.5f;
				}
				else if (0.15 < (double)num2 && (double)num2 < 0.75)
				{
					bot.Controller.Pitch = 1f;
					bot.Controller.Roll = _side;
				}
				else if (0.75 < (double)num2 && (double)num2 < 0.9)
				{
					bot.Controller.Pitch = 1f;
					bot.Controller.Handbrake = true;
					bot.Controller.Roll = _side;
					bot.Controller.Yaw = _side;
				}
				else if (0.9 < (double)num2)
				{
					Finished = true;
				}
			}
			if (!Field.InField(bot.Me.Location, 150f) || (_startTime < 0f && !bot.Me.IsGrounded))
			{
				Finished = true;
			}
		}
	}
	public class Wavedash : IAction
	{
		public Vec3 Direction;

		public float JumpTime;

		private bool _jumping = true;

		private float _startTime = -1f;

		private Vec3 _input = Vec3.Zero;

		public bool Finished { get; private set; }

		public bool Interruptible { get; private set; }

		public float Duration => JumpTime * 4f + 0.8f;

		public Wavedash()
		{
			Interruptible = false;
			Finished = false;
			Direction = Vec3.Zero;
			JumpTime = 0.05f;
		}

		public Wavedash(Vec3 direction)
		{
			Interruptible = false;
			Finished = false;
			Direction = direction;
			JumpTime = 0.05f;
		}

		public Wavedash(Vec3 direction, float jumpTime)
		{
			Interruptible = false;
			Finished = false;
			Direction = direction;
			JumpTime = jumpTime;
		}

		public void Run(RUBot bot)
		{
			if (_startTime == -1f)
			{
				_startTime = Game.Time;
				_jumping = bot.Me.IsGrounded;
			}
			if (Game.Time - _startTime < JumpTime && _jumping)
			{
				bot.Controller.Jump = true;
			}
			else if (!bot.Me.IsGrounded && bot.Me.Location.z < 50f && bot.Me.Velocity.z < -100f)
			{
				if (_input.Length() == 0f)
				{
					_input = ((Direction.Length() > 0f) ? new Vec3(bot.Me.Local(Direction)[1], 0f - bot.Me.Local(Direction)[0]) : new Vec3(bot.Me.Local(bot.Me.Velocity).Normalize()[1], 0f - bot.Me.Local(bot.Me.Velocity).Normalize()[0]));
				}
				bot.Controller.Yaw = _input[0];
				bot.Controller.Pitch = _input[1];
				bot.Controller.Jump = true;
			}
			else if (!bot.Me.IsGrounded)
			{
				Vec3 normal = Field.FindLandingSurface(bot.Me).Normal;
				Vec3 vec = ((Direction.Length() > 0f) ? Direction.FlatNorm(normal) : bot.Me.Velocity.FlatNorm(normal)) + normal * 0.2f;
				bot.AimAt(bot.Me.Location + vec, vec.Cross(-vec.Cross(normal)).Normalize(), backwards: false, prioritizeForward: false);
			}
			else
			{
				Finished = true;
			}
		}
	}
	public abstract class RUBot : Bot
	{
		public delegate Shot ShotCheck(BallSlice slice, Target target);

		public Controller Controller;

		public IAction Action;

		private bool _ready;

		private float _lastTouchTime;

		private float _lastTime;

		public new ExtendedRenderer Renderer { get; internal set; }

		public Car Me
		{
			get
			{
				if (Index >= Cars.Count)
				{
					return new Car();
				}
				return Cars.AllCars[Index];
			}
		}

		public List<Car> Teammates
		{
			get
			{
				List<Car> list = Cars.AllCars.FindAll((Car car) => car.Team == Team);
				list.Remove(Me);
				return list;
			}
		}

		public List<Car> LivingTeammates
		{
			get
			{
				List<Car> list = Cars.AllCars.FindAll((Car car) => !car.IsDemolished && car.Team == Team);
				list.Remove(Me);
				return list;
			}
		}

		public List<Car> Opponents => Cars.AllCars.FindAll((Car car) => car.Team != Team);

		public List<Car> LivingOpponents => Cars.AllCars.FindAll((Car car) => !car.IsDemolished && car.Team != Team);

		public Goal OurGoal => Field.Goals[Team];

		public Goal TheirGoal => Field.Goals[1 - Team];

		public int OurScore => Game.Scores[Team];

		public int TheirScore => Game.Scores[1 - Team];

		public bool IsKickoff { get; private set; }

		public float DeltaTime { get; private set; }

		public RUBot(string botName, int botTeam, int botIndex)
			: base(botName, botTeam, botIndex)
		{
			Console.WriteLine("RedUtils bot \"" + botName + "\" is up and running.");
		}

		private void GetReady(GameTickPacket packet)
		{
			Renderer = new ExtendedRenderer(base.Renderer);
			Field.Initialize(GetFieldInfo());
			Cars.Initialize(packet);
			_ready = true;
		}

		private void Process(GameTickPacket packet)
		{
			if (Cars.Count != packet.PlayersLength)
			{
				Cars.Initialize(packet);
			}
			else
			{
				Cars.Update(packet);
			}
			Ball.Update(this, packet.Ball.Value);
			Game.Update(packet);
			Field.Update(packet);
			if (!IsKickoff && Game.IsKickoffPause && Game.IsRoundActive)
			{
				Action = null;
			}
			IsKickoff = Game.IsKickoffPause && Game.IsRoundActive;
		}

		private void UpdateDeltaTime()
		{
			DeltaTime = Game.Time - _lastTime;
			_lastTime = Game.Time;
		}

		public override Controller GetOutput(GameTickPacket packet)
		{
			Controller = default;
			if (!_ready)
			{
				GetReady(packet);
			}
			Process(packet);
			Run();
			if (Action != null)
			{
				Action.Run(this);
				float num = ((Ball.LatestTouch == null) ? (-1f) : Ball.LatestTouch.Time);
				if (Action.Finished || (_lastTouchTime != num && Action.Interruptible))
				{
					_lastTouchTime = num;
					Action = null;
				}
			}
			UpdateDeltaTime();
			return Controller;
		}

		public abstract void Run();

		internal new BallPrediction GetBallPrediction()
		{
			return new BallPrediction(base.GetBallPrediction());
		}

		public float Throttle(float targetSpeed, bool backwards = false)
		{
			float x = Me.Local(Me.Velocity).x;
			float num = targetSpeed * (float)((!backwards) ? 1 : (-1)) - x;
			Controller.Throttle = Utils.Cap(MathF.Pow(num, 2f) * (float)MathF.Sign(num) / 1000f, -1f, 1f);
			Controller.Boost = targetSpeed > 1400f && num > 100f && x < 2250f && Controller.Throttle == 1f && !backwards;
			return x;
		}

		public float[] AimAt(Vec3 targetLocation, Vec3 up = default(Vec3), bool backwards = false, bool prioritizeForward = true)
		{
			Vec3 vec = Me.Local(targetLocation - Me.Location) * ((!backwards) ? 1 : (-1));
			Vec3 vec2 = ((up.Length() != 0f) ? up : Vec3.Up);
			Vec3 vec3 = Me.Local(vec2.Normalize());
			float[] array = new float[3]
			{
				prioritizeForward ? MathF.Atan2(vec.z, vec.x) : (0f - MathF.Atan2(vec3.x, vec3.z)),
				MathF.Atan2(vec.y, vec.x),
				MathF.Atan2(vec3.y, vec3.z)
			};
			Controller.Steer = SteerPD(array[1], (0f - Me.LocalAngularVelocity[2]) * (Controller.Handbrake ? 0.5f : 0.01f)) * (float)((!backwards) ? 1 : (-1));
			Controller.Pitch = SteerPD(array[0], Me.LocalAngularVelocity[1] * 0.2f);
			Controller.Yaw = SteerPD(array[1], (0f - Me.LocalAngularVelocity[2]) * 0.15f);
			Controller.Roll = SteerPD(array[2], Me.LocalAngularVelocity[0] * 0.25f);
			return array;
		}

		private static float SteerPD(float angle, float rate)
		{
			return Utils.Cap(MathF.Pow(35f * (angle + rate), 3f) / 10f, -1f, 1f);
		}

		public static Shot FindShot(ShotCheck shotCheck, Target target)
		{
			return shotCheck(Ball.Prediction.Find((BallSlice slice) => shotCheck(slice, target) != null, 1), target);
		}

		public Shot DefaultShotCheck(BallSlice slice, Target target)
		{
			if (slice != null)
			{
				float num = slice.Time - Game.Time;
				if (num > 0f && target.Fits(slice.Location))
				{
					Vec3 shotTarget;
					if (target.TargetSurface.Location.FlatDist(slice.Location) < 900f && slice.Location.z < 640f)
					{
						Ball ball = slice.ToBall();
						ball.velocity = (slice.Location - Me.Location) / num;
						shotTarget = target.Clamp(ball);
					}
					else
					{
						shotTarget = target.Clamp(slice.ToBall(), slice.Location.Direction(target.TargetSurface.Location));
					}
					AerialShot aerialShot = new AerialShot(Me, slice, shotTarget);
					if (aerialShot.IsValid(Me))
					{
						return aerialShot;
					}
					GroundShot groundShot = new GroundShot(Me, slice, shotTarget);
					Vec3 vec = (groundShot.TargetLocation - Me.Location) / num;
					if (groundShot.IsValid(Me) && (groundShot.TargetLocation.z > 300f || Me.Location.FlatDirection(groundShot.TargetLocation).Dot(groundShot.ShotDirection.FlatNorm()) < 0.3f || (slice.Velocity - vec).Length() > 1400f))
					{
						return groundShot;
					}
					JumpShot jumpShot = new JumpShot(Me, slice, shotTarget);
					if (jumpShot.IsValid(Me))
					{
						return jumpShot;
					}
					DoubleJumpShot doubleJumpShot = new DoubleJumpShot(Me, slice, shotTarget);
					if (doubleJumpShot.IsValid(Me))
					{
						return doubleJumpShot;
					}
				}
			}
			return null;
		}

		public Shot ClearShotCheck(BallSlice slice, Target target)
		{
			if (slice != null)
			{
				float num = slice.Time - Game.Time;
				if (num > 0f && target.Fits(slice.Location))
				{
					Vec3 shotTarget = target.Clamp(slice.ToBall(), Me.Location.Direction(slice.Location));
					AerialShot aerialShot = new AerialShot(Me, slice, shotTarget);
					if (aerialShot.IsValid(Me))
					{
						return aerialShot;
					}
					GroundShot groundShot = new GroundShot(Me, slice, shotTarget);
					Vec3 vec = (groundShot.TargetLocation - Me.Location) / num;
					if (groundShot.IsValid(Me) && (groundShot.TargetLocation.z > 300f || Me.Location.FlatDirection(groundShot.TargetLocation).Dot(groundShot.ShotDirection.FlatNorm()) < 0.3f || (slice.Velocity - vec).Length() > 1400f))
					{
						return groundShot;
					}
					JumpShot jumpShot = new JumpShot(Me, slice, shotTarget);
					if (jumpShot.IsValid(Me))
					{
						return jumpShot;
					}
					DoubleJumpShot doubleJumpShot = new DoubleJumpShot(Me, slice, shotTarget);
					if (doubleJumpShot.IsValid(Me))
					{
						return doubleJumpShot;
					}
				}
			}
			return null;
		}

		public Shot SaveShotCheck(BallSlice slice, Target target)
		{
			if (slice != null && slice.Time - Game.Time > 0f)
			{
				Vec3 vec = Me.Location.FlatDirection(slice.Location).Clamp(Vec3.X * -Field.Side(Team), Vec3.X * Field.Side(Team));
				Vec3 shotTarget = slice.Location + vec.Normalize() * 2000f;
				AerialShot aerialShot = new AerialShot(Me, slice, shotTarget);
				if (aerialShot.IsValid(Me))
				{
					return aerialShot;
				}
				JumpShot jumpShot = new JumpShot(Me, slice, shotTarget);
				if (jumpShot.IsValid(Me) && (Field.BestJumpingSurface(Me.Location, jumpShot.TargetLocation).Normal.z > 0.95f || jumpShot.TargetLocation.Dist(OurGoal.Location) > 1200f))
				{
					return jumpShot;
				}
				DoubleJumpShot doubleJumpShot = new DoubleJumpShot(Me, slice, shotTarget);
				if (doubleJumpShot.IsValid(Me))
				{
					return doubleJumpShot;
				}
			}
			return null;
		}
	}
	public class ExtendedRenderer
	{
		public System.Drawing.Color Color = System.Drawing.Color.White;

		private readonly Renderer _renderer;

		public ExtendedRenderer(Renderer renderer)
		{
			_renderer = renderer;
		}

		public void Text2D(string text, Vec3 upperLeft, int scale = 1, System.Drawing.Color? color = null)
		{
			_renderer.DrawString2D(text, color ?? Color, NumVec2(upperLeft), scale, scale);
		}

		public void Text3D(string text, Vec3 pos, int scale = 1, System.Drawing.Color? color = null)
		{
			_renderer.DrawString3D(text, color ?? Color, NumVec(pos), scale, scale);
		}

		public void Rect3D(Vec3 pos, int width, int height, bool fill = true, System.Drawing.Color? color = null)
		{
			_renderer.DrawRectangle3D(color ?? Color, NumVec(pos), width, height, fill);
		}

		public void Line3D(Vec3 start, Vec3 end, System.Drawing.Color? color = null)
		{
			_renderer.DrawLine3D(color ?? Color, NumVec(start), NumVec(end));
		}

		public void Line2D(Vec3 start, Vec3 end, System.Drawing.Color? color = null)
		{
			_renderer.DrawLine2D(color ?? Color, NumVec2(start), NumVec2(end));
		}

		public void Polyline3D(IEnumerable<Vec3> points, System.Drawing.Color? color = null)
		{
			_renderer.DrawPolyLine3D(color ?? Color, points.Select(NumVec).ToArray());
		}

		public void Polyline2D(IEnumerable<Vec3> points, System.Drawing.Color? color = null)
		{
			_renderer.DrawPolyLine2D(color ?? Color, points.Select(NumVec2).ToArray());
		}

		public void Circle(Vec3 pos, Vec3 normal, float radius, System.Drawing.Color? color = null)
		{
			Vec3 vec = normal.Cross(pos).Normalize() * radius;
			int num = (int)MathF.Pow(radius, 0.69f) + 4;
			float angle = (float)System.Math.PI * 2f / (float)num;
			Mat3x3 mat3x = Mat3x3.RotationFromAxis(normal.Normalize(), angle);
			Vec3[] array = new Vec3[num + 1];
			for (int i = 0; i <= num; i++)
			{
				vec = mat3x.Dot(vec);
				array[i] = pos + vec;
			}
			Polyline3D(array, color);
		}

		public void Cross(Vec3 pos, float size, System.Drawing.Color? color = null)
		{
			float num = size / 2f;
			Line3D(pos + num * Vec3.X, pos - num * Vec3.X, color);
			Line3D(pos + num * Vec3.Y, pos - num * Vec3.Y, color);
			Line3D(pos + num * Vec3.Z, pos - num * Vec3.Z, color);
		}

		public void CrossAngled(Vec3 pos, float size, System.Drawing.Color? color = null)
		{
			float num = 0.5f * size / MathF.Sqrt(2f);
			Line3D(pos + new Vec3(num, num, num), pos + new Vec3(0f - num, 0f - num, 0f - num), color);
			Line3D(pos + new Vec3(num, num, 0f - num), pos + new Vec3(0f - num, 0f - num, num), color);
			Line3D(pos + new Vec3(num, 0f - num, 0f - num), pos + new Vec3(0f - num, num, num), color);
			Line3D(pos + new Vec3(num, 0f - num, num), pos + new Vec3(0f - num, num, 0f - num), color);
		}

		public void Cube(Vec3 pos, float size, System.Drawing.Color? color = null)
		{
			Cube(pos, new Vec3(size, size, size), color);
		}

		public void Cube(Vec3 pos, Vec3 size, System.Drawing.Color? color = null)
		{
			Vec3 vec = size / 2f;
			Line3D(pos + new Vec3(0f - vec.x, 0f - vec.y, 0f - vec.z), pos + new Vec3(0f - vec.x, 0f - vec.y, vec.z), color);
			Line3D(pos + new Vec3(vec.x, 0f - vec.y, 0f - vec.z), pos + new Vec3(vec.x, 0f - vec.y, vec.z), color);
			Line3D(pos + new Vec3(0f - vec.x, vec.y, 0f - vec.z), pos + new Vec3(0f - vec.x, vec.y, vec.z), color);
			Line3D(pos + new Vec3(vec.x, vec.y, 0f - vec.z), pos + new Vec3(vec.x, vec.y, vec.z), color);
			Line3D(pos + new Vec3(0f - vec.x, 0f - vec.y, 0f - vec.z), pos + new Vec3(0f - vec.x, vec.y, 0f - vec.z), color);
			Line3D(pos + new Vec3(vec.x, 0f - vec.y, 0f - vec.z), pos + new Vec3(vec.x, vec.y, 0f - vec.z), color);
			Line3D(pos + new Vec3(0f - vec.x, 0f - vec.y, vec.z), pos + new Vec3(0f - vec.x, vec.y, vec.z), color);
			Line3D(pos + new Vec3(vec.x, 0f - vec.y, vec.z), pos + new Vec3(vec.x, vec.y, vec.z), color);
			Line3D(pos + new Vec3(0f - vec.x, 0f - vec.y, 0f - vec.z), pos + new Vec3(vec.x, 0f - vec.y, 0f - vec.z), color);
			Line3D(pos + new Vec3(0f - vec.x, 0f - vec.y, vec.z), pos + new Vec3(vec.x, 0f - vec.y, vec.z), color);
			Line3D(pos + new Vec3(0f - vec.x, vec.y, 0f - vec.z), pos + new Vec3(vec.x, vec.y, 0f - vec.z), color);
			Line3D(pos + new Vec3(0f - vec.x, vec.y, vec.z), pos + new Vec3(vec.x, vec.y, vec.z), color);
		}

		public void OrientatedCube(Vec3 pos, Mat3x3 orientation, Vec3 size, System.Drawing.Color? color = null)
		{
			Vec3 vec = size / 2f;
			Mat3x3 mat3x = orientation.Transpose();
			Line3D(pos + mat3x.Dot(new Vec3(0f - vec.x, 0f - vec.y, 0f - vec.z)), pos + mat3x.Dot(new Vec3(0f - vec.x, 0f - vec.y, vec.z)), color);
			Line3D(pos + mat3x.Dot(new Vec3(vec.x, 0f - vec.y, 0f - vec.z)), pos + mat3x.Dot(new Vec3(vec.x, 0f - vec.y, vec.z)), color);
			Line3D(pos + mat3x.Dot(new Vec3(0f - vec.x, vec.y, 0f - vec.z)), pos + mat3x.Dot(new Vec3(0f - vec.x, vec.y, vec.z)), color);
			Line3D(pos + mat3x.Dot(new Vec3(vec.x, vec.y, 0f - vec.z)), pos + mat3x.Dot(new Vec3(vec.x, vec.y, vec.z)), color);
			Line3D(pos + mat3x.Dot(new Vec3(0f - vec.x, 0f - vec.y, 0f - vec.z)), pos + mat3x.Dot(new Vec3(0f - vec.x, vec.y, 0f - vec.z)), color);
			Line3D(pos + mat3x.Dot(new Vec3(vec.x, 0f - vec.y, 0f - vec.z)), pos + mat3x.Dot(new Vec3(vec.x, vec.y, 0f - vec.z)), color);
			Line3D(pos + mat3x.Dot(new Vec3(0f - vec.x, 0f - vec.y, vec.z)), pos + mat3x.Dot(new Vec3(0f - vec.x, vec.y, vec.z)), color);
			Line3D(pos + mat3x.Dot(new Vec3(vec.x, 0f - vec.y, vec.z)), pos + mat3x.Dot(new Vec3(vec.x, vec.y, vec.z)), color);
			Line3D(pos + mat3x.Dot(new Vec3(0f - vec.x, 0f - vec.y, 0f - vec.z)), pos + mat3x.Dot(new Vec3(vec.x, 0f - vec.y, 0f - vec.z)), color);
			Line3D(pos + mat3x.Dot(new Vec3(0f - vec.x, 0f - vec.y, vec.z)), pos + mat3x.Dot(new Vec3(vec.x, 0f - vec.y, vec.z)), color);
			Line3D(pos + mat3x.Dot(new Vec3(0f - vec.x, vec.y, 0f - vec.z)), pos + mat3x.Dot(new Vec3(vec.x, vec.y, 0f - vec.z)), color);
			Line3D(pos + mat3x.Dot(new Vec3(0f - vec.x, vec.y, vec.z)), pos + mat3x.Dot(new Vec3(vec.x, vec.y, vec.z)), color);
		}

		public void Octahedron(Vec3 pos, float size, System.Drawing.Color? color = null)
		{
			float num = size / 2f;
			Line3D(pos + new Vec3(num, 0f, 0f), pos + new Vec3(0f, num, 0f), color);
			Line3D(pos + new Vec3(0f, num, 0f), pos + new Vec3(0f - num, 0f, 0f), color);
			Line3D(pos + new Vec3(0f - num, 0f, 0f), pos + new Vec3(0f, 0f - num, 0f), color);
			Line3D(pos + new Vec3(0f, 0f - num, 0f), pos + new Vec3(num, 0f, 0f), color);
			Line3D(pos + new Vec3(num, 0f, 0f), pos + new Vec3(0f, 0f, num), color);
			Line3D(pos + new Vec3(0f, 0f, num), pos + new Vec3(0f - num, 0f, 0f), color);
			Line3D(pos + new Vec3(0f - num, 0f, 0f), pos + new Vec3(0f, 0f, 0f - num), color);
			Line3D(pos + new Vec3(0f, 0f, 0f - num), pos + new Vec3(num, 0f, 0f), color);
			Line3D(pos + new Vec3(0f, num, 0f), pos + new Vec3(0f, 0f, num), color);
			Line3D(pos + new Vec3(0f, 0f, num), pos + new Vec3(0f, 0f - num, 0f), color);
			Line3D(pos + new Vec3(0f, 0f - num, 0f), pos + new Vec3(0f, 0f, 0f - num), color);
			Line3D(pos + new Vec3(0f, 0f, 0f - num), pos + new Vec3(0f, num, 0f), color);
		}

		private System.Numerics.Vector3 NumVec(Vec3 v)
		{
			return new System.Numerics.Vector3(v.x, v.y, v.z);
		}

		private Vector2 NumVec2(Vec3 v)
		{
			return new Vector2(v.x, v.y);
		}
	}
	public static class AccelLUT
	{
		public static readonly string ThrottleCsv = "Throttle.csv";

		public static readonly string BoostCsv = "Boost.csv";

		public static List<AccelData> ThrottleData { get; private set; }

		public static List<AccelData> BoostData { get; private set; }

		public static void Initialize()
		{
			InitializeThrottleData();
			InitializeBoostData();
		}

		public static AccelData FindDataByTime(List<AccelData> data, float time)
		{
			AccelData item = new AccelData
			{
				time = time
			};
			int num = data.BinarySearch(item, Comparer<AccelData>.Create((AccelData a, AccelData b) => a.time.CompareTo(b.time)));
			if (num < 0)
			{
				num = ~num;
				if (num >= data.Count)
				{
					num = data.Count - 1;
				}
			}
			return data[num];
		}

		public static AccelData FindDataByDist(List<AccelData> data, float dist)
		{
			AccelData item = new AccelData
			{
				distance = dist
			};
			int num = data.BinarySearch(item, Comparer<AccelData>.Create((AccelData a, AccelData b) => a.distance.CompareTo(b.distance)));
			if (num < 0)
			{
				num = ~num;
				if (num >= data.Count)
				{
					num = data.Count - 1;
				}
			}
			return data[num];
		}

		public static AccelData FindDataBySpeed(List<AccelData> data, float speed)
		{
			AccelData item = new AccelData
			{
				speed = speed
			};
			int num = data.BinarySearch(item, Comparer<AccelData>.Create((AccelData a, AccelData b) => a.speed.CompareTo(b.speed)));
			if (num < 0)
			{
				num = ~num;
				if (num >= data.Count)
				{
					num = data.Count - 1;
				}
			}
			return data[num];
		}

		private static void InitializeThrottleData()
		{
			using StreamReader streamReader = new StreamReader("Throttle.csv");
			ThrottleData = new List<AccelData>();
			while (!streamReader.EndOfStream)
			{
				string[] array = streamReader.ReadLine().Split(',');
				ThrottleData.Add(new AccelData(float.Parse(array[0]), float.Parse(array[1]), float.Parse(array[2])));
			}
		}

		private static void InitializeBoostData()
		{
			using StreamReader streamReader = new StreamReader("Boost.csv");
			BoostData = new List<AccelData>();
			while (!streamReader.EndOfStream)
			{
				string[] array = streamReader.ReadLine().Split(',');
				BoostData.Add(new AccelData(float.Parse(array[0]), float.Parse(array[1]), float.Parse(array[2])));
			}
		}
	}
	public class AccelData
	{
		public float time;

		public float distance;

		public float speed;

		public AccelData()
		{
		}

		public AccelData(float time, float distance, float speed)
		{
			this.time = time;
			this.distance = distance;
			this.speed = speed;
		}
	}
	public class Ball
	{
		public const float MaxSpeed = 6000f;

		public const float Radius = 93.15f;

		public Vec3 location;

		public Vec3 velocity;

		public Vec3 angularVelocity;

		public static Ball MainBall => new Ball(Location, Velocity, AngularVelocity);

		public static Vec3 Location { get; private set; }

		public static Vec3 Velocity { get; private set; }

		public static Vec3 AngularVelocity { get; private set; }

		public static BallTouch LatestTouch { get; private set; }

		public static BallPrediction Prediction { get; private set; }

		static Ball()
		{
			Location = Vec3.Zero;
			Velocity = Vec3.Zero;
			AngularVelocity = Vec3.Zero;
		}

		public Ball(Vec3 location, Vec3 velocity)
		{
			this.location = location;
			this.velocity = velocity;
			angularVelocity = Vec3.Zero;
		}

		public Ball(Vec3 location, Vec3 velocity, Vec3 angularVelocity)
		{
			this.location = location;
			this.velocity = velocity;
			this.angularVelocity = angularVelocity;
		}

		public static void Update(RUBot bot, BallInfo ballInfo)
		{
			Location = (ballInfo.Physics.Value.Location.HasValue ? new Vec3(ballInfo.Physics.Value.Location.Value) : Location);
			Velocity = (ballInfo.Physics.Value.Velocity.HasValue ? new Vec3(ballInfo.Physics.Value.Velocity.Value) : Velocity);
			AngularVelocity = (ballInfo.Physics.Value.AngularVelocity.HasValue ? new Vec3(ballInfo.Physics.Value.AngularVelocity.Value) : AngularVelocity);
			LatestTouch = (ballInfo.LatestTouch.HasValue ? new BallTouch(ballInfo.LatestTouch.Value) : null);
			Prediction = bot.GetBallPrediction();
		}

		public Ball Predict(float time)
		{
			return new Ball(location + velocity * time + Game.Gravity * 0.5f * time * time, velocity + Game.Gravity * time);
		}

		public Vec3 PredictLocation(float time)
		{
			return location + velocity * time + Game.Gravity * 0.5f * time * time;
		}

		public Vec3 PredictVelocity(float time)
		{
			return velocity + Game.Gravity * time;
		}
	}
	public struct BallPrediction
	{
		public BallSlice[] Slices;

		public BallSlice this[int index] => Slices[index];

		public int Length => Slices.Length;

		public BallPrediction(rlbot.flat.BallPrediction ballPrediction)
		{
			Slices = new BallSlice[ballPrediction.SlicesLength];
			for (int i = 0; i < ballPrediction.SlicesLength; i++)
			{
				Slices[i] = new BallSlice(ballPrediction.Slices(i).Value);
			}
		}

		public BallSlice Find(Predicate<BallSlice> predicate, int increment = 20)
		{
			if (Length > 0)
			{
				for (int i = increment; i < Length; i += increment)
				{
					if (predicate(Slices[i]))
					{
						for (int j = i - increment; j < i && !(MathF.Abs(Slices[j].Location.y) > 5250f); j++)
						{
							if (predicate(Slices[j]))
							{
								return Slices[j];
							}
						}
					}
					else if (MathF.Abs(Slices[i].Location.y) > 5250f)
					{
						break;
					}
				}
			}
			return null;
		}

		public BallSlice FindGoal(int team)
		{
			int num = -Field.Side(team);
			if (Length > 0)
			{
				for (int i = 6; i < Length; i += 6)
				{
					if (!(Slices[i].Location.y * (float)num > 5250f))
					{
						continue;
					}
					for (int j = i - 6; j < i; j++)
					{
						if (Slices[j].Location.y * (float)num > 5250f)
						{
							return Slices[j];
						}
					}
				}
			}
			return null;
		}
	}
	public class BallSlice
	{
		public readonly Vec3 Location;

		public readonly Vec3 Velocity;

		public readonly Vec3 AngularVelocity;

		public readonly float Time;

		public BallSlice(PredictionSlice slice)
		{
			Location = new Vec3(slice.Physics.Value.Location.Value);
			Velocity = new Vec3(slice.Physics.Value.Velocity.Value);
			AngularVelocity = new Vec3(slice.Physics.Value.AngularVelocity.Value);
			Time = slice.GameSeconds;
		}

		public Ball ToBall()
		{
			return new Ball(Location, Velocity, AngularVelocity);
		}
	}
	public class BallTouch
	{
		public readonly float Time;

		public readonly Vec3 Location;

		public readonly Vec3 Normal;

		public readonly string PlayerName;

		public readonly int PlayerIndex;

		public readonly int Team;

		public BallTouch(Touch touch)
		{
			Time = touch.GameSeconds;
			Location = new Vec3(touch.Location.Value);
			Normal = new Vec3(touch.Normal.Value);
			PlayerName = touch.PlayerName;
			PlayerIndex = touch.PlayerIndex;
			Team = touch.Team;
		}
	}
	public class Boost
	{
		public readonly int Index;

		public readonly Vec3 Location;

		public readonly bool IsLarge;

		public bool IsActive { get; private set; }

		public float TimeUntilActive { get; private set; }

		public Boost(int index)
		{
			Index = index;
			Location = Vec3.Zero;
			IsLarge = false;
			IsActive = true;
			TimeUntilActive = 0f;
		}

		public Boost(int index, BoostPad boostPad)
		{
			Index = index;
			Location = new Vec3(boostPad.Location.Value);
			IsLarge = boostPad.IsFullBoost;
			IsActive = true;
			TimeUntilActive = 0f;
		}

		public void Update(BoostPadState boost)
		{
			IsActive = boost.IsActive;
			TimeUntilActive = boost.Timer;
		}
	}
	public class Car
	{
		public const float JumpMaxDuration = 0.2f;

		public const float JumpAccel = 1458.333f;

		public const float JumpVel = 291.667f;

		public const float AirThrottleAccel = 66.667f;

		public const float BrakeAccel = 3500f;

		public const float BoostAccel = 991.667f;

		public const float BoostConsumption = 33.3f;

		public const float MaxSpeed = 2300f;

		public const float StickyAccel = 325f;

		public const float PitchAngularAccel = 12.46f;

		public const float YawAngularAccel = 9.11f;

		public const float RollAngularAccel = 38.34f;

		public const float MaxAngularVel = 5.5f;

		public Vec3 Location;

		public Vec3 Velocity;

		public Vec3 AngularVelocity;

		public Vec3 LocalAngularVelocity;

		public Vec3 Rotation;

		public Mat3x3 Orientation;

		public string Name;

		public int Index;

		public int Team;

		public int Boost;

		public bool IsBot;

		public bool IsGrounded;

		public bool HasJumped;

		public bool HasDoubleJumped;

		public bool IsDemolished;

		public bool IsSupersonic;

		private Vec3 _hitboxDimensions;

		private Vec3 _hitboxOffset;

		public Hitbox Hitbox => new Hitbox(Location, _hitboxDimensions, _hitboxOffset, Orientation);

		public Vec3 Forward
		{
			get
			{
				return Orientation.Forward;
			}
			set
			{
				Orientation[0] = value;
			}
		}

		public Vec3 Right
		{
			get
			{
				return Orientation.Right;
			}
			set
			{
				Orientation[1] = value;
			}
		}

		public Vec3 Up
		{
			get
			{
				return Orientation.Up;
			}
			set
			{
				Orientation[2] = value;
			}
		}

		public int Score { get; private set; }

		public int Goals { get; private set; }

		public int OwnGoals { get; private set; }

		public int Assists { get; private set; }

		public int Saves { get; private set; }

		public int Shots { get; private set; }

		public int Demolitions { get; private set; }

		public Car()
		{
			Location = Vec3.Zero;
			Velocity = Vec3.Zero;
			AngularVelocity = Vec3.Zero;
			LocalAngularVelocity = Vec3.Zero;
			Rotation = Vec3.Zero;
			Orientation = new Mat3x3(Rotation);
			Name = "";
			Index = 0;
			Team = 0;
			Boost = 0;
			IsBot = true;
			IsGrounded = false;
			HasJumped = false;
			HasDoubleJumped = false;
			IsDemolished = false;
			IsSupersonic = false;
			Score = 0;
			Goals = 0;
			OwnGoals = 0;
			Assists = 0;
			Saves = 0;
			Shots = 0;
			Demolitions = 0;
			_hitboxDimensions = Vec3.Zero;
			_hitboxOffset = Vec3.Zero;
		}

		public Car(Car originalCar)
		{
			Location = originalCar.Location;
			Velocity = originalCar.Velocity;
			AngularVelocity = originalCar.AngularVelocity;
			LocalAngularVelocity = originalCar.LocalAngularVelocity;
			Rotation = originalCar.Rotation;
			Orientation = new Mat3x3(Rotation);
			Name = originalCar.Name;
			Index = originalCar.Index;
			Team = originalCar.Team;
			Boost = originalCar.Boost;
			IsBot = originalCar.IsBot;
			IsGrounded = originalCar.IsGrounded;
			HasJumped = originalCar.HasJumped;
			HasDoubleJumped = originalCar.HasDoubleJumped;
			IsDemolished = originalCar.IsDemolished;
			IsSupersonic = originalCar.IsSupersonic;
			Score = originalCar.Score;
			Goals = originalCar.Goals;
			OwnGoals = originalCar.OwnGoals;
			Assists = originalCar.Assists;
			Saves = originalCar.Saves;
			Shots = originalCar.Shots;
			Demolitions = originalCar.Demolitions;
			_hitboxDimensions = originalCar._hitboxDimensions;
			_hitboxOffset = originalCar._hitboxOffset;
		}

		public Car(int index, PlayerInfo playerInfo)
		{
			Location = (playerInfo.Physics.Value.Location.HasValue ? new Vec3(playerInfo.Physics.Value.Location.Value) : Vec3.Zero);
			Velocity = (playerInfo.Physics.Value.Velocity.HasValue ? new Vec3(playerInfo.Physics.Value.Velocity.Value) : Vec3.Zero);
			AngularVelocity = (playerInfo.Physics.Value.AngularVelocity.HasValue ? new Vec3(playerInfo.Physics.Value.AngularVelocity.Value) : Vec3.Zero);
			Rotation = (playerInfo.Physics.Value.Rotation.HasValue ? new Vec3(playerInfo.Physics.Value.Rotation.Value) : Vec3.Zero);
			Orientation = new Mat3x3(Rotation);
			LocalAngularVelocity = Local(AngularVelocity);
			Name = playerInfo.Name;
			Index = index;
			Team = playerInfo.Team;
			Boost = playerInfo.Boost;
			IsBot = playerInfo.IsBot;
			IsGrounded = playerInfo.HasWheelContact;
			HasJumped = playerInfo.Jumped;
			HasDoubleJumped = playerInfo.DoubleJumped;
			IsDemolished = playerInfo.IsDemolished;
			IsSupersonic = playerInfo.IsSupersonic;
			Score = (playerInfo.ScoreInfo.HasValue ? playerInfo.ScoreInfo.Value.Score : 0);
			Goals = (playerInfo.ScoreInfo.HasValue ? playerInfo.ScoreInfo.Value.Goals : 0);
			OwnGoals = (playerInfo.ScoreInfo.HasValue ? playerInfo.ScoreInfo.Value.OwnGoals : 0);
			Assists = (playerInfo.ScoreInfo.HasValue ? playerInfo.ScoreInfo.Value.Assists : 0);
			Saves = (playerInfo.ScoreInfo.HasValue ? playerInfo.ScoreInfo.Value.Saves : 0);
			Shots = (playerInfo.ScoreInfo.HasValue ? playerInfo.ScoreInfo.Value.Shots : 0);
			Demolitions = (playerInfo.ScoreInfo.HasValue ? playerInfo.ScoreInfo.Value.Demolitions : 0);
			_hitboxDimensions = (playerInfo.Hitbox.HasValue ? new Vec3(playerInfo.Hitbox.Value.Length, playerInfo.Hitbox.Value.Width, playerInfo.Hitbox.Value.Height) : Vec3.Zero);
			_hitboxOffset = (playerInfo.HitboxOffset.HasValue ? new Vec3(playerInfo.HitboxOffset.Value) : Vec3.Zero);
		}

		public void Update(PlayerInfo playerInfo)
		{
			Location = (playerInfo.Physics.Value.Location.HasValue ? new Vec3(playerInfo.Physics.Value.Location.Value) : Location);
			Velocity = (playerInfo.Physics.Value.Velocity.HasValue ? new Vec3(playerInfo.Physics.Value.Velocity.Value) : Velocity);
			AngularVelocity = (playerInfo.Physics.Value.AngularVelocity.HasValue ? new Vec3(playerInfo.Physics.Value.AngularVelocity.Value) : AngularVelocity);
			Rotation = (playerInfo.Physics.Value.Rotation.HasValue ? new Vec3(playerInfo.Physics.Value.Rotation.Value) : Rotation);
			Orientation = new Mat3x3(Rotation);
			LocalAngularVelocity = Local(AngularVelocity);
			Boost = playerInfo.Boost;
			IsBot = playerInfo.IsBot;
			IsGrounded = playerInfo.HasWheelContact;
			HasJumped = playerInfo.Jumped;
			HasDoubleJumped = playerInfo.DoubleJumped;
			IsDemolished = playerInfo.IsDemolished;
			IsSupersonic = playerInfo.IsSupersonic;
			Score = (playerInfo.ScoreInfo.HasValue ? playerInfo.ScoreInfo.Value.Score : Score);
			Goals = (playerInfo.ScoreInfo.HasValue ? playerInfo.ScoreInfo.Value.Goals : Goals);
			OwnGoals = (playerInfo.ScoreInfo.HasValue ? playerInfo.ScoreInfo.Value.OwnGoals : OwnGoals);
			Assists = (playerInfo.ScoreInfo.HasValue ? playerInfo.ScoreInfo.Value.Assists : Assists);
			Saves = (playerInfo.ScoreInfo.HasValue ? playerInfo.ScoreInfo.Value.Saves : Saves);
			Shots = (playerInfo.ScoreInfo.HasValue ? playerInfo.ScoreInfo.Value.Shots : Shots);
			Demolitions = (playerInfo.ScoreInfo.HasValue ? playerInfo.ScoreInfo.Value.Demolitions : Demolitions);
			_hitboxDimensions = (playerInfo.Hitbox.HasValue ? new Vec3(playerInfo.Hitbox.Value.Length, playerInfo.Hitbox.Value.Width, playerInfo.Hitbox.Value.Height) : _hitboxDimensions);
			_hitboxOffset = (playerInfo.HitboxOffset.HasValue ? new Vec3(playerInfo.HitboxOffset.Value) : _hitboxOffset);
		}

		public Vec3 Local(Vec3 vec)
		{
			return Orientation.Dot(vec);
		}

		public float PredictLandingTime()
		{
			if (IsGrounded)
			{
				return 0f;
			}
			float num = Utils.Quadratic(Game.Gravity.z / 2f, Velocity.z, Location.z - 15f)[1];
			float num2 = Utils.Quadratic(Game.Gravity.z / 2f, Velocity.z, Location.z + 15f - 1950f)[1];
			float num3 = ((num2 < 0f) ? num : num2);
			if (!Field.InField(PredictLocation(num3), 150f))
			{
				Surface surface = Field.FindLandingSurface(this);
				num3 = MathF.Max((Location - surface.Limit(Location)).Dot(surface.Normal) / Velocity.Dot(-surface.Normal), 0f);
			}
			return num3;
		}

		public Vec3 PredictLandingPosition()
		{
			return PredictLocation(PredictLandingTime());
		}

		public Car Predict(float time)
		{
			return new Car(this)
			{
				Location = Location + Velocity * time + Game.Gravity * 0.5f * time * time,
				Velocity = Velocity + Game.Gravity * time
			};
		}

		public Vec3 PredictLocation(float time)
		{
			return Location + Velocity * time + Game.Gravity * 0.5f * time * time;
		}

		public Vec3 PredictVelocity(float time)
		{
			return Velocity + Game.Gravity * time;
		}

		public Vec3 LocationAfterDodge()
		{
			return Location + (Velocity + Velocity.FlatNorm() * 500f).Cap(0f, 2300f) * 1.3f;
		}

		public Vec3 LocationAfterSpeedFlip(bool isBoosting = false)
		{
			if (isBoosting)
			{
				float num = MathF.Min((float)Boost / 33.3f, 1.3f);
				return Location + (Velocity + Velocity.FlatNorm() * 500f + Velocity.FlatNorm() * 991.667f * num * (1.3f - 0.5f * num)).Cap(0f, 2300f) * 1.3f;
			}
			return Location + (Velocity + Velocity.FlatNorm() * 500f).Cap(0f, 2300f) * 1.3f;
		}

		public Vec3 LocationAfterJump(float time, float elapsed)
		{
			float num = Utils.Cap(0.2f - elapsed, 0f, 0.2f);
			float num2 = Utils.Cap(0.05f - elapsed, 0f, 0.05f);
			return Location + Velocity * time + Game.Gravity * 0.5f * MathF.Pow(time, 2f) + (IsGrounded ? (Up * 291.667f * time) : Vec3.Zero) + Up * 1458.333f * num * (time - 0.5f * num) - Up * 325f * num2 * (time - 0.5f * num2);
		}

		public Vec3 LocationAfterDoubleJump(float time, float elapsed)
		{
			float num = Utils.Cap(0.2f - elapsed, 0f, 0.2f);
			float num2 = Utils.Cap(0.05f - elapsed, 0f, 0.05f);
			return Location + Velocity * time + Game.Gravity * 0.5f * MathF.Pow(time, 2f) + (IsGrounded ? (Up * 291.667f * time) : Vec3.Zero) + Up * 1458.333f * num * (time - 0.5f * num) - Up * 325f * num2 * (time - 0.5f * num2) + ((!HasDoubleJumped) ? (Up * 291.667f * (time - num)) : Vec3.Zero);
		}

		public Vec3 VelocityAfterJump(float time, float elapsed)
		{
			float num = Utils.Cap(0.2f - elapsed, 0f, 0.2f);
			float num2 = Utils.Cap(0.05f - elapsed, 0f, 0.05f);
			return Velocity + Game.Gravity * time + Up * (IsGrounded ? 291.667f : 0f) + Up * 1458.333f * num - Up * 325f * num2;
		}

		public Vec3 VelocityAfterDoubleJump(float time, float elapsed)
		{
			float num = Utils.Cap(0.2f - elapsed, 0f, 0.2f);
			float num2 = Utils.Cap(0.05f - elapsed, 0f, 0.05f);
			return Velocity + Game.Gravity * time + Up * 291.667f * ((!IsGrounded) ? 1 : 2) + Up * 1458.333f * num - Up * 325f * num2;
		}
	}
	public static class Cars
	{
		public static int Count => AllCars.Count;

		public static List<Car> AllCars { get; private set; }

		public static List<Car> AllLivingCars => AllCars.FindAll((Car car) => !car.IsDemolished);

		public static List<Car> BlueCars => AllCars.FindAll((Car car) => car.Team == 0);

		public static List<Car> LivingBlueCars => AllCars.FindAll((Car car) => !car.IsDemolished && car.Team == 0);

		public static List<Car> OrangeCars => AllCars.FindAll((Car car) => car.Team == 1);

		public static List<Car> LivingOrangeCars => AllCars.FindAll((Car car) => !car.IsDemolished && car.Team == 1);

		public static void Initialize(GameTickPacket packet)
		{
			AllCars = new List<Car>();
			for (int i = 0; i < packet.PlayersLength; i++)
			{
				AllCars.Add(new Car(i, packet.Players(i).Value));
			}
		}

		public static void Update(GameTickPacket packet)
		{
			foreach (Car allCar in AllCars)
			{
				allCar.Update(packet.Players(allCar.Index).Value);
			}
		}
	}
	public static class Field
	{
		public const float Length = 10240f;

		public const float Width = 8192f;

		public const float Height = 1950f;

		public const float CornerIntersection = 8064f;

		public const float CornerLength = 1629f;

		public const float CornerWidth = 1152f;

		public static Goal[] Goals { get; private set; }

		public static Goal BlueGoal => Goals[0];

		public static Goal OrangeGoal => Goals[1];

		public static List<Boost> Boosts { get; private set; }

		public static Dictionary<string, Surface> Surfaces { get; private set; }

		public static Dictionary<string, Surface> DrivableSurfaces { get; private set; }

		static Field()
		{
			Goals = new Goal[2]
			{
				new Goal(0),
				new Goal(1)
			};
			Boosts = new List<Boost>();
			Surfaces = new Dictionary<string, Surface>
			{
				{
					"Ground",
					new Surface("Ground", new Vec3(0f, 0f, 0f), Vec3.Up, new Vec3(8192f, 10240f), Vec3.X, Vec3.Y)
				},
				{
					"Ceiling",
					new Surface("Ceiling", new Vec3(0f, 0f, 1950f), -Vec3.Up, new Vec3(8192f, 10240f), Vec3.X, Vec3.Y)
				},
				{
					"Orange Backboard",
					new Surface("Orange Backboard", new Vec3(0f, 5120f, 1295f), new Vec3(0f, -1f, 0f), new Vec3(1780f, 1310f), Vec3.X, Vec3.Z)
				},
				{
					"Orange Right Backwall",
					new Surface("Orange Right Backwall", new Vec3(1917f, 5120f, 975f), new Vec3(0f, -1f, 0f), new Vec3(2054f, 1950f), Vec3.X, Vec3.Z)
				},
				{
					"Orange Left Backwall",
					new Surface("Orange Left Backwall", new Vec3(-1917f, 5120f, 975f), new Vec3(0f, -1f, 0f), new Vec3(2054f, 1950f), Vec3.X, Vec3.Z)
				},
				{
					"Blue Backboard",
					new Surface("Blue Backboard", new Vec3(0f, -5120f, 1295f), new Vec3(0f, 1f, 0f), new Vec3(1780f, 1310f), Vec3.X, Vec3.Z)
				},
				{
					"Blue Right Backwall",
					new Surface("Blue Right Backwall", new Vec3(-1917f, -5120f, 975f), new Vec3(0f, 1f, 0f), new Vec3(2054f, 1950f), Vec3.X, Vec3.Z)
				},
				{
					"Blue Left Backwall",
					new Surface("Blue Left Backwall", new Vec3(1917f, -5120f, 975f), new Vec3(0f, 1f, 0f), new Vec3(2054f, 1950f), Vec3.X, Vec3.Z)
				},
				{
					"Right Orange Sidewall",
					new Surface("Right Orange Sidewall", new Vec3(4096f, 0f, 975f), new Vec3(-1f, 0f, 0f), new Vec3(1950f, 10240f), Vec3.Z, Vec3.Y)
				},
				{
					"Right Blue Sidewall",
					new Surface("Right Blue Sidewall", new Vec3(-4096f, 0f, 975f), new Vec3(1f, 0f, 0f), new Vec3(1950f, 10240f), Vec3.Z, Vec3.Y)
				},
				{
					"Right Orange Corner",
					new Surface("Right Orange Corner", new Vec3(3520f, 4544f, 975f), new Vec3((0f - MathF.Sqrt(2f)) / 2f, (0f - MathF.Sqrt(2f)) / 2f, 0f), new Vec3(1629f, 1950f), new Vec3((0f - MathF.Sqrt(2f)) / 2f, MathF.Sqrt(2f) / 2f, 0f), Vec3.Z)
				},
				{
					"Left Orange Corner",
					new Surface("Left Orange Corner", new Vec3(-3520f, 4544f, 975f), new Vec3(MathF.Sqrt(2f) / 2f, (0f - MathF.Sqrt(2f)) / 2f, 0f), new Vec3(1629f, 1950f), new Vec3(MathF.Sqrt(2f) / 2f, MathF.Sqrt(2f) / 2f, 0f), Vec3.Z)
				},
				{
					"Right Blue Corner",
					new Surface("Right Blue Corner", new Vec3(-3520f, -4544f, 975f), new Vec3(MathF.Sqrt(2f) / 2f, MathF.Sqrt(2f) / 2f, 0f), new Vec3(1629f, 1950f), new Vec3(MathF.Sqrt(2f) / 2f, (0f - MathF.Sqrt(2f)) / 2f, 0f), Vec3.Z)
				},
				{
					"Left Blue Corner",
					new Surface("Left Blue Corner", new Vec3(3520f, -4544f, 975f), new Vec3((0f - MathF.Sqrt(2f)) / 2f, MathF.Sqrt(2f) / 2f, 0f), new Vec3(1629f, 1950f), new Vec3((0f - MathF.Sqrt(2f)) / 2f, (0f - MathF.Sqrt(2f)) / 2f, 0f), Vec3.Z)
				},
				{
					"Orange Goal Ground",
					new Surface("Orange Goal Ground", new Vec3(0f, 5545f, 0f), Vec3.Up, new Vec3(1780f, 850f), Vec3.X, Vec3.Y)
				},
				{
					"Blue Goal Ground",
					new Surface("Blue Goal Ground", new Vec3(0f, -5545f, 0f), Vec3.Up, new Vec3(1780f, 850f), Vec3.X, Vec3.Y)
				}
			};
			DrivableSurfaces = new Dictionary<string, Surface>(Surfaces);
			DrivableSurfaces.Remove("Ceiling");
		}

		public static void Initialize(rlbot.flat.FieldInfo fieldInfo)
		{
			for (int i = 0; i < fieldInfo.BoostPadsLength; i++)
			{
				if (fieldInfo.BoostPads(i).HasValue)
				{
					Boosts.Add(new Boost(i, fieldInfo.BoostPads(i).Value));
				}
				else
				{
					Boosts.Add(new Boost(i));
				}
			}
		}

		public static void Update(GameTickPacket packet)
		{
			for (int i = 0; i < packet.BoostPadStatesLength; i++)
			{
				if (packet.BoostPadStates(i).HasValue)
				{
					Boosts[i].Update(packet.BoostPadStates(i).Value);
				}
			}
		}

		public static int Side(int team)
		{
			return 2 * team - 1;
		}

		public static bool InField(Vec3 pos, float radius)
		{
			Vec3 vec = Vec3.Abs(pos);
			if (vec.x > 4096f - radius)
			{
				return false;
			}
			if (vec.y > 5970f - radius)
			{
				return false;
			}
			if ((vec.x > 890f - radius || vec.z > 640f - radius) && vec.y > 5120f - radius)
			{
				return false;
			}
			if (vec.x + vec.y > 8064f - radius)
			{
				return false;
			}
			return true;
		}

		public static bool InGoal(Vec3 pos, float radius)
		{
			Vec3 vec = Vec3.Abs(pos);
			if (vec.x < 890f + radius && vec.y > 5120f - radius && vec.y < 5970f + radius)
			{
				return vec.z < 640f + radius;
			}
			return false;
		}

		public static bool OnSeam(Vec3 pos, float radius)
		{
			Surface surface = NearestSurface(pos);
			Surface surface2 = NearestSurface(pos, new Surface[1] { surface });
			if (surface.Limit(pos).Dist(surface2.Limit(pos)) < radius)
			{
				return surface.Normal.Dot(surface2.Normal) < 0.95f;
			}
			return false;
		}

		public static Surface NearestSurface(Vec3 pos)
		{
			Surface surface = DrivableSurfaces.First().Value;
			foreach (Surface value in DrivableSurfaces.Values)
			{
				if (pos.Dist(surface.Limit(pos)) > pos.Dist(value.Limit(pos)))
				{
					surface = value;
				}
			}
			return surface;
		}

		public static Surface NearestSurface(Vec3 pos, Surface[] excludedSurfaces)
		{
			List<Surface> list = new List<Surface>(Surfaces.Values);
			foreach (Surface item in excludedSurfaces)
			{
				list.Remove(item);
			}
			Surface surface = list.First();
			foreach (Surface item2 in list)
			{
				if (pos.Dist(surface.Limit(pos)) > pos.Dist(item2.Limit(pos)))
				{
					surface = item2;
				}
			}
			return surface;
		}

		public static Surface BestJumpingSurface(Vec3 carPos, Vec3 targetPos)
		{
			Surface surface = NearestSurface(carPos);
			if (surface.Limit(targetPos).Dist(targetPos) < 300f)
			{
				return surface;
			}
			return NearestSurface(targetPos);
		}

		public static Surface FindLandingSurface(Car car)
		{
			if (car.IsGrounded)
			{
				return NearestSurface(car.Location, Array.Empty<Surface>());
			}
			float num = Utils.Quadratic(Game.Gravity.z / 2f, car.Velocity.z, car.Location.z - 15f)[1];
			float num2 = Utils.Quadratic(Game.Gravity.z / 2f, car.Velocity.z, car.Location.z + 15f - 1950f)[1];
			Surface result = ((num2 < 0f) ? Surfaces["Ground"] : Surfaces["Ceiling"]);
			float num3 = ((num2 < 0f) ? num : num2);
			if (!InField(car.PredictLocation(num3), 150f))
			{
				foreach (Surface value in DrivableSurfaces.Values)
				{
					if (!(value.Key == "Ground"))
					{
						float num4 = car.Location.Dist(value.Limit(car.Location)) / car.Velocity.Dot(-value.Normal);
						if (num4 > 0f && num4 < num3)
						{
							num3 = num4;
							result = value;
						}
					}
				}
			}
			return result;
		}

		public static Vec3 LimitToNearestSurface(Vec3 pos)
		{
			return NearestSurface(pos).Limit(pos);
		}

		public static Vec3 LimitToNearestSurface(Vec3 pos, Surface[] excludedSurfaces)
		{
			return NearestSurface(pos, excludedSurfaces).Limit(pos);
		}

		public static float DistanceBetweenPoints(Vec3 pos1, Vec3 pos2)
		{
			Surface surface = NearestSurface(pos1);
			Vec3 vec = surface.Limit(pos1);
			Vec3 vec2 = NearestSurface(pos2).Limit(pos2);
			Surface surface2 = NearestSurface((vec + vec2) / 2f);
			Vec3 pos3 = surface2.Limit((vec + vec2) / 2f);
			return vec.Dist(surface.Limit(pos3)) + surface.Limit(pos3).Dist(surface2.Limit(vec2)) + surface2.Limit(vec2).Dist(vec2);
		}
	}
	public class Surface
	{
		public string Key { get; private set; }

		public Vec3 Normal { get; private set; }

		public Vec3 Location { get; private set; }

		public Vec3 Size { get; private set; }

		public Vec3 Xdirection { get; private set; }

		public Vec3 Ydirection { get; private set; }

		public Surface(string key, Vec3 location, Vec3 normal, Vec3 size, Vec3 xDirection, Vec3 yDirection)
		{
			Key = key;
			Location = location;
			Normal = normal;
			Size = size;
			Xdirection = xDirection;
			Ydirection = yDirection;
		}

		public Vec3 Limit(Vec3 pos)
		{
			Vec3 v = pos - Location;
			return Location + Xdirection * Utils.Cap(Xdirection.Dot(v), (0f - Size.x) / 2f, Size.x / 2f) + Ydirection * Utils.Cap(Ydirection.Dot(v), (0f - Size.y) / 2f, Size.y / 2f);
		}
	}
	public static class Game
	{
		public static int[] Scores { get; private set; }

		public static int BlueScore => Scores[1];

		public static int OrangeScore => Scores[0];

		public static float Time { get; private set; }

		public static float TimeRemaining { get; private set; }

		public static float GameSpeed { get; private set; }

		public static bool IsUnlimitedTime { get; private set; }

		public static bool IsOvertime { get; private set; }

		public static bool IsRoundActive { get; private set; }

		public static bool IsKickoffPause { get; private set; }

		public static bool IsMatchEnded { get; private set; }

		public static Vec3 Gravity { get; private set; }

		static Game()
		{
			Scores = new int[2];
			Time = 0f;
			TimeRemaining = 300f;
			GameSpeed = 1f;
			IsUnlimitedTime = false;
			IsOvertime = false;
			IsRoundActive = false;
			IsKickoffPause = false;
			IsMatchEnded = false;
			Gravity = new Vec3(0f, 0f, -650f);
		}

		public static void Update(GameTickPacket packet)
		{
			Scores = new int[2]
			{
				packet.Teams(0).Value.Score,
				packet.Teams(1).Value.Score
			};
			Time = packet.GameInfo.Value.SecondsElapsed;
			TimeRemaining = packet.GameInfo.Value.GameTimeRemaining;
			GameSpeed = packet.GameInfo.Value.GameSpeed;
			IsUnlimitedTime = packet.GameInfo.Value.IsUnlimitedTime;
			IsOvertime = packet.GameInfo.Value.IsOvertime;
			IsRoundActive = packet.GameInfo.Value.IsRoundActive;
			IsKickoffPause = packet.GameInfo.Value.IsKickoffPause;
			IsMatchEnded = packet.GameInfo.Value.IsMatchEnded;
			Gravity = new Vec3(0f, 0f, packet.GameInfo.Value.WorldGravityZ);
		}
	}
	public class Goal
	{
		public const float Width = 1780f;

		public const float Height = 640f;

		public const float Depth = 850f;

		public readonly int Team;

		public readonly Vec3 Location;

		public readonly Vec3 LeftPost;

		public readonly Vec3 RightPost;

		public readonly Vec3 TopLeftCorner;

		public readonly Vec3 TopRightCorner;

		public readonly Vec3 BottomLeftCorner;

		public readonly Vec3 BottomRightCorner;

		public readonly Vec3 Crossbar;

		public Goal(int team)
		{
			Team = team;
			Location = new Vec3(0f, 5120f * (float)Field.Side(Team), 0f);
			LeftPost = new Vec3(890f * (float)Field.Side(Team), 5120f * (float)Field.Side(Team), 320f);
			RightPost = new Vec3(890f * (float)(-Field.Side(Team)), 5120f * (float)Field.Side(Team), 320f);
			TopLeftCorner = new Vec3(890f * (float)Field.Side(Team), 5120f * (float)Field.Side(Team), 640f);
			TopRightCorner = new Vec3(890f * (float)(-Field.Side(Team)), 5120f * (float)Field.Side(Team), 640f);
			BottomLeftCorner = new Vec3(890f * (float)Field.Side(Team), 5120f * (float)Field.Side(Team), 0f);
			BottomRightCorner = new Vec3(890f * (float)(-Field.Side(Team)), 5120f * (float)Field.Side(Team), 0f);
			Crossbar = new Vec3(0f, 5120f * (float)Field.Side(Team), 640f);
		}
	}
	public class Hitbox
	{
		public Vec3 Location;

		public Vec3 Dimensions;

		public Vec3 Offset;

		public Mat3x3 Orientation;

		public float Length
		{
			get
			{
				return Dimensions[0];
			}
			private set
			{
				Dimensions[0] = value;
			}
		}

		public float Width
		{
			get
			{
				return Dimensions[1];
			}
			private set
			{
				Dimensions[1] = value;
			}
		}

		public float Height
		{
			get
			{
				return Dimensions[2];
			}
			private set
			{
				Dimensions[2] = value;
			}
		}

		public Vec3 Center => Location + Offset.Dot(Orientation);

		public Hitbox(Vec3 location, Vec3 dimensions, Vec3 offset, Mat3x3 orientation)
		{
			Location = location;
			Dimensions = dimensions;
			Offset = offset;
			Orientation = orientation;
		}

		public Vec3 GetNearestPoint(Vec3 pos)
		{
			Vec3 vec = Orientation.Dot(pos - Center);
			Vec3 vec2 = new Vec3(Utils.Cap(vec.x, (0f - Length) / 2f, Length / 2f), Utils.Cap(vec.y, (0f - Width) / 2f, Width / 2f), Utils.Cap(vec.z, (0f - Height) / 2f, Height / 2f));
			return Center + vec2.Dot(Orientation);
		}

		public bool IsTouching(Vec3 pos)
		{
			return GetNearestPoint(pos).Dist(pos) < 93.15f;
		}

		public float GetOffset(Vec3 direction)
		{
			direction = direction.Normalize();
			Vec3 vec = Orientation.Dot(direction * 500f);
			float num = Utils.Cap(vec.x, (0f - Length) / 2f + Offset.x - 93.15f, Length / 2f + Offset.x + 93.15f);
			vec *= ((num == vec.x) ? 1f : (num / vec.x));
			float num2 = Utils.Cap(vec.y, (0f - Width) / 2f - 93.15f, Width / 2f + 93.15f);
			vec *= ((num2 == vec.y) ? 1f : (num2 / vec.y));
			float num3 = Utils.Cap(vec.z, (0f - Height) / 2f + Offset.z - 93.15f, Height / 2f + Offset.z + 93.15f);
			vec *= ((num3 == vec.z) ? 1f : (num3 / vec.z));
			Vec3 vec2 = vec.Dot(Orientation);
			Vec3 nearestPoint = GetNearestPoint(Location + vec2);
			if (nearestPoint.Dist(Location + vec2) > 93.15f)
			{
				Vec3 vec3 = vec2.Rescale(vec2.Normalize().Dot(nearestPoint - Location));
				float num4 = Utils.Cap(vec3.Dist(nearestPoint - Location), 0f, 93.15f);
				return vec3.Length() + MathF.Sin(MathF.Acos(num4 / 93.15f)) * 93.15f;
			}
			return vec.Length();
		}

		public Vec3 GroundOffset(float height, Vec3 direction)
		{
			direction = direction.FlatNorm(Orientation.Up);
			Vec3 vec = Orientation.Dot(direction * 500f);
			float num = Height / 2f + Offset.z + 93.15f;
			height = MathF.Min(height - 10f, num);
			bool flag = height == num;
			float num2 = Utils.Cap(vec.x, (0f - Length) / 2f + Offset.x - 93.15f, Length / 2f + Offset.x + 93.15f);
			vec *= ((num2 == vec.x) ? 1f : (num2 / vec.x));
			float num3 = Utils.Cap(vec.y, (0f - Width) / 2f - 93.15f, Width / 2f + 93.15f);
			vec *= ((num3 == vec.y) ? 1f : (num3 / vec.y));
			Vec3 vec2 = Orientation.Up * height + vec.Dot(Orientation);
			Vec3 nearestPoint = GetNearestPoint(Location + vec2);
			float num4 = MathF.Abs((Location + vec2 - nearestPoint).Dot(Orientation.Up));
			float num5 = MathF.Sqrt(MathF.Max(8676.923f - num4 * num4, 0f));
			Vec3 vec3 = direction * direction.Dot(nearestPoint - Location);
			float x = MathF.Acos(Utils.Cap(vec3.Dist(nearestPoint - Location) / num5, 0f, 1f));
			float num6 = vec3.Length() + MathF.Sin(x) * num5;
			if (flag && MathF.Asin(direction.Dot(Orientation.Up)) > MathF.Atan2(height, num6))
			{
				return direction * GetOffset(direction);
			}
			return Orientation.Up * height + direction * num6;
		}

		public Vec3 DirectionFromNormal(Vec3 normal)
		{
			normal = normal.Normalize();
			normal = new Vec3((MathF.Abs(normal.x) < 0.2f) ? 0f : normal.x, (MathF.Abs(normal.y) < 0.2f) ? 0f : normal.y, (MathF.Abs(normal.z) < 0.2f) ? 0f : normal.z).Normalize();
			return (GetNearestPoint(Center + normal * 1000f) - Location + normal * 93.15f).Normalize();
		}

		public Vec3 OffsetContactPoint(Vec3 direction)
		{
			return Location + direction * GetOffset(direction);
		}

		public Vec3 OffsetContactNormal(Vec3 direction)
		{
			Vec3 vec = OffsetContactPoint(direction);
			return (vec - GetNearestPoint(vec)).Normalize();
		}
	}
	public class Target
	{
		private const float MinError = 100f;

		public Vec3 TopLeft;

		public Vec3 BottomRight;

		public Surface TargetSurface;

		public Target(Vec3 topLeft, Vec3 bottomRight)
		{
			TopLeft = topLeft;
			BottomRight = bottomRight;
			TargetSurface = new Surface("target", (topLeft + BottomRight) / 2f, (TopLeft - BottomRight).Cross(Vec3.Up).Normalize(), new Vec3(MathF.Max(TopLeft.FlatDist(BottomRight) - 186.3f, 1f), MathF.Max(MathF.Abs(TopLeft.z - BottomRight.z) - 186.3f, 1f)), (TopLeft - BottomRight).FlatNorm(), Vec3.Up);
		}

		public Target(Goal goal)
		{
			TopLeft = goal.TopLeftCorner;
			BottomRight = goal.BottomRightCorner;
			TargetSurface = new Surface("target", (TopLeft + BottomRight) / 2f + Vec3.Y * Field.Side(goal.Team) * 93.15f, -Vec3.Y * Field.Side(goal.Team), new Vec3(MathF.Max(TopLeft.FlatDist(BottomRight) - 186.3f, 1f), MathF.Max(MathF.Abs(TopLeft.z - BottomRight.z) - 186.3f, 1f)), (TopLeft - BottomRight).FlatNorm(), Vec3.Up);
		}

		public bool Fits(Vec3 ballLocation)
		{
			Vec3 v = (BottomRight - TopLeft).FlatNorm();
			GetCorrectedLimits(ballLocation, out var correctedLeft, out var correctedRight, out var correctedTop, out var correctedBottom);
			float num = correctedLeft.FlatDist(correctedRight);
			float num2 = MathF.Abs(correctedLeft.z - correctedRight.z);
			Vec3 vec = (correctedLeft - correctedRight).Cross(Vec3.Up).Normalize();
			Vec3 vec2 = (correctedTop - correctedBottom).Cross(v).Normalize();
			Vec3 vec3 = (correctedLeft + correctedRight) / 2f - ballLocation;
			Vec3 vec4 = (correctedTop + correctedBottom) / 2f - ballLocation;
			if ((!(vec3.Normalize().Dot(-vec) * num > 100f) && !(vec3.Length() < num / 2f)) || (!(vec4.Normalize().Dot(-vec2) * num2 > 100f) && !(vec4.Length() < num2 / 2f)))
			{
				return TopLeft.FlatDirection(BottomRight).Cross(Vec3.Up).Dot(ballLocation.FlatDirection((TopLeft + BottomRight) / 2f)) < 0f;
			}
			return true;
		}

		public Vec3 Clamp(Ball ball)
		{
			Vec3 up = (BottomRight - TopLeft).FlatNorm();
			GetCorrectedLimits(ball.location, out var correctedLeft, out var correctedRight, out var correctedTop, out var correctedBottom);
			if (up.Cross(-Vec3.Up).Dot(ball.location - (TopLeft + BottomRight) / 2f) > 0f)
			{
				float time = MathF.Abs((ball.location - TopLeft).Dot(up.Cross(-Vec3.Up)) / ball.velocity.Dot(up.Cross(-Vec3.Up)));
				Vec3 vec = (TargetSurface.Limit(ball.PredictLocation(time)) - ball.location).Normalize().Clamp((correctedLeft - ball.location).Normalize(), (correctedRight - ball.location).Normalize()).Clamp((correctedTop - ball.location).Normalize(), (correctedBottom - ball.location).Normalize(), up);
				return TargetSurface.Limit(ball.location + vec * MathF.Abs((ball.location - TopLeft).Dot(up.Cross(-Vec3.Up)) / vec.Dot(up.Cross(-Vec3.Up))));
			}
			Vec3 vec2 = ball.velocity.Normalize().Clamp((correctedLeft - ball.location).Normalize(), (correctedRight - ball.location).Normalize()).Clamp((correctedTop - ball.location).Normalize(), (correctedBottom - ball.location).Normalize(), up);
			return ball.location + vec2.Normalize() * 2000f;
		}

		public Vec3 Clamp(Ball ball, Vec3 dir)
		{
			Vec3 up = (BottomRight - TopLeft).FlatNorm();
			GetCorrectedLimits(ball.location, out var correctedLeft, out var correctedRight, out var correctedTop, out var correctedBottom);
			if (up.Cross(-Vec3.Up).Dot(ball.location - (TopLeft + BottomRight) / 2f) > 0f)
			{
				Vec3 vec = dir.Normalize().Clamp((correctedLeft - ball.location).Normalize(), (correctedRight - ball.location).Normalize()).Clamp((correctedTop - ball.location).Normalize(), (correctedBottom - ball.location).Normalize(), up);
				return TargetSurface.Limit(ball.location + vec * MathF.Abs((ball.location - TopLeft).Dot(up.Cross(-Vec3.Up)) / vec.Dot(up.Cross(-Vec3.Up))));
			}
			Vec3 vec2 = dir.Normalize().Clamp((correctedLeft - ball.location).Normalize(), (correctedRight - ball.location).Normalize()).Clamp((correctedTop - ball.location).Normalize(), (correctedBottom - ball.location).Normalize(), up);
			return ball.location + vec2.Normalize() * 1000f;
		}

		public void GetCorrectedLimits(Vec3 ballLocation, out Vec3 correctedLeft, out Vec3 correctedRight, out Vec3 correctedTop, out Vec3 correctedBottom)
		{
			Vec3 vec = (BottomRight - TopLeft).FlatNorm();
			Vec3 vec2 = TopLeft + (TopLeft - ballLocation).Normalize().Rotate(0f - MathF.Asin(93.15f / TopLeft.FlatDist(ballLocation))).Cross(-Vec3.Up)
				.Normalize() * 93.15f;
			Vec3 vec3 = BottomRight + (BottomRight - ballLocation).Normalize().Rotate(MathF.Asin(93.15f / BottomRight.FlatDist(ballLocation))).Cross(Vec3.Up)
				.Normalize() * 93.15f;
			Vec3 vec4 = (TopLeft + BottomRight).Flatten() / 2f + Vec3.Up * TopLeft.z;
			Vec3 vec5 = vec4 + (TopLeft - ballLocation).Normalize().Rotate(0f - MathF.Asin(93.15f / TopLeft.FlatDist(ballLocation, vec)), vec).Cross(-vec)
				.Normalize() * 93.15f;
			Vec3 vec6 = (TopLeft + BottomRight).Flatten() / 2f + Vec3.Up * BottomRight.z;
			Vec3 vec7 = vec6 + (BottomRight - ballLocation).Normalize().Rotate(MathF.Asin(93.15f / BottomRight.FlatDist(ballLocation, vec)), vec).Cross(vec)
				.Normalize() * 93.15f;
			correctedLeft = (((TopLeft - ballLocation).Flatten().Dot((TopLeft - BottomRight).Flatten()) > 0f) ? TopLeft : vec2);
			correctedRight = (((BottomRight - ballLocation).Flatten().Dot((BottomRight - TopLeft).Flatten()) > 0f) ? BottomRight : vec3);
			correctedTop = (((TopLeft - ballLocation).Flatten(vec).Dot(Vec3.Up) > 0f) ? vec4 : vec5);
			correctedBottom = (((BottomRight - ballLocation).Flatten(vec).Dot(-Vec3.Up) > 0f) ? vec6 : vec7);
		}
	}
	public static class Utils
	{
		public static float Cap(float value, float min, float max)
		{
			if (!(value < min))
			{
				if (!(value > max))
				{
					return value;
				}
				return max;
			}
			return min;
		}

		public static int Cap(int value, int min, int max)
		{
			if (value >= min)
			{
				if (value <= max)
				{
					return value;
				}
				return max;
			}
			return min;
		}

		public static float Lerp(float t, float a, float b)
		{
			return (b - a) * t + a;
		}

		public static Vec3 Lerp(float t, Vec3 a, Vec3 b)
		{
			return (b - a) * t + a;
		}

		public static float Invlerp(float v, float a, float b)
		{
			return (v - a) / (b - a);
		}

		public static float[] Quadratic(float a, float b, float c)
		{
			float num = MathF.Sqrt(b * b - 4f * a * c);
			if (a != 0f && !float.IsNaN(num))
			{
				return new float[2]
				{
					(0f - b + num) / (2f * a),
					(0f - b - num) / (2f * a)
				};
			}
			return new float[2] { -1f, -1f };
		}

		public static float TimeToJump(Vec3 up, float height, bool doubleJump = false)
		{
			float num = ((up.Dot(Game.Gravity) != 0f) ? up.Dot(Game.Gravity) : (-0.001f));
			float num2 = 84.65631f + num * 0.2f * 0.2f / 2f;
			float num3 = ((!doubleJump) ? 1 : 2);
			float num4 = 291.667f * num3 - 16.25f + (num + 1458.333f) * 0.2f;
			float num5 = MathF.Sqrt(MathF.Max(MathF.Pow(num4, 2f) + 2f * num * (height - num2), 0f));
			if (height < num2)
			{
				return (MathF.Sqrt(MathF.Max(MathF.Pow(275.417f, 2f) + 2f * (num + 1458.333f) * height, 0f)) - 291.667f + 16.25f) / (num + 1458.333f);
			}
			return 0.2f + (num5 - num4) / num;
		}

		public static float PredictRotation(float acceleration, float time)
		{
			if (acceleration * time > 5.5f)
			{
				float num = 5.5f / acceleration;
				return acceleration / 2f * MathF.Pow(num, 2f) + 5.5f * (time - num);
			}
			return acceleration / 2f * MathF.Pow(time, 2f);
		}

		public static float ShotPowerModifier(float value)
		{
			value = Cap(value, 0f, 4600f);
			if (value <= 500f)
			{
				return 0.65f;
			}
			if (value <= 2300f)
			{
				return Lerp((value - 500f) / 1800f, 0.65f, 0.55f);
			}
			return Lerp((value - 2300f) / 4600f, 0.55f, 0.3f);
		}
	}
}
namespace RedUtils.Math
{
	public struct Mat3x3
	{
		public Vec3[] Data { get; set; }

		public Vec3 Forward => Data[0];

		public Vec3 Right => Data[1];

		public Vec3 Up => Data[2];

		public Vec3 this[int index]
		{
			get
			{
				return Data[index];
			}
			set
			{
				Data[index] = value;
			}
		}

		public Mat3x3(Vec3 v1, Vec3 v2, Vec3 v3)
		{
			Data = new Vec3[3] { v1, v2, v3 };
		}

		public Mat3x3(Vec3 rotation)
		{
			float num = MathF.Cos(rotation[0]);
			float num2 = MathF.Sin(rotation[0]);
			float num3 = MathF.Cos(rotation[1]);
			float num4 = MathF.Sin(rotation[1]);
			float num5 = MathF.Cos(rotation[2]);
			float num6 = MathF.Sin(rotation[2]);
			Data = new Vec3[3]
			{
				new Vec3(num * num3, num * num4, num2),
				new Vec3(num3 * num2 * num6 - num5 * num4, num4 * num2 * num6 + num5 * num3, (0f - num) * num6),
				new Vec3((0f - num5) * num3 * num2 - num6 * num4, (0f - num5) * num4 * num2 + num6 * num3, num * num5)
			};
		}

		public Vec3 Dot(Vec3 v)
		{
			return new Vec3(Forward.Dot(v), Right.Dot(v), Up.Dot(v));
		}

		public Mat3x3 Transpose()
		{
			return new Mat3x3(new Vec3(Forward.x, Right.x, Up.x), new Vec3(Forward.y, Right.y, Up.y), new Vec3(Forward.z, Right.z, Up.z));
		}

		public static Mat3x3 RotationFromAxis(Vec3 axis, float angle)
		{
			float num = MathF.Cos(angle);
			float num2 = MathF.Sin(angle);
			float num3 = 1f - num;
			Vec3 vec = axis.Normalize();
			return new Mat3x3(new Vec3(num + vec.x * vec.x * num3, vec.x * vec.y * num3 - vec.z * num2, vec.x * vec.z * num3 + vec.y * num2), new Vec3(vec.y * vec.x * num3 + vec.z * num2, num + vec.y * vec.y * num3, vec.y * vec.z * num3 - vec.x * num2), new Vec3(vec.z * vec.x * num3 - vec.y * num2, vec.z * vec.y * num3 + vec.x * num2, num + vec.z * vec.z * num3));
		}
	}
	public struct Vec3
	{
		public float x;

		public float y;

		public float z;

		public float this[int index]
		{
			get
			{
				return index switch
				{
					0 => x, 
					1 => y, 
					2 => z, 
					_ => float.NaN, 
				};
			}
			set
			{
				switch (index)
				{
				case 0:
					x = value;
					break;
				case 1:
					y = value;
					break;
				case 2:
					z = value;
					break;
				}
			}
		}

		public static Vec3 Zero => new Vec3(0f, 0f, 0f);

		public static Vec3 Up => new Vec3(0f, 0f, 1f);

		public static Vec3 Z => new Vec3(0f, 0f, 1f);

		public static Vec3 Y => new Vec3(0f, 1f, 0f);

		public static Vec3 X => new Vec3(1f, 0f, 0f);

		public Vec3(float x, float y, float z)
		{
			this.x = x;
			this.y = y;
			this.z = z;
		}

		public Vec3(float x, float y)
		{
			this.x = x;
			this.y = y;
			z = 0f;
		}

		public Vec3(float[] values)
		{
			if (values.Length != 0)
			{
				x = values[0];
			}
			else
			{
				x = 0f;
			}
			if (values.Length > 1)
			{
				y = values[1];
			}
			else
			{
				y = 0f;
			}
			if (values.Length > 2)
			{
				z = values[2];
			}
			else
			{
				z = 0f;
			}
		}

		public Vec3(Vec3 vector)
		{
			x = vector.x;
			y = vector.y;
			z = vector.z;
		}

		public Vec3(rlbot.flat.Vector3 vector)
		{
			x = vector.X;
			y = vector.Y;
			z = vector.Z;
		}

		public Vec3(Rotator rotator)
		{
			x = rotator.Pitch;
			y = rotator.Yaw;
			z = rotator.Roll;
		}

		public static Vec3 operator +(Vec3 v1, Vec3 v2)
		{
			return new Vec3(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
		}

		public static Vec3 operator -(Vec3 v)
		{
			return new Vec3(0f - v.x, 0f - v.y, 0f - v.z);
		}

		public static Vec3 operator -(Vec3 v1, Vec3 v2)
		{
			return new Vec3(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
		}

		public static Vec3 operator *(Vec3 v1, Vec3 v2)
		{
			return new Vec3(v1.x * v2.x, v1.y * v2.y, v1.z * v2.z);
		}

		public static Vec3 operator *(Vec3 v, float a)
		{
			return new Vec3(v.x * a, v.y * a, v.z * a);
		}

		public static Vec3 operator *(float a, Vec3 v)
		{
			return new Vec3(a * v.x, a * v.y, a * v.z);
		}

		public static Vec3 operator /(Vec3 v1, Vec3 v2)
		{
			return new Vec3(v1.x / v2.x, v1.y / v2.y, v1.z / v2.z);
		}

		public static Vec3 operator /(Vec3 v, float x)
		{
			return new Vec3(v.x / x, v.y / x, v.z / x);
		}

		public static bool operator ==(Vec3 v1, Vec3 v2)
		{
			if (v1.x == v2.x && v1.y == v2.y)
			{
				return v1.z == v2.z;
			}
			return false;
		}

		public static bool operator !=(Vec3 v1, Vec3 v2)
		{
			if (v1.x == v2.x && v1.y == v2.y)
			{
				return v1.z != v2.z;
			}
			return true;
		}

		public static Vec3 Abs(Vec3 vec)
		{
			return new Vec3(MathF.Abs(vec.x), MathF.Abs(vec.y), MathF.Abs(vec.z));
		}

		public float Dot(Vec3 v)
		{
			return x * v.x + y * v.y + z * v.z;
		}

		public Vec3 Dot(Mat3x3 m)
		{
			return m.Forward * x + m.Right * y + m.Up * z;
		}

		public Vec3 Cross()
		{
			return new Vec3(y, 0f - x, 0f);
		}

		public Vec3 Cross(Vec3 v)
		{
			return new Vec3(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
		}

		public float Length()
		{
			return MathF.Sqrt(x * x + y * y + z * z);
		}

		public Vec3 Normalize()
		{
			float num = Length();
			if (!(num > 0f))
			{
				return this;
			}
			return this / num;
		}

		public Vec3 FlatNorm()
		{
			return Flatten().Normalize();
		}

		public Vec3 FlatNorm(Vec3 up)
		{
			return Flatten(up).Normalize();
		}

		public Vec3 Flatten()
		{
			return new Vec3(x, y, 0f);
		}

		public Vec3 Flatten(Vec3 up)
		{
			up = up.Normalize();
			return this - up * Dot(up);
		}

		public Vec3 Direction(Vec3 vec)
		{
			return (vec - this).Normalize();
		}

		public Vec3 FlatDirection(Vec3 vec)
		{
			return (vec - this).FlatNorm();
		}

		public Vec3 FlatDirection(Vec3 vec, Vec3 up)
		{
			return (vec - this).FlatNorm(up);
		}

		public float Dist(Vec3 v)
		{
			return (v - this).Length();
		}

		public float FlatDist(Vec3 v)
		{
			return (v - this).Flatten().Length();
		}

		public float FlatDist(Vec3 v, Vec3 up)
		{
			return (v - this).Flatten(up).Length();
		}

		public float FlatLen()
		{
			return Flatten().Length();
		}

		public float FlatLen(Vec3 up)
		{
			return Flatten(up).Length();
		}

		public float Angle(Vec3 v)
		{
			return MathF.Acos(MathF.Round(Normalize().Dot(v.Normalize()), 4));
		}

		public float FlatAngle(Vec3 v)
		{
			return MathF.Acos(MathF.Round(Normalize().Flatten().Dot(v.Normalize().Flatten()), 4));
		}

		public float FlatAngle(Vec3 v, Vec3 up)
		{
			return MathF.Acos(MathF.Round(Normalize().Flatten(up).Dot(v.Normalize().Flatten(up)), 4));
		}

		public Vec3 Rescale(float n)
		{
			return Normalize() * n;
		}

		public Vec3 Cap(float min, float max)
		{
			return Rescale(Utils.Cap(Length(), min, max));
		}

		public Vec3 Clamp(Vec3 start, Vec3 end)
		{
			Vec3 v = Normalize();
			bool flag = v.Dot(end.Cross(-Up)) < 0f;
			bool flag2 = v.Dot(start.Cross(-Up)) > 0f;
			if ((end.Dot(start.Cross(-Up)) > 0f) ? (flag2 & flag) : (flag2 | flag))
			{
				return this;
			}
			if (start.Dot(v) < end.Dot(v))
			{
				return end.Flatten().Normalize() * Flatten().Length() + Up * z;
			}
			return start.Flatten().Normalize() * Flatten().Length() + Up * z;
		}

		public Vec3 Clamp(Vec3 start, Vec3 end, Vec3 up)
		{
			Vec3 v = Normalize();
			bool flag = v.Dot(end.Cross(-up)) < 0f;
			bool flag2 = v.Dot(start.Cross(-up)) > 0f;
			if ((end.Dot(start.Cross(-up)) > 0f) ? (flag2 & flag) : (flag2 | flag))
			{
				return this;
			}
			if (start.Dot(v) < end.Dot(v))
			{
				return end.FlatNorm(up) * Flatten(up).Length() + up * Dot(up);
			}
			return start.FlatNorm(up) * Flatten(up).Length() + up * Dot(up);
		}

		public Vec3 Rotate(float angle)
		{
			return Flatten(Up) * MathF.Cos(angle) + Flatten(Up).Cross(Up).Rescale(Flatten(Up).Length()) * MathF.Sin(angle) + Up * Dot(Up);
		}

		public Vec3 Rotate(float angle, Vec3 rotationAxis)
		{
			return Flatten(rotationAxis) * MathF.Cos(angle) + Flatten(rotationAxis).Cross(rotationAxis).Rescale(Flatten(rotationAxis).Length()) * MathF.Sin(angle) + rotationAxis * Dot(rotationAxis);
		}

		public override string ToString()
		{
			return $"({x}, {y}, {z})";
		}
	}
}
