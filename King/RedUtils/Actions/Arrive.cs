using System;
using RedUtils.Math;

namespace RedUtils
{
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
}
