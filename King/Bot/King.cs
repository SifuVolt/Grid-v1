using System;
using System.Drawing;
using RedUtils;
using RedUtils.Math;

namespace Bot;


public class PossesionProperties
{
	public bool hasPossesion;

	public BallSlice intercept;

	public float freeTime;

	public float eta;

	public float power;
}
public class ParkAt : IAction
{
	public Arrive arriveAction;

	private bool directionSet;

	public bool Finished { get; internal set; }

	public bool Interruptible { get; internal set; }

	public ParkAt(Car car, Vec3 location)
	{
		Finished = false;
		Interruptible = true;
		arriveAction = new Arrive(car, location, location.FlatDirection(Ball.Location));
	}

	public ParkAt(Car car, Vec3 location, Vec3 direction)
	{
		Finished = false;
		Interruptible = true;
		directionSet = true;
		arriveAction = new Arrive(car, location, direction);
	}

	public void Run(RUBot bot)
	{
		Interruptible = arriveAction.Interruptible;
		if (!directionSet)
		{
			arriveAction.Direction = arriveAction.Target.FlatDirection(Ball.Location);
		}
		arriveAction.Run(bot);
		if (arriveAction.Target.Dist(bot.Me.Location) < 400f)
		{
			bot.Throttle(0f);
			bot.AimAt(bot.Me.Location + arriveAction.Direction);
		}
	}
}
public class Position : IAction
{
	public GetBoost getBoost;

	public bool Finished { get; internal set; }

	public bool Interruptible { get; internal set; }

	public Vec3 TargetLocation => getBoost.ChosenBoost.Location;

	public Position(Car car, int index)
	{
		Finished = false;
		Interruptible = true;
		getBoost = new GetBoost(car, index, interruptible: true);
	}

	public void Run(RUBot bot)
	{
		Interruptible = getBoost.Interruptible;
		Finished = getBoost.Finished;
		int num = (int)MathF.Min(bot.Me.Location.Dist(Ball.Location) / 1410f * 60f, 359f);
		Vec3 vec = ((Ball.Prediction.Length > num) ? Ball.Prediction[num].Location : Ball.Location);
		bool flag = bot.Me.Location.Direction(bot.OurGoal.Location).Dot(bot.Me.Location.Direction(vec)) > 0.2f;
		getBoost.DriveAction.WasteBoost = flag;
		getBoost.DriveAction.AllowDodges = flag;
		getBoost.Run(bot);
		if (getBoost.ChosenBoost.Location.Dist(bot.Me.Location) < 600f && getBoost.ChosenBoost.IsLarge)
		{
			Interruptible = false;
		}
	}
}
public class Shadow : IAction
{
	private Drive _driveAction;

	public bool Finished { get; internal set; }

	public bool Interruptible { get; internal set; }

	public Vec3 TargetLocation => _driveAction.Target;

	public Shadow(Car car)
	{
		Finished = false;
		Interruptible = true;
		Vec3 location = Field.Goals[car.Team].Location;
		_driveAction = new Drive(car, CalculateTargetPos(car, location), 2300f, allowDodges: true, wasteBoost: false);
		_driveAction.Backwards = false;
	}

	public void Run(RUBot bot)
	{
		_driveAction.Target = CalculateTargetPos(bot.Me, bot.OurGoal.Location);
		Interruptible = _driveAction.Interruptible;
		_driveAction.WasteBoost = !(bot as King).IsBack();
		_driveAction.Run(bot);
	}

	private Vec3 CalculateTargetPos(Car car, Vec3 ourGoal)
	{
		int num = (int)MathF.Min(car.Location.Dist(Ball.Location) / 1410f * 60f, 359f);
		Vec3 vec = ((Ball.Prediction.Length > num) ? Ball.Prediction[num].Location : Ball.Location);
		if (_driveAction != null)
		{
			Vec3 vec2 = car.LocationAfterDodge();
			_driveAction.AllowDodges = car.Location.Direction(ourGoal).Dot(car.Location.Direction(vec)) > 0f || vec2.Dist(ourGoal) > 1500f;
			_driveAction.WasteBoost = car.Location.Direction(ourGoal).Dot(car.Location.Direction(vec)) > 0f;
		}
		return Field.LimitToNearestSurface((vec + car.Location.Direction(ourGoal) * car.Location.Dist(vec) * 0.8f).Flatten());
	}
}


public class King : RUBot
{
	private string extraStratInfo = "";

	public override void Run()
	{
		Renderer.Text2D((Action != null) ? Action.ToString() : "", new Vec3(10f, 10f), 3, Color.White);
		Renderer.Text2D(extraStratInfo, new Vec3(10f, 40f), 2, Color.White);
		if (IsKickoff && Action == null)
		{
			bool flag = true;
			float num = Me.Location.Dist(Ball.Location);
			foreach (Car teammate in Teammates)
			{
				float num2 = teammate.Location.Dist(Ball.Location);
				bool flag2 = MathF.Sign(Me.Location.x) != Field.Side(Team);
				flag = flag && (num < num2 || ((num == num2) & flag2));
			}
			IAction action2;
			if (!flag)
			{
				IAction action = new GetBoost(Me, interruptible: false);
				action2 = action;
			}
			else
			{
				IAction action = new Kickoff();
				action2 = action;
			}
			Action = action2;
			return;
		}
		PossesionProperties possesionProperties = MyBall(out var theirPossesion);
		if (Action != null && Action is Shot)
		{
			if (Action is JumpShot)
			{
				JumpShot jumpShot = (JumpShot)Action;
				jumpShot.ArriveAction.AllowFlipping = ImportantBall(possesionProperties, theirPossesion);
				Action = jumpShot;
			}
			if (Action is DoubleJumpShot)
			{
				DoubleJumpShot doubleJumpShot = (DoubleJumpShot)Action;
				doubleJumpShot.ArriveAction.AllowFlipping = ImportantBall(possesionProperties, theirPossesion);
				Action = doubleJumpShot;
			}
			if (Action is GroundShot)
			{
				GroundShot groundShot = (GroundShot)Action;
				groundShot.ArriveAction.AllowFlipping = ImportantBall(possesionProperties, theirPossesion);
				Action = groundShot;
			}
		}
		if (Action != null && ((!(Action is ParkAt) && !(Action is Shadow) && !(Action is Position)) || !Action.Interruptible))
		{
			return;
		}
		Vec3 location = ((possesionProperties.intercept != null) ? possesionProperties.intercept.Location : Ball.Location);
		Vec3 location2 = ((theirPossesion.intercept != null) ? theirPossesion.intercept.Location : Ball.Location);
		new Vec3(-MathF.Sign(location.x) * 3500, -Field.Side(Team) * 4500);
		bool flag3 = ImportantBall(possesionProperties, theirPossesion);
		bool flag4 = MathF.Abs(possesionProperties.eta - theirPossesion.eta) < 0.25f && possesionProperties.intercept != null && possesionProperties.intercept.Location.z < 300f;
		if (!flag3 && !flag4 && CanDefend(Me, theirPossesion, OurGoal.Location) && Me.Boost < 20 && !possesionProperties.hasPossesion && (!theirPossesion.hasPossesion || ShotAngle(location2, new Target(OurGoal)) < 0.4f))
		{
			float num3 = 0f;
			Boost boost = null;
			foreach (Boost boost2 in Field.Boosts)
			{
				if (boost2.IsLarge && boost2.IsActive)
				{
					float num4 = location.Direction(boost2.Location).Dot(location.Direction(OurGoal.Location)) / location.Dist(boost2.Location);
					if (num4 > num3)
					{
						boost = boost2;
						num3 = num4;
					}
				}
			}
			if (boost != null)
			{
				extraStratInfo = "Getting Boost";
				SetAction(new Position(Me, boost.Index));
				return;
			}
		}
		if (theirPossesion.hasPossesion && (Ball.Location + Ball.Velocity).Dist(OurGoal.Location) > 1400f && (!AnyTeammatesBack() || IsBack()) && !flag4 && ShotAngle(location2, new Target(OurGoal)) > 0.4f)
		{
			Vec3 location3 = new Vec3(800 * -MathF.Sign(Ball.Location.x), 4900 * Field.Side(Team));
			extraStratInfo = "Rotating Back Post";
			SetAction(new ParkAt(Me, location3), lookForBoost: false);
		}
		else if (!flag3 && !flag4 && (Ball.Location + Ball.Velocity).Dist(OurGoal.Location) > 1400f && !possesionProperties.hasPossesion)
		{
			extraStratInfo = "Shadowing";
			SetAction(new Shadow(Me));
		}
		else if (flag4 || location.Dist(OurGoal.Location) < 2000f)
		{
			extraStratInfo = "Saving";
			SetAction(RUBot.FindShot(SaveShotCheck, new Target(TheirGoal)));
		}
		else if (AnyOpponentsBack() && (location.Dist(TheirGoal.Location) > 2500f || ShotAngle(location, new Target(TheirGoal)) < 0.2f))
		{
			extraStratInfo = "Clearing";
			SetAction(RUBot.FindShot(ClearShotCheck, new Target(new Vec3(1200 * -Field.Side(Team), 5200 * -Field.Side(Team), 1800f), new Vec3(1200 * Field.Side(Team), 5200 * -Field.Side(Team), 800f))));
		}
		else
		{
			extraStratInfo = "Shooting";
			SetAction(RUBot.FindShot(DefaultShotCheck, new Target(TheirGoal)));
		}
	}

	private void SetAction(IAction newAction, bool lookForBoost = true)
	{
		Vec3 target;
		if (newAction is Shot)
		{
			target = (newAction as Shot).TargetLocation;
		}
		else
		{
			target = ((newAction is Shadow) ? (newAction as Shadow).TargetLocation : OurGoal.Location);
		}
		bool flag = IsBack();
		float maxDetour = 0.05f + ((newAction is GetBoost) ? 0f : ((flag ? 0.05f : 0f) + ((!(newAction is Shot) & flag) ? 0.4f : 0f) + ((newAction is ParkAt) ? (-0.05f) : 0f)));
		int num = FindBoost(target, maxDetour);
		if (num < 0 || !lookForBoost)
		{
			Action = ((newAction is ParkAt && Action is ParkAt) ? Action : (newAction ?? Action ?? new ParkAt(Me, (Ball.Location + 2f * OurGoal.Location) / 3f)));
			return;
		}
		Position position = ((Action is Position) ? (Action as Position) : new Position(Me, num));
		IAction action2;
		if (!position.getBoost.DriveAction.Backwards || newAction == null)
		{
			IAction action = position;
			action2 = action;
		}
		else
		{
			action2 = newAction;
		}
		Action = action2;
	}

	public int FindBoost(Vec3 target, float maxDetour = 0.1f)
	{
		int num = -1;
		foreach (Boost boost in Field.Boosts)
		{
			if (boost.IsActive && PathViable(target, boost, maxDetour) && (num == -1 || (boost.IsLarge && !Field.Boosts[num].IsLarge)))
			{
				num = boost.Index;
			}
		}
		return num;
	}

	public bool PathViable(Vec3 target, Boost boost, float maxDetour = 0.1f)
	{
		float eta = KingCompat.GetEtaAuto(Me, target, true);
		float eta2 = KingCompat.GetEta(Me, boost.Location, KingCompat.ShouldDriveBackwards(Me, boost.Location, true), true, out var boostUsed);
		Car car = new Car(Me)
		{
			Location = boost.Location,
			Boost = (int)Utils.Cap((float)Me.Boost - boostUsed + (float)(boost.IsLarge ? 100 : 12), 0f, 100f),
			Forward = Me.Location.FlatDirection(boost.Location),
			Right = -Me.Location.FlatDirection(boost.Location).Cross().Normalize(),
			Up = Vec3.Up
		};
		if (eta2 + KingCompat.GetEtaAuto(car, target, true) - maxDetour < eta)
		{
			return target.Dist(boost.Location) > 800f;
		}
		return false;
	}

	private PossesionProperties MyBall(out PossesionProperties theirPossesion)
	{
		float num = 6f;
		float power = 0f;
		BallSlice intercept = null;
		float num2 = 6f;
		foreach (Car car in Cars.AllLivingCars)
		{
			BallSlice ballSlice = Ball.Prediction.Find((BallSlice slice) => Drive.GetEta(car, slice.Location) < slice.Time - Game.Time);
			Vec3 vec = ballSlice?.Location ?? Ball.Location;
			float num3;
			if (ballSlice != null)
			{
				num3 = ballSlice.Time - Game.Time;
			}
			else
			{
				num3 = ((6f + car.Location.Dist(vec) < 500f) ? 0f : car.Forward.Angle(car.Location.Direction(vec)));
			}
			if (num3 < num2)
			{
				num2 = num3;
			}
			if (num3 < num && (int)car.Team != Team && car.Location.Direction(TheirGoal.Location).Dot(vec.Direction(car.Location)) > 0f)
			{
				num = num3;
				intercept = ballSlice;
				power = CalculatePower(car, ballSlice, num3);
			}
		}
		BallSlice ballSlice2 = Ball.Prediction.Find((BallSlice slice) => Drive.GetEta(Me, slice.Location) < slice.Time - Game.Time);
		Vec3 vec2 = ballSlice2?.Location ?? Ball.Location;
		float num4 = ((ballSlice2 != null) ? (ballSlice2.Time - Game.Time) : 6f);
		bool flag = Me.Location.Direction(OurGoal.Location).Dot(vec2.Direction(Me.Location)) > 0f;
		theirPossesion = new PossesionProperties
		{
			freeTime = num2 - num,
			hasPossesion = (num2 > num - 0.2f),
			intercept = intercept,
			eta = num,
			power = power
		};
		return new PossesionProperties
		{
			freeTime = num2 - num4,
			hasPossesion = ((num2 > num4 - 0.2f) & flag),
			intercept = ballSlice2,
			eta = num4,
			power = CalculatePower(Me, ballSlice2, num4)
		};
	}

	private float CalculatePower(Car car, BallSlice intercept, float eta)
	{
		Ball ball = ((intercept != null) ? intercept.ToBall() : Ball.MainBall);
		return MathF.Max((ball.velocity - (ball.location - car.Location) / eta).Length(), ((ball.location - car.Location) / eta).Length());
	}

	private bool ImportantBall(PossesionProperties possesion, PossesionProperties otherPossesion)
	{
		if (possesion.intercept == null)
		{
			return true;
		}
		Vec3 location = ((possesion.intercept != null) ? possesion.intercept.Location : Ball.Location);
		Vec3 location2 = ((otherPossesion.intercept != null) ? otherPossesion.intercept.Location : Ball.Location);
		if (Ball.Prediction.FindGoal((int)(1u - Me.Team)) == null && (!(ShotAngle(location, new Target(TheirGoal)) > 0.5f) || AnyOpponentsBack() || possesion.hasPossesion) && !(ShotAngle(location2, new Target(OurGoal)) > 0.5f))
		{
			if (location.z > 500f)
			{
				return possesion.freeTime > -0.5f;
			}
			return false;
		}
		return true;
	}

	private float ShotAngle(Vec3 location, Target target)
	{
		return location.FlatDirection(target.TopLeft).Angle(location.FlatDirection(target.BottomRight));
	}

	public Shot ClearShotCheck(BallSlice slice, Target target)
	{
		if (slice != null)
		{
			float timeRemaining = slice.Time - Game.Time;
			if (timeRemaining > 0f && target.Fits(slice.Location))
			{
				Vec3 shotTarget = KingCompat.ClampTarget(target, slice.ToBall(), Me.Location.Direction(slice.Location));
				AerialShot aerialShot = new AerialShot(Me, slice, shotTarget);
				if (aerialShot.IsValid(Me))
				{
					return aerialShot;
				}
				GroundShot groundShot = new GroundShot(Me, slice, shotTarget);
				Vec3 requiredVelocity = (groundShot.TargetLocation - Me.Location) / timeRemaining;
				if (groundShot.IsValid(Me) && (groundShot.TargetLocation.z > 300f || Me.Location.FlatDirection(groundShot.TargetLocation).Dot(groundShot.ShotDirection.FlatNorm()) < 0.3f || (slice.Velocity - requiredVelocity).Length() > 1400f))
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
			Vec3 direction = Me.Location.FlatDirection(slice.Location).Clamp(Vec3.X * -Field.Side(Team), Vec3.X * Field.Side(Team));
			Vec3 shotTarget = slice.Location + direction.Normalize() * 2000f;
			AerialShot aerialShot = new AerialShot(Me, slice, shotTarget);
			if (aerialShot.IsValid(Me))
			{
				return aerialShot;
			}
			JumpShot jumpShot = new JumpShot(Me, slice, shotTarget);
			if (jumpShot.IsValid(Me) && (KingCompat.BestJumpingSurface(Me.Location, jumpShot.TargetLocation).Normal.z > 0.95f || jumpShot.TargetLocation.Dist(OurGoal.Location) > 1200f))
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

	public bool CanDefend(Car car, Vec3 location)
	{
		if (!CanBlock(car, location))
		{
			return Drive.GetEta(car, location) < Ball.Location.Dist(location) / MathF.Max(Ball.Velocity.Dot(Ball.Location.Direction(location)), 1500f);
		}
		return true;
	}

	public bool CanDefend(Car car, PossesionProperties enemyPossesion, Vec3 location)
	{
		Vec3 vec = ((enemyPossesion.intercept != null) ? enemyPossesion.intercept.Location : Ball.Location);
		if (!CanBlock(car, location))
		{
			return Drive.GetEta(car, location) < enemyPossesion.eta + vec.Dist(location) / enemyPossesion.power;
		}
		return true;
	}

	public bool CanBlock(Car car, Vec3 location)
	{
		return car.Location.Direction(location).Dot(Ball.Location.Direction(car.Location)) > 0.7f;
	}

	public bool IsBack()
	{
		PossesionProperties theirPossesion;
		if (!CanDefend(Me, OurGoal.Location))
		{
			return MyBall(out theirPossesion).hasPossesion;
		}
		return true;
	}

	private bool AnyOpponentsBack()
	{
		foreach (Car livingOpponent in LivingOpponents)
		{
			if (CanDefend(livingOpponent, TheirGoal.Location))
			{
				return true;
			}
		}
		return false;
	}

	private bool AnyTeammatesBack()
	{
		foreach (Car livingTeammate in LivingTeammates)
		{
			if (CanDefend(livingTeammate, OurGoal.Location))
			{
				return true;
			}
		}
		return false;
	}
}
