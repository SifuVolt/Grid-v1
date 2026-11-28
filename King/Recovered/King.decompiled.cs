using System;
using System.Diagnostics;
using System.Drawing;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.Versioning;
using Bot;
using RLBotDotNet;
using RedUtils;
using RedUtils.Math;

[assembly: CompilationRelaxations(8)]
[assembly: RuntimeCompatibility(WrapNonExceptionThrows = true)]
[assembly: Debuggable(DebuggableAttribute.DebuggingModes.IgnoreSymbolStoreSequencePoints)]
[assembly: TargetFramework(".NETCoreApp,Version=v6.0", FrameworkDisplayName = "")]
[assembly: AssemblyCompany("Bot")]
[assembly: AssemblyConfiguration("Release")]
[assembly: AssemblyFileVersion("1.0.0.0")]
[assembly: AssemblyInformationalVersion("1.0.0")]
[assembly: AssemblyProduct("Bot")]
[assembly: AssemblyTitle("Bot")]
[assembly: AssemblyVersion("1.0.0.0")]
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
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Expected Obj, but got Unknown
		Finished = false;
		Interruptible = true;
		arriveAction = new Arrive(car, location, location.FlatDirection(Ball.Location));
	}

	public ParkAt(Car car, Vec3 location, Vec3 direction)
	{
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Expected Obj, but got Unknown
		Finished = false;
		Interruptible = true;
		directionSet = true;
		arriveAction = new Arrive(car, location, direction);
	}

	public void Run(RUBot bot)
	{
		//IL_0056: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		//IL_007b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0086: Unknown result type (might be due to invalid IL or missing references)
		//IL_008b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		Interruptible = arriveAction.Interruptible;
		if (!directionSet)
		{
			arriveAction.Direction = arriveAction.Target.FlatDirection(Ball.Location);
		}
		arriveAction.Run(bot);
		if (arriveAction.Target.Dist(bot.Me.Location) < 400f)
		{
			bot.Throttle(0f, false);
			bot.AimAt(bot.Me.Location + arriveAction.Direction, default(Vec3), false, true);
		}
	}
}
public class Position : IAction
{
	public GetBoost getBoost;

	public bool Finished { get; internal set; }

	public bool Interruptible { get; internal set; }

	public Vec3 TargetLocation
	{
		get
		{
			//IL_000b: Unknown result type (might be due to invalid IL or missing references)
			return getBoost.ChosenBoost.Location;
		}
	}

	public Position(Car car, int index)
	{
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_0022: Expected Obj, but got Unknown
		Finished = false;
		Interruptible = true;
		getBoost = new GetBoost(car, index, true);
	}

	public void Run(RUBot bot)
	{
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_004f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0054: Unknown result type (might be due to invalid IL or missing references)
		//IL_0066: Unknown result type (might be due to invalid IL or missing references)
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0074: Unknown result type (might be due to invalid IL or missing references)
		//IL_005f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0079: Unknown result type (might be due to invalid IL or missing references)
		//IL_008b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0090: Unknown result type (might be due to invalid IL or missing references)
		//IL_0095: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f0: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ff: Unknown result type (might be due to invalid IL or missing references)
		Interruptible = getBoost.Interruptible;
		Finished = getBoost.Finished;
		int num = (int)MathF.Min(bot.Me.Location.Dist(Ball.Location) / 1410f * 60f, 359f);
		BallPrediction prediction = Ball.Prediction;
		Vec3 location;
		if (prediction.Length <= num)
		{
			location = Ball.Location;
		}
		else
		{
			prediction = Ball.Prediction;
			location = prediction[num].Location;
		}
		Vec3 val = location;
		Vec3 val2 = bot.Me.Location.Direction(bot.OurGoal.Location);
		bool flag = val2.Dot(bot.Me.Location.Direction(val)) > 0.2f;
		getBoost.DriveAction.WasteBoost = flag;
		getBoost.DriveAction.AllowDodges = flag;
		getBoost.Run(bot);
		val2 = getBoost.ChosenBoost.Location;
		if (val2.Dist(bot.Me.Location) < 600f && getBoost.ChosenBoost.IsLarge)
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

	public Vec3 TargetLocation
	{
		get
		{
			//IL_0006: Unknown result type (might be due to invalid IL or missing references)
			return _driveAction.Target;
		}
	}

	public Shadow(Car car)
	{
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
		//IL_0025: Unknown result type (might be due to invalid IL or missing references)
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_002b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0037: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Expected Obj, but got Unknown
		Finished = false;
		Interruptible = true;
		Vec3 location = Field.Goals[car.Team].Location;
		_driveAction = new Drive(car, CalculateTargetPos(car, location), 2300f, true, false);
		_driveAction.Backwards = false;
	}

	public void Run(RUBot bot)
	{
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		_driveAction.Target = CalculateTargetPos(bot.Me, bot.OurGoal.Location);
		Interruptible = _driveAction.Interruptible;
		_driveAction.WasteBoost = !(bot as King).IsBack();
		_driveAction.Run(bot);
	}

	private Vec3 CalculateTargetPos(Car car, Vec3 ourGoal)
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_0028: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0044: Unknown result type (might be due to invalid IL or missing references)
		//IL_004d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0038: Unknown result type (might be due to invalid IL or missing references)
		//IL_0052: Unknown result type (might be due to invalid IL or missing references)
		//IL_00da: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e1: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ed: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00fd: Unknown result type (might be due to invalid IL or missing references)
		//IL_0102: Unknown result type (might be due to invalid IL or missing references)
		//IL_0107: Unknown result type (might be due to invalid IL or missing references)
		//IL_010b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0110: Unknown result type (might be due to invalid IL or missing references)
		//IL_005c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0061: Unknown result type (might be due to invalid IL or missing references)
		//IL_006e: Unknown result type (might be due to invalid IL or missing references)
		//IL_006f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0074: Unknown result type (might be due to invalid IL or missing references)
		//IL_007e: Unknown result type (might be due to invalid IL or missing references)
		//IL_007f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0092: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c3: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c4: Unknown result type (might be due to invalid IL or missing references)
		int num = (int)MathF.Min(car.Location.Dist(Ball.Location) / 1410f * 60f, 359f);
		BallPrediction prediction = Ball.Prediction;
		Vec3 location;
		if (prediction.Length <= num)
		{
			location = Ball.Location;
		}
		else
		{
			prediction = Ball.Prediction;
			location = prediction[num].Location;
		}
		Vec3 val = location;
		Vec3 val3;
		if (_driveAction != null)
		{
			Vec3 val2 = car.LocationAfterDodge();
			Drive driveAction = _driveAction;
			val3 = car.Location.Direction(ourGoal);
			driveAction.AllowDodges = val3.Dot(car.Location.Direction(val)) > 0f || val2.Dist(ourGoal) > 1500f;
			Drive driveAction2 = _driveAction;
			val3 = car.Location.Direction(ourGoal);
			driveAction2.WasteBoost = val3.Dot(car.Location.Direction(val)) > 0f;
		}
		val3 = val + car.Location.Direction(ourGoal) * car.Location.Dist(val) * 0.8f;
		return Field.LimitToNearestSurface(val3.Flatten());
	}
}
namespace Bot;

public class King(string botName, int botTeam, int botIndex) : RUBot(botName, botTeam, botIndex)
{
	private string extraStratInfo = "";

	public override void Run()
	{
		//IL_002a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0055: Unknown result type (might be due to invalid IL or missing references)
		//IL_008d: Unknown result type (might be due to invalid IL or missing references)
		//IL_025d: Unknown result type (might be due to invalid IL or missing references)
		//IL_024f: Unknown result type (might be due to invalid IL or missing references)
		//IL_016f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0176: Expected Obj, but got Unknown
		//IL_0262: Unknown result type (might be due to invalid IL or missing references)
		//IL_01a7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01ae: Expected Obj, but got Unknown
		//IL_00b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_027b: Unknown result type (might be due to invalid IL or missing references)
		//IL_026d: Unknown result type (might be due to invalid IL or missing references)
		//IL_01df: Unknown result type (might be due to invalid IL or missing references)
		//IL_01e6: Expected Obj, but got Unknown
		//IL_0280: Unknown result type (might be due to invalid IL or missing references)
		//IL_0282: Unknown result type (might be due to invalid IL or missing references)
		//IL_02a9: Unknown result type (might be due to invalid IL or missing references)
		//IL_0128: Unknown result type (might be due to invalid IL or missing references)
		//IL_012f: Expected Obj, but got Unknown
		//IL_011d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0124: Expected Obj, but got Unknown
		//IL_0435: Unknown result type (might be due to invalid IL or missing references)
		//IL_043a: Unknown result type (might be due to invalid IL or missing references)
		//IL_043f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0444: Unknown result type (might be due to invalid IL or missing references)
		//IL_044e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0319: Unknown result type (might be due to invalid IL or missing references)
		//IL_0566: Unknown result type (might be due to invalid IL or missing references)
		//IL_0571: Unknown result type (might be due to invalid IL or missing references)
		//IL_057b: Expected Obj, but got Unknown
		//IL_057b: Expected Obj, but got Unknown
		//IL_0542: Unknown result type (might be due to invalid IL or missing references)
		//IL_04e5: Unknown result type (might be due to invalid IL or missing references)
		//IL_04ea: Unknown result type (might be due to invalid IL or missing references)
		//IL_04ef: Unknown result type (might be due to invalid IL or missing references)
		//IL_04f4: Unknown result type (might be due to invalid IL or missing references)
		//IL_04fe: Unknown result type (might be due to invalid IL or missing references)
		//IL_0474: Unknown result type (might be due to invalid IL or missing references)
		//IL_047c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0486: Expected Obj, but got Unknown
		//IL_065e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0669: Unknown result type (might be due to invalid IL or missing references)
		//IL_0673: Expected Obj, but got Unknown
		//IL_0673: Expected Obj, but got Unknown
		//IL_0595: Unknown result type (might be due to invalid IL or missing references)
		//IL_0494: Unknown result type (might be due to invalid IL or missing references)
		//IL_04b8: Unknown result type (might be due to invalid IL or missing references)
		//IL_04cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_0350: Unknown result type (might be due to invalid IL or missing references)
		//IL_0358: Unknown result type (might be due to invalid IL or missing references)
		//IL_0362: Expected Obj, but got Unknown
		//IL_05d6: Unknown result type (might be due to invalid IL or missing references)
		//IL_0606: Unknown result type (might be due to invalid IL or missing references)
		//IL_0635: Unknown result type (might be due to invalid IL or missing references)
		//IL_063a: Unknown result type (might be due to invalid IL or missing references)
		//IL_0644: Expected Obj, but got Unknown
		//IL_0644: Expected Obj, but got Unknown
		//IL_05a7: Unknown result type (might be due to invalid IL or missing references)
		//IL_05af: Unknown result type (might be due to invalid IL or missing references)
		//IL_05b9: Expected Obj, but got Unknown
		//IL_03a3: Unknown result type (might be due to invalid IL or missing references)
		//IL_03a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_03ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_03b9: Unknown result type (might be due to invalid IL or missing references)
		//IL_03be: Unknown result type (might be due to invalid IL or missing references)
		//IL_03cc: Unknown result type (might be due to invalid IL or missing references)
		((RUBot)this).Renderer.Text2D((base.Action != null) ? ((object)base.Action).ToString() : "", new Vec3(10f, 10f), 3, (Color?)Color.White);
		((RUBot)this).Renderer.Text2D(extraStratInfo, new Vec3(10f, 40f), 2, (Color?)Color.White);
		if (((RUBot)this).IsKickoff && base.Action == null)
		{
			bool flag = true;
			float num = ((RUBot)this).Me.Location.Dist(Ball.Location);
			foreach (Car teammate in ((RUBot)this).Teammates)
			{
				float num2 = teammate.Location.Dist(Ball.Location);
				bool flag2 = MathF.Sign(((RUBot)this).Me.Location.x) != Field.Side(((Bot)this).Team);
				flag = flag && (num < num2 || ((num == num2) & flag2));
			}
			IAction action;
			if (!flag)
			{
				IAction val = (IAction)new GetBoost(((RUBot)this).Me, false);
				action = val;
			}
			else
			{
				IAction val = (IAction)new Kickoff();
				action = val;
			}
			base.Action = action;
			return;
		}
		PossesionProperties possesionProperties = MyBall(out var theirPossesion);
		if (base.Action != null && base.Action is Shot)
		{
			if (base.Action is JumpShot)
			{
				JumpShot val2 = (JumpShot)base.Action;
				val2.ArriveAction.AllowFlipping = ImportantBall(possesionProperties, theirPossesion);
				base.Action = (IAction)(object)val2;
			}
			if (base.Action is DoubleJumpShot)
			{
				DoubleJumpShot val3 = (DoubleJumpShot)base.Action;
				val3.ArriveAction.AllowFlipping = ImportantBall(possesionProperties, theirPossesion);
				base.Action = (IAction)(object)val3;
			}
			if (base.Action is GroundShot)
			{
				GroundShot val4 = (GroundShot)base.Action;
				val4.ArriveAction.AllowFlipping = ImportantBall(possesionProperties, theirPossesion);
				base.Action = (IAction)(object)val4;
			}
		}
		if (base.Action != null && ((!(base.Action is ParkAt) && !(base.Action is Shadow) && !(base.Action is Position)) || !base.Action.Interruptible))
		{
			return;
		}
		Vec3 val5 = ((possesionProperties.intercept != null) ? possesionProperties.intercept.Location : Ball.Location);
		Vec3 location = ((theirPossesion.intercept != null) ? theirPossesion.intercept.Location : Ball.Location);
		new Vec3((float)(-MathF.Sign(val5.x) * 3500), (float)(-Field.Side(((Bot)this).Team) * 4500));
		bool flag3 = ImportantBall(possesionProperties, theirPossesion);
		bool flag4 = MathF.Abs(possesionProperties.eta - theirPossesion.eta) < 0.25f && possesionProperties.intercept != null && possesionProperties.intercept.Location.z < 300f;
		Vec3 val7;
		if (!flag3 && !flag4 && CanDefend(((RUBot)this).Me, theirPossesion, ((RUBot)this).OurGoal.Location) && ((RUBot)this).Me.Boost < 20 && !possesionProperties.hasPossesion && (!theirPossesion.hasPossesion || ShotAngle(location, new Target(((RUBot)this).OurGoal)) < 0.4f))
		{
			float num3 = 0f;
			Boost val6 = null;
			foreach (Boost boost in Field.Boosts)
			{
				if (boost.IsLarge && boost.IsActive)
				{
					val7 = val5.Direction(boost.Location);
					float num4 = val7.Dot(val5.Direction(((RUBot)this).OurGoal.Location)) / val5.Dist(boost.Location);
					if (num4 > num3)
					{
						val6 = boost;
						num3 = num4;
					}
				}
			}
			if (val6 != null)
			{
				extraStratInfo = "Getting Boost";
				SetAction((IAction)(object)new Position(((RUBot)this).Me, val6.Index));
				return;
			}
		}
		if (theirPossesion.hasPossesion)
		{
			val7 = Ball.Location + Ball.Velocity;
			if (val7.Dist(((RUBot)this).OurGoal.Location) > 1400f && (!AnyTeammatesBack() || IsBack()) && !flag4 && ShotAngle(location, new Target(((RUBot)this).OurGoal)) > 0.4f)
			{
				Vec3 location2 = new Vec3((float)(800 * -MathF.Sign(Ball.Location.x)), (float)(4900 * Field.Side(((Bot)this).Team)));
				extraStratInfo = "Rotating Back Post";
				SetAction((IAction)(object)new ParkAt(((RUBot)this).Me, location2), lookForBoost: false);
				return;
			}
		}
		if (!flag3 && !flag4)
		{
			val7 = Ball.Location + Ball.Velocity;
			if (val7.Dist(((RUBot)this).OurGoal.Location) > 1400f && !possesionProperties.hasPossesion)
			{
				extraStratInfo = "Shadowing";
				SetAction((IAction)(object)new Shadow(((RUBot)this).Me));
				return;
			}
		}
		if (flag4 || val5.Dist(((RUBot)this).OurGoal.Location) < 2000f)
		{
			extraStratInfo = "Saving";
			SetAction((IAction)(object)RUBot.FindShot((ShotCheck)base.SaveShotCheck, new Target(((RUBot)this).TheirGoal)));
		}
		else if (AnyOpponentsBack() && (val5.Dist(((RUBot)this).TheirGoal.Location) > 2500f || ShotAngle(val5, new Target(((RUBot)this).TheirGoal)) < 0.2f))
		{
			extraStratInfo = "Clearing";
			SetAction((IAction)(object)RUBot.FindShot((ShotCheck)base.ClearShotCheck, new Target(new Vec3((float)(1200 * -Field.Side(((Bot)this).Team)), (float)(5200 * -Field.Side(((Bot)this).Team)), 1800f), new Vec3((float)(1200 * Field.Side(((Bot)this).Team)), (float)(5200 * -Field.Side(((Bot)this).Team)), 800f))));
		}
		else
		{
			extraStratInfo = "Shooting";
			SetAction((IAction)(object)RUBot.FindShot((ShotCheck)base.DefaultShotCheck, new Target(((RUBot)this).TheirGoal)));
		}
	}

	private void SetAction(IAction newAction, bool lookForBoost = true)
	{
		//IL_0030: Unknown result type (might be due to invalid IL or missing references)
		//IL_0035: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0095: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cf: Unknown result type (might be due to invalid IL or missing references)
		//IL_00df: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e4: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00f3: Unknown result type (might be due to invalid IL or missing references)
		Vec3 target;
		if (newAction is Shot)
		{
			target = ((Shot)((newAction is Shot) ? newAction : null)).TargetLocation;
		}
		else
		{
			target = ((newAction is Shadow) ? (newAction as Shadow).TargetLocation : ((RUBot)this).OurGoal.Location);
		}
		bool flag = IsBack();
		float maxDetour = 0.05f + ((newAction is GetBoost) ? 0f : ((flag ? 0.05f : 0f) + ((!(newAction is Shot) & flag) ? 0.4f : 0f) + ((newAction is ParkAt) ? (-0.05f) : 0f)));
		int num = FindBoost(target, maxDetour);
		if (num < 0 || !lookForBoost)
		{
			base.Action = (IAction)((newAction is ParkAt && base.Action is ParkAt) ? base.Action : (newAction ?? ((object)base.Action) ?? ((object)new ParkAt(((RUBot)this).Me, (Ball.Location + 2f * ((RUBot)this).OurGoal.Location) / 3f))));
			return;
		}
		Position position = ((base.Action is Position) ? (base.Action as Position) : new Position(((RUBot)this).Me, num));
		IAction action;
		if (!position.getBoost.DriveAction.Backwards || newAction == null)
		{
			IAction val = (IAction)(object)position;
			action = val;
		}
		else
		{
			action = newAction;
		}
		base.Action = action;
	}

	public int FindBoost(Vec3 target, float maxDetour = 0.1f)
	{
		//IL_0020: Unknown result type (might be due to invalid IL or missing references)
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
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_0021: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0041: Unknown result type (might be due to invalid IL or missing references)
		//IL_0046: Unknown result type (might be due to invalid IL or missing references)
		//IL_004b: Unknown result type (might be due to invalid IL or missing references)
		//IL_007f: Unknown result type (might be due to invalid IL or missing references)
		//IL_008c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0091: Unknown result type (might be due to invalid IL or missing references)
		//IL_009b: Unknown result type (might be due to invalid IL or missing references)
		//IL_00a8: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00b5: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ba: Unknown result type (might be due to invalid IL or missing references)
		//IL_00bd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_00cd: Unknown result type (might be due to invalid IL or missing references)
		//IL_00d8: Expected Obj, but got Unknown
		//IL_00d9: Unknown result type (might be due to invalid IL or missing references)
		//IL_00e9: Unknown result type (might be due to invalid IL or missing references)
		float eta = Drive.GetEta(((RUBot)this).Me, target, true);
		float num = default;
		float eta2 = Drive.GetEta(((RUBot)this).Me, boost.Location, Drive.ShouldDriveBackwrds(((RUBot)this).Me, boost.Location, true), true, ref num);
		Car val = new Car(((RUBot)this).Me)
		{
			Location = boost.Location,
			Boost = (int)Utils.Cap((float)((RUBot)this).Me.Boost - num + (float)(boost.IsLarge ? 100 : 12), 0f, 100f),
			Forward = ((RUBot)this).Me.Location.FlatDirection(boost.Location)
		};
		Vec3 val2 = ((RUBot)this).Me.Location.FlatDirection(boost.Location);
		val2 = val2.Cross();
		val.Right = -val2.Normalize();
		val.Up = Vec3.Up;
		Car val3 = val;
		if (eta2 + Drive.GetEta(val3, target, true) - maxDetour < eta)
		{
			return target.Dist(boost.Location) > 800f;
		}
		return false;
	}

	private PossesionProperties MyBall(out PossesionProperties theirPossesion)
	{
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0066: Unknown result type (might be due to invalid IL or missing references)
		//IL_005d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0165: Unknown result type (might be due to invalid IL or missing references)
		//IL_016a: Unknown result type (might be due to invalid IL or missing references)
		//IL_006b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0190: Unknown result type (might be due to invalid IL or missing references)
		//IL_0187: Unknown result type (might be due to invalid IL or missing references)
		//IL_0082: Unknown result type (might be due to invalid IL or missing references)
		//IL_0195: Unknown result type (might be due to invalid IL or missing references)
		//IL_0098: Unknown result type (might be due to invalid IL or missing references)
		//IL_009d: Unknown result type (might be due to invalid IL or missing references)
		//IL_00ad: Unknown result type (might be due to invalid IL or missing references)
		//IL_00af: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c2: Unknown result type (might be due to invalid IL or missing references)
		//IL_01c7: Unknown result type (might be due to invalid IL or missing references)
		//IL_01cc: Unknown result type (might be due to invalid IL or missing references)
		//IL_01d8: Unknown result type (might be due to invalid IL or missing references)
		//IL_01dd: Unknown result type (might be due to invalid IL or missing references)
		//IL_0104: Unknown result type (might be due to invalid IL or missing references)
		//IL_0109: Unknown result type (might be due to invalid IL or missing references)
		//IL_010e: Unknown result type (might be due to invalid IL or missing references)
		//IL_011b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0120: Unknown result type (might be due to invalid IL or missing references)
		float num = 6f;
		float power = 0f;
		BallSlice intercept = null;
		float num2 = 6f;
		BallPrediction prediction;
		Vec3 val3;
		foreach (Car car in Cars.AllLivingCars)
		{
			prediction = Ball.Prediction;
			BallSlice val = prediction.Find((Predicate<BallSlice>)((BallSlice slice) =>
			{
				//IL_0007: Unknown result type (might be due to invalid IL or missing references)
				return Drive.GetEta(car, slice.Location) < slice.Time - Game.Time;
			}), 20);
			Vec3 val2 = val?.Location ?? Ball.Location;
			float num3;
			if (val == null)
			{
				if (!(6f + car.Location.Dist(val2) < 500f))
				{
					val3 = car.Forward;
					num3 = val3.Angle(car.Location.Direction(val2));
				}
				else
				{
					num3 = 0f;
				}
			}
			else
			{
				num3 = val.Time - Game.Time;
			}
			float num4 = num3;
			if (num4 < num2)
			{
				num2 = num4;
			}
			if (num4 < num && car.Team != ((Bot)this).Team)
			{
				val3 = car.Location.Direction(((RUBot)this).TheirGoal.Location);
				if (val3.Dot(val2.Direction(car.Location)) > 0f)
				{
					num = num4;
					intercept = val;
					power = CalculatePower(car, val, num4);
				}
			}
		}
		prediction = Ball.Prediction;
		BallSlice val4 = prediction.Find((Predicate<BallSlice>)((BallSlice slice) =>
		{
			//IL_0007: Unknown result type (might be due to invalid IL or missing references)
			return Drive.GetEta(((RUBot)this).Me, slice.Location) < slice.Time - Game.Time;
		}), 20);
		Vec3 val5 = val4?.Location ?? Ball.Location;
		float num5 = ((val4 != null) ? (val4.Time - Game.Time) : 6f);
		val3 = ((RUBot)this).Me.Location.Direction(((RUBot)this).OurGoal.Location);
		bool flag = val3.Dot(val5.Direction(((RUBot)this).Me.Location)) > 0f;
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
			freeTime = num2 - num5,
			hasPossesion = ((num2 > num5 - 0.2f) & flag),
			intercept = val4,
			eta = num5,
			power = CalculatePower(((RUBot)this).Me, val4, num5)
		};
	}

	private float CalculatePower(Car car, BallSlice intercept, float eta)
	{
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0023: Unknown result type (might be due to invalid IL or missing references)
		//IL_0029: Unknown result type (might be due to invalid IL or missing references)
		//IL_002e: Unknown result type (might be due to invalid IL or missing references)
		//IL_0033: Unknown result type (might be due to invalid IL or missing references)
		//IL_003c: Unknown result type (might be due to invalid IL or missing references)
		//IL_0042: Unknown result type (might be due to invalid IL or missing references)
		//IL_0047: Unknown result type (might be due to invalid IL or missing references)
		//IL_004d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0052: Unknown result type (might be due to invalid IL or missing references)
		Ball val = ((intercept != null) ? intercept.ToBall() : Ball.MainBall);
		Vec3 val2 = val.velocity - (val.location - car.Location) / eta;
		float x = val2.Length();
		val2 = (val.location - car.Location) / eta;
		return MathF.Max(x, val2.Length());
	}

	private bool ImportantBall(PossesionProperties possesion, PossesionProperties otherPossesion)
	{
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0012: Unknown result type (might be due to invalid IL or missing references)
		//IL_0024: Unknown result type (might be due to invalid IL or missing references)
		//IL_003a: Unknown result type (might be due to invalid IL or missing references)
		//IL_002d: Unknown result type (might be due to invalid IL or missing references)
		//IL_003f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0040: Unknown result type (might be due to invalid IL or missing references)
		//IL_0045: Unknown result type (might be due to invalid IL or missing references)
		//IL_005d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0064: Unknown result type (might be due to invalid IL or missing references)
		//IL_006e: Expected Obj, but got Unknown
		//IL_0086: Unknown result type (might be due to invalid IL or missing references)
		//IL_008d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0097: Expected Obj, but got Unknown
		//IL_009e: Unknown result type (might be due to invalid IL or missing references)
		if (possesion.intercept == null)
		{
			return true;
		}
		Vec3 val = ((possesion.intercept != null) ? possesion.intercept.Location : Ball.Location);
		Vec3 location = ((otherPossesion.intercept != null) ? otherPossesion.intercept.Location : Ball.Location);
		BallPrediction prediction = Ball.Prediction;
		if (prediction.FindGoal(1 - ((RUBot)this).Me.Team) == null && (!(ShotAngle(val, new Target(((RUBot)this).TheirGoal)) > 0.5f) || AnyOpponentsBack() || possesion.hasPossesion) && !(ShotAngle(location, new Target(((RUBot)this).OurGoal)) > 0.5f))
		{
			if (val.z > 500f)
			{
				return possesion.freeTime > -0.5f;
			}
			return false;
		}
		return true;
	}

	private float ShotAngle(Vec3 location, Target target)
	{
		//IL_0003: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0013: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		Vec3 val = location.FlatDirection(target.TopLeft);
		return val.Angle(location.FlatDirection(target.BottomRight));
	}

	public bool CanDefend(Car car, Vec3 location)
	{
		//IL_0002: Unknown result type (might be due to invalid IL or missing references)
		//IL_000b: Unknown result type (might be due to invalid IL or missing references)
		//IL_0011: Unknown result type (might be due to invalid IL or missing references)
		//IL_0016: Unknown result type (might be due to invalid IL or missing references)
		//IL_0019: Unknown result type (might be due to invalid IL or missing references)
		//IL_001f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0024: Unknown result type (might be due to invalid IL or missing references)
		//IL_0027: Unknown result type (might be due to invalid IL or missing references)
		//IL_002c: Unknown result type (might be due to invalid IL or missing references)
		//IL_002f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0030: Unknown result type (might be due to invalid IL or missing references)
		if (!CanBlock(car, location))
		{
			float eta = Drive.GetEta(car, location);
			Vec3 val = Ball.Location;
			float num = val.Dist(location);
			val = Ball.Velocity;
			Vec3 location2 = Ball.Location;
			return eta < num / MathF.Max(val.Dot(location2.Direction(location)), 1500f);
		}
		return true;
	}

	public bool CanDefend(Car car, PossesionProperties enemyPossesion, Vec3 location)
	{
		//IL_0015: Unknown result type (might be due to invalid IL or missing references)
		//IL_0008: Unknown result type (might be due to invalid IL or missing references)
		//IL_001a: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		//IL_0026: Unknown result type (might be due to invalid IL or missing references)
		//IL_0034: Unknown result type (might be due to invalid IL or missing references)
		Vec3 val = ((enemyPossesion.intercept != null) ? enemyPossesion.intercept.Location : Ball.Location);
		if (!CanBlock(car, location))
		{
			return Drive.GetEta(car, location) < enemyPossesion.eta + val.Dist(location) / enemyPossesion.power;
		}
		return true;
	}

	public bool CanBlock(Car car, Vec3 location)
	{
		//IL_0006: Unknown result type (might be due to invalid IL or missing references)
		//IL_0007: Unknown result type (might be due to invalid IL or missing references)
		//IL_000c: Unknown result type (might be due to invalid IL or missing references)
		//IL_000f: Unknown result type (might be due to invalid IL or missing references)
		//IL_0014: Unknown result type (might be due to invalid IL or missing references)
		//IL_0018: Unknown result type (might be due to invalid IL or missing references)
		//IL_001d: Unknown result type (might be due to invalid IL or missing references)
		Vec3 val = car.Location.Direction(location);
		Vec3 location2 = Ball.Location;
		return val.Dot(location2.Direction(car.Location)) > 0.7f;
	}

	public bool IsBack()
	{
		//IL_000d: Unknown result type (might be due to invalid IL or missing references)
		PossesionProperties theirPossesion;
		if (!CanDefend(((RUBot)this).Me, ((RUBot)this).OurGoal.Location))
		{
			return MyBall(out theirPossesion).hasPossesion;
		}
		return true;
	}

	private bool AnyOpponentsBack()
	{
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		foreach (Car livingOpponent in ((RUBot)this).LivingOpponents)
		{
			if (CanDefend(livingOpponent, ((RUBot)this).TheirGoal.Location))
			{
				return true;
			}
		}
		return false;
	}

	private bool AnyTeammatesBack()
	{
		//IL_001e: Unknown result type (might be due to invalid IL or missing references)
		foreach (Car livingTeammate in ((RUBot)this).LivingTeammates)
		{
			if (CanDefend(livingTeammate, ((RUBot)this).OurGoal.Location))
			{
				return true;
			}
		}
		return false;
	}
}
internal class Program
{
	private static void Main(string[] args)
	{
		int num;
		try
		{
			num = int.Parse(args[0]);
		}
		catch (Exception)
		{
			ConsoleColor foregroundColor = Console.ForegroundColor;
			Console.ForegroundColor = ConsoleColor.Red;
			Console.WriteLine("Could not get port from arguments to C# bot!\nIf you're reading this message, it means that the C# bot did not receive a valid port in the command line arguments.\nIf you have configured auto-run, the port should be given to the bot automatically. Otherwise, you'll need to run the bot with the port every time (e.g. Bot.exe 36969). Note that this port should match the one in python_run_file.py.\nIf you're trying to run the bot without auto-run in an IDE, see this source file (Bot/Program.cs) for IDE instructions.");
			Console.ForegroundColor = foregroundColor;
			Console.ReadKey();
			throw;
		}
		new BotManager<King>(0).Start(num);
	}
}
