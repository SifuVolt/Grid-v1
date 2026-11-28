using System;
using RLBotDotNet.GameState;
using RLBotDotNet.Renderer;
using RLBotDotNet.Utils;
using rlbot.flat;

namespace RLBotDotNet;

public abstract class Bot : IDisposable
{
	private const float MaxChatRate = 2f;

	private const int MaxChatCount = 5;

	[Obsolete("This field is obsolete. Use Name instead.", true)]
	public readonly string name;

	[Obsolete("This field is obsolete. Use Team instead.", true)]
	public readonly int team;

	[Obsolete("This field is obsolete. Use Index instead.", true)]
	public readonly int index;

	public readonly string Name;

	public readonly int Team;

	public readonly int Index;

	private DateTime lastChatTime;

	private bool resetChatTime;

	private int chatCounter;

	private int lastMessageId = -1;

	public RLBotDotNet.Renderer.Renderer Renderer { get; internal set; }

	public Bot(string botName, int botTeam, int botIndex)
	{
		Name = botName;
		Team = botTeam;
		Index = botIndex;
	}

	public abstract Controller GetOutput(GameTickPacket gameTickPacket);

	protected FieldInfo GetFieldInfo()
	{
		try
		{
			return RLBotInterface.GetFieldInfo();
		}
		catch (FlatbuffersPacketException)
		{
			throw new FlatbuffersPacketException("The game did not send any information. This could mean that the match has not started yet. This happens when you run the bot before (or as soon as) RLBot.exe gets started and the game has not started the match yet. This usually happens on the map loading screen.");
		}
	}

	protected BallPrediction GetBallPrediction()
	{
		return RLBotInterface.GetBallPredictionData();
	}

	[Obsolete("The RigidBodyTick has been deprecated. GetOutput gives all raw physics engine values, so there is no need to use GetRigidBodyTick.")]
	protected RigidBodyTick GetRigidBodyTick()
	{
		return RLBotInterface.GetRigidBodyTick();
	}

	protected MatchSettings GetMatchSettings()
	{
		return RLBotInterface.GetMatchSettingsData();
	}

	protected void SendQuickChatFromAgent(bool teamOnly, QuickChatSelection quickChat)
	{
		try
		{
			TimeSpan timeSpan = DateTime.Now - lastChatTime;
			if (!resetChatTime && timeSpan.TotalSeconds >= 2.0)
			{
				resetChatTime = true;
			}
			if (resetChatTime)
			{
				lastChatTime = DateTime.Now;
				chatCounter = 0;
				resetChatTime = false;
			}
			if (chatCounter < 5)
			{
				RLBotInterface.SendQuickChatFlat(Index, teamOnly, quickChat);
				chatCounter++;
			}
			else
			{
				Console.WriteLine($"Quick chat disabled for {(int)(2.0 - timeSpan.TotalSeconds)} seconds.");
			}
		}
		catch (FlatbuffersPacketException)
		{
			throw new FlatbuffersPacketException("The game did not send any information. This could mean that the match has not started yet. This happens when you run the bot before (or as soon as) RLBot.exe gets started and the game has not started the match yet. This usually happens on the map loading screen.");
		}
	}

	public QuickChatMessages ReceiveQuickChat()
	{
		QuickChatMessages result = RLBotInterface.ReceiveQuickChat(Index, Team, lastMessageId);
		if (result.MessagesLength > 0)
		{
			lastMessageId = result.Messages(result.MessagesLength - 1).Value.MessageIndex;
		}
		return result;
	}

	protected void SetGameState(RLBotDotNet.GameState.GameState gameState)
	{
		if (gameState == null)
		{
			throw new ArgumentNullException("gameState");
		}
		RLBotInterface.SetGameStatePacket(gameState.BuildGameStatePacket());
	}

	public virtual void Dispose()
	{
	}
}
