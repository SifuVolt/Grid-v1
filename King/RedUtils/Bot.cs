using System;
using System.Collections.Generic;
using RedUtils.Math;
using RLBot.Flat;
using RLBot.Manager;

namespace RedUtils
{
	/// <summary>The main class in RedUtils. 
	/// <para>It contains properties that are unique to your bot, such as your bot's car, your teammates, etc.</para>
	/// <para>It also receives the GameTickPacket from the RLBot framework, and processes all the data in it.</para>
	/// </summary>
	public abstract partial class RUBot : Bot
	{
		/// <summary>A tool to draw debug lines in-game</summary>
		public new ExtendedRenderer Renderer { get; internal set; }

		/// <summary>Your car</summary>
		public Car Me => Index < Cars.Count ? Cars.AllCars[Index] : new Car();

		/// <summary>A list of all of the cars on your team (not including yourself, and including ones that are respawning)</summary>
		public List<Car> Teammates 
		{ 
			get 
			{ 
				List<Car> teammates = Cars.AllCars.FindAll(car => car.Team == Team);
				teammates.Remove(Me);
				return teammates;
			} 
		}
		/// <summary>A list of all of the cars on your team (not including yourself, and NOT including ones that are respawning)</summary>
		public List<Car> LivingTeammates
		{
			get
			{
				List<Car> teammates = Cars.AllCars.FindAll(car => !car.IsDemolished && car.Team == Team);
				teammates.Remove(Me);
				return teammates;
			}
		}

		/// <summary>A list of all of the cars on the opposing team (including ones that are respawning)</summary>
		public List<Car> Opponents => Cars.AllCars.FindAll(car => car.Team != Team);
		/// <summary>A list of all of the cars on the opposing team (NOT including ones that are respawning)</summary>
		public List<Car> LivingOpponents => Cars.AllCars.FindAll(car => !car.IsDemolished && car.Team != Team);

		/// <summary>Our team's goal</summary>
		public Goal OurGoal => Field.Goals[Team];
		/// <summary>The opposing team's goal</summary>
		public Goal TheirGoal => Field.Goals[1 - Team];

		/// <summary>The current score for your team</summary>
		public uint OurScore => Game.Scores[Team];
		/// <summary>The current score for the opposing teams</summary>
		public uint TheirScore => Game.Scores[1 - Team];

		/// <summary>Whether or not it's time for kickoff</summary>
		public bool IsKickoff { get; private set; }
		/// <summary>The change in time since the last tick</summary>
		public float DeltaTime { get; private set; }

		/// <summary>The inputs that are returned to RLBot every tick</summary>
		public ControllerStateT Controller = new ControllerStateT();
		/// <summary>
		/// The action that will be executed every tick, along side the "Run" function. 
		/// <para>Resets when the ball is touched, unless the action isn't interruptible. Otherwise only resets when the action has finished</para>
		/// </summary>
		public IAction Action = null;

		/// <summary>Whether the "GetReady" function has run</summary>
		private bool _ready = false;
		/// <summary>The last time the ball was touched</summary>
		private float _lastTouchTime;
		/// <summary>The previous moment in time. <para>Used to calculate DeltaTime</summary>
		private float _lastTime = 0;

		//a
		public RUBot() : base()
        {
			Console.WriteLine($"RedUtils bot \"{GetType().Name}\" is up and running.");
		}

		/// <summary>Initializes some static classes using data from the packet</summary>
		/// <param name="packet">Contains all information about the current game state</param>
		private void GetReady(GamePacketT packet)
		{
			Renderer = new ExtendedRenderer(base.Renderer, 1920, 1080);
            if (!Game.Initialized)
            {
                Game.Initialize();

                Field.Initialize(FieldInfo);
                Cars.Initialize(packet);
            }
            _ready = true;
		}

		/// <summary>Processes data from the packet, and uses said data to update some static classes</summary>
		/// <param name="packet">Contains all information about the current game state</param>
		private void Process(GamePacketT packet)
		{
            if (Game.Time < packet.MatchInfo.SecondsElapsed)
            {
                // only update static stuff if it hasn't already been updated this tick

                if (Cars.Count != packet.Players.Count)
                {
                    // Reinitializes the cars if someone has left or joined the game
                    Cars.Initialize(packet);
                }
                else
                {
                    // Updates the cars' positions, velocities, etc
                    Cars.Update(packet);
                }
				if (packet.Balls.Count > 0)
				{
                    // Updates the ball's position, velocity, etc
                    Ball.Update(this, packet.Balls[0]);
                }
                // Updates the game's score, time, etc
                Game.Update(packet);
                // Updates the boost pads
                Field.Update(packet);
            }

            if (!IsKickoff && Game.MatchPhase == MatchPhase.Kickoff)
            {
                // Reset the action right as a kickoff starts
                Action = null;
            }

            IsKickoff = Game.MatchPhase == MatchPhase.Kickoff;
        }

		/// <summary>Updates DeltaTime... pretty self explanitory</summary>
		private void UpdateDeltaTime()
		{
			DeltaTime = Game.Time - _lastTime;
			_lastTime = Game.Time;
		}

		/// <summary>The function that gets called every tick to get inputs from your bot</summary>
		/// <param name="packet">Contains all information about the current game state</param>
		/// <returns>The bot's inputs for that tick</returns>
		public override ControllerStateT GetOutput(GamePacketT packet)
		{
			// Resets the Controller every tick
			Controller = new ControllerStateT(); 

			if (!_ready)
			{
				// Gets the packet ready for the first time during startup
				GetReady(packet); 
			}
			// Proccesses the packet so that data is up to date during this frame
			Process(packet);

			base.Renderer.Begin();
			
			// Runs our strategy code
			Run(); 

			// if there is an action to execute...
			if (Action != null)
			{
				Action.Run(this); // execute it!

				// If the ball hasn't been touched, set it to -1, so we don't get errors.
                float latestTouchTime = Ball.LatestTouch == null ? -1 : Ball.LatestTouch.Time; 
				if (Action.Finished || (_lastTouchTime != latestTouchTime && Action.Interruptible) || Me.IsDemolished) 
				{
					// If the action has completed, or the ball has been touched and the action is interruptible,
					// or if our bot is demolished reset the action
					_lastTouchTime = latestTouchTime;
					Action = null;
				}
			}

			UpdateDeltaTime();

			base.Renderer.End();

			// returns our inputs to RLBot
			return Controller; 
		}

		/// <summary>Runs every tick. Overwrite with your own strategy code!</summary>
		public abstract void Run();

		/// <summary>Gets the ball prediction struct from the framework</summary>
		internal BallPrediction GetBallPrediction() => new BallPrediction(base.BallPrediction);
	}
}
