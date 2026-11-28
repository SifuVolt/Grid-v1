using RedUtils.Math;

namespace RedUtils
{
	/// <summary>A car's hitbox</summary>
	public class Hitbox
	{
		/// <summary>The location of the car that this hitbox belongs to</summary>
		public Vec3 Location;
		/// <summary>The dimensions of this hitbox</summary>
		public Vec3 Dimensions;
		/// <summary>The local offset between the car and this hitbox</summary>
		public Vec3 Offset;
		/// <summary>The orientation matrix for this hitbox (and the car this hitbox belongs to)</summary>
		public Mat3x3 Orientation;

		/// <summary>The center of this hitbox</summary>
		public Vec3 Center { get { return Location + Offset.Dot(Orientation); } }

		/// <summary>Initializes a new car hitbox</summary>
		public Hitbox(Vec3 location, Vec3 dimensions, Vec3 offset, Mat3x3 orientation)
		{
			Location = location;
			Dimensions = dimensions;
			Offset = offset;
			Orientation = orientation;
		}

		/// <summary>Finds the closest point on the oriented hitbox to a world-space position.</summary>
		public Vec3 GetNearestPoint(Vec3 pos)
		{
			Vec3 local = Orientation.Dot(pos - Center);
			Vec3 clamped = new Vec3(
				Utils.Cap(local.x, -Dimensions.x / 2f, Dimensions.x / 2f),
				Utils.Cap(local.y, -Dimensions.y / 2f, Dimensions.y / 2f),
				Utils.Cap(local.z, -Dimensions.z / 2f, Dimensions.z / 2f));
			return Center + clamped.Dot(Orientation);
		}

		/// <summary>Checks whether a ball-sized sphere at the position overlaps this hitbox.</summary>
		public bool IsTouching(Vec3 pos)
		{
			return GetNearestPoint(pos).Dist(pos) < Ball.Radius;
		}

	}
}
