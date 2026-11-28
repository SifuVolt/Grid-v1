using System;
using RedUtils;
using RedUtils.Math;

namespace Bot;

internal static class KingCompat
{
    public static Surface BestJumpingSurface(Vec3 carPosition, Vec3 targetPosition)
    {
        Surface carSurface = Field.NearestSurface(carPosition);
        return carSurface.Limit(targetPosition).Dist(targetPosition) < 300f
            ? carSurface
            : Field.NearestSurface(targetPosition);
    }

    // RedUtils v4 Target.Clamp(Ball, direction), retained for King's clear-shot selection.
    public static Vec3 ClampTarget(Target target, Ball ball, Vec3 direction)
    {
        Vec3 up = (target.BottomRight - target.TopLeft).FlatNorm();
        target.GetCorrectedLimits(ball.location, out Vec3 left, out Vec3 right, out Vec3 top, out Vec3 bottom);

        Vec3 clamped = direction.Normalize()
            .Clamp((left - ball.location).Normalize(), (right - ball.location).Normalize())
            .Clamp((top - ball.location).Normalize(), (bottom - ball.location).Normalize(), up);

        if (up.Cross(-Vec3.Up).Dot(ball.location - (target.TopLeft + target.BottomRight) / 2f) > 0f)
        {
            float distance = MathF.Abs(
                (ball.location - target.TopLeft).Dot(up.Cross(-Vec3.Up)) /
                clamped.Dot(up.Cross(-Vec3.Up)));
            return target.TargetSurface.Limit(ball.location + clamped * distance);
        }

        return ball.location + clamped.Normalize() * 1000f;
    }

    public static bool ShouldDriveBackwards(Car car, Vec3 target, bool allowDodges = false)
    {
        float forwardsEta = GetEta(car, target, backwards: false, allowDodges, out _);
        float backwardsEta = GetEta(car, target, backwards: true, allowDodges, out _);

        if (backwardsEta + 0.5f < forwardsEta && car.Forward.Dot(car.Velocity) < 0f)
        {
            return car.Forward.FlatAngle(car.Location.Direction(target), car.Up) > 1.8849558f;
        }

        return false;
    }

    public static float GetEtaAuto(Car car, Vec3 target, bool allowDodges = true)
    {
        return GetEta(car, target, ShouldDriveBackwards(car, target, allowDodges), allowDodges, out _);
    }

    // Preserves King's original boost-use estimate, which RedUtilsV5 does not expose.
    public static float GetEta(Car car, Vec3 target, bool backwards, bool allowDodges, out float boostUsed)
    {
        float distance = Drive.GetDistance(car, target, backwards, out float angle, out float radius);
        float turnDistance = angle * radius;
        distance -= turnDistance;

        Vec3 surfaceNormal = car.IsGrounded
            ? Field.NearestSurface(car.Location).Normal
            : Field.FindLandingSurface(car).Normal;
        Vec3 carForward = car.IsGrounded
            ? car.Forward
            : (car.Velocity.FlatLen() > 500f
                ? car.Velocity.FlatNorm(surfaceNormal)
                : car.Location.FlatDirection(target, surfaceNormal));
        float currentSpeed = carForward.Dot(car.Velocity);
        float landingTime = car.PredictLandingTime();

        if (backwards)
        {
            float speed = MathF.Max(Drive.SpeedAfterTurn(-currentSpeed, angle, 0.8f), 1400f);
            boostUsed = 0f;
            return landingTime + turnDistance / MathF.Max(Drive.SpeedFromTurnRadius(radius), 400f) + distance / speed;
        }

        float minSpeed = MathF.Max(Drive.SpeedAfterTurn(currentSpeed, angle), 1400f);
        float maxSpeed = Utils.Cap(minSpeed + Car.BoostAccel * car.Boost / Car.BoostConsumption, 1400f, Car.MaxSpeed);
        float distanceWhileBoosting = (maxSpeed * maxSpeed - minSpeed * minSpeed) / (2f * Car.BoostAccel);

        if (distance < distanceWhileBoosting)
        {
            maxSpeed = Utils.Cap(MathF.Sqrt(MathF.Max(minSpeed * minSpeed + 2f * Car.BoostAccel * distance, 0f)), 1400f, Car.MaxSpeed);
            boostUsed = Car.BoostConsumption * (distance / ((minSpeed + maxSpeed) / 2f));
            return landingTime + turnDistance / MathF.Max(Drive.SpeedFromTurnRadius(radius), 400f) + distance / ((minSpeed + maxSpeed) / 2f);
        }

        boostUsed = car.Boost;
        if (allowDodges && distance / maxSpeed > 1.25f)
        {
            return landingTime + turnDistance / MathF.Max(Drive.SpeedFromTurnRadius(radius), 400f)
                + distanceWhileBoosting / ((minSpeed + maxSpeed) / 2f)
                + (distance - distanceWhileBoosting) / (maxSpeed + 500f);
        }

        return landingTime + turnDistance / MathF.Max(Drive.SpeedFromTurnRadius(radius), 400f)
            + distanceWhileBoosting / ((minSpeed + maxSpeed) / 2f)
            + (distance - distanceWhileBoosting) / maxSpeed;
    }
}
