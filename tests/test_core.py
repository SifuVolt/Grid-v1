import math
import unittest

from redutils.math3d import Mat3x3, Vec3
from redutils.objects import Ball, BallPrediction, BallSlice, Car, Field, Game, Target
from redutils.actions import Drive
from intercept_calculator import InterceptCalculator
from redutils.utils import shot_power_modifier, time_to_jump


class PartyCannonPortTests(unittest.TestCase):
    def test_vector_operations(self):
        v = Vec3(3, 4, 0)
        self.assertAlmostEqual(v.length, 5.0)
        self.assertAlmostEqual(v.Normalize().x, 0.6)
        self.assertAlmostEqual(v.Normalize().y, 0.8)
        self.assertAlmostEqual(Vec3(1, 0, 0).Cross(Vec3(0, 1, 0)).z, 1.0)

    def test_orientation_matrix(self):
        orientation = Mat3x3(Vec3(0, 0, 0))
        self.assertAlmostEqual(orientation.Forward.x, 1.0)
        self.assertAlmostEqual(orientation.Right.y, 1.0)
        self.assertAlmostEqual(orientation.Up.z, 1.0)

    def test_field_geometry(self):
        self.assertEqual(len(Field.surfaces), 16)
        self.assertEqual(len(Field.drivable_surfaces), 15)
        self.assertTrue(Field.InField(Vec3(0, 0, 100), 50))
        self.assertFalse(Field.InField(Vec3(5000, 0, 100), 50))
        self.assertEqual(Field.LimitToNearestSurface(Vec3(0, 0, 100)).z, 0)

    def test_drive_eta_is_finite(self):
        car = Car()
        car.location = Vec3(0, 0, 17)
        car.velocity = Vec3(800, 0, 0)
        car.orientation = Mat3x3(Vec3(0, 0, 0))
        eta = Drive.GetEta(car, Vec3(1000, 1000, 17))
        self.assertTrue(math.isfinite(eta))
        self.assertGreater(eta, 0)

    def test_physics_helpers(self):
        self.assertGreater(time_to_jump(Vec3.Up, 100), 0)
        self.assertGreater(shot_power_modifier(1000), 0)
        self.assertLess(shot_power_modifier(4000), shot_power_modifier(1000))

    def test_ball_prediction_search(self):
        prediction = BallPrediction([
            BallSlice(location=Vec3(0, 0, 100), velocity=Vec3(), time=0.0),
            BallSlice(location=Vec3(0, 0, 110), velocity=Vec3(), time=0.1),
            BallSlice(location=Vec3(0, 0, 120), velocity=Vec3(), time=0.2),
            BallSlice(location=Vec3(0, 0, 130), velocity=Vec3(), time=0.3),
            BallSlice(location=Vec3(0, 0, 140), velocity=Vec3(), time=0.4),
            BallSlice(location=Vec3(0, 0, 150), velocity=Vec3(), time=0.5),
            BallSlice(location=Vec3(0, 0, 160), velocity=Vec3(), time=0.6),
        ])
        self.assertEqual(len(prediction), 7)
        self.assertIsNotNone(prediction.Find(lambda s: s.location.z >= 150))

    def test_intercept_turn_radius(self):
        self.assertGreater(InterceptCalculator.GetTurningRadius(1400), 0)


if __name__ == "__main__":
    unittest.main()
