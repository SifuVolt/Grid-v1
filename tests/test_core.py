import math
import unittest
from types import SimpleNamespace

from redutils.math3d import Mat3x3, Vec3
from redutils.objects import Ball, BallPrediction, BallSlice, Car, Field, Game, Target
from redutils.actions import AerialShot, Drive
from intercept_calculator import InterceptCalculator
from redutils.utils import shot_power_modifier, time_to_jump
from redutils.renderer import ExtendedRenderer


class PartyCannonPortTests(unittest.TestCase):
    def test_vector_operations(self):
        v = Vec3(3, 4, 0)
        self.assertAlmostEqual(v.length, 5.0)
        self.assertAlmostEqual(v.Normalize().x, 0.6)
        self.assertAlmostEqual(v.Normalize().y, 0.8)
        self.assertAlmostEqual(Vec3(1, 0, 0).Cross(Vec3(0, 1, 0)).z, 1.0)

    def test_vector_rotation_handedness(self):
        rotated = Vec3(1, 0, 0).Rotate(math.pi / 2)
        self.assertAlmostEqual(rotated.x, 0.0, places=6)
        self.assertAlmostEqual(rotated.y, -1.0, places=6)

    def test_renderer_matches_rlbot_v5_signatures(self):
        class FakeRenderer:
            red = object()
            def draw_string_2d(self, *args): self.string_args = args
            def draw_rect_3d(self, *args): self.rect_args = args
            def is_rendering(self): return True

        raw = FakeRenderer()
        renderer = ExtendedRenderer(raw, 1920, 1080)
        renderer.Text2D("hello", Vec3(960, 540, 0), 1, "red")
        renderer.Rect3D(Vec3(1, 2, 3), 192, 108, True, "red")
        self.assertEqual(raw.string_args[:4], ("hello", 0.5, 0.5, 1))
        self.assertIs(raw.string_args[4], raw.red)
        self.assertEqual(len(raw.rect_args), 4)
        self.assertTrue(renderer.is_rendering())

    def test_aerial_action_can_initialize(self):
        car = Car()
        car.location = Vec3(0, 0, 17)
        car.velocity = Vec3(800, 0, 0)
        car.orientation = Mat3x3(Vec3(0, 0, 0))
        car.is_grounded = True
        slice_ = BallSlice(location=Vec3(0, 0, 500), velocity=Vec3(0, 0, 0), time=1.0)
        action = AerialShot(car, slice_, Vec3(0, 5120, 320))
        self.assertIsNotNone(action.drive_action)

    def test_rotator_packet_conversion(self):
        rotation = Vec3.from_value(SimpleNamespace(pitch=0.1, yaw=0.2, roll=0.3))
        self.assertAlmostEqual(rotation.x, 0.1)
        self.assertAlmostEqual(rotation.y, 0.2)
        self.assertAlmostEqual(rotation.z, 0.3)

    def test_car_keeps_packet_array_index(self):
        player = SimpleNamespace(
            physics=SimpleNamespace(location=Vec3(0, 0, 17), velocity=Vec3(), angular_velocity=Vec3(), rotation=SimpleNamespace(pitch=0.0, yaw=0.0, roll=0.0)),
            name="test", player_id=987, team=0, boost=0.0, is_bot=True,
            air_state="on_ground", demolished_timeout=0.0, is_supersonic=False,
            score_info=SimpleNamespace(score=0, goals=0, own_goals=0, assists=0, saves=0, shots=0, demolitions=0),
            latest_touch=None, hitbox=SimpleNamespace(length=118, width=84, height=36), hitbox_offset=Vec3()
        )
        car = Car(3, player)
        self.assertEqual(car.index, 3)

    def test_vector_dot_matrix(self):
        matrix = Mat3x3(Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1))
        result = Vec3(2, 3, 4).Dot(matrix)
        self.assertEqual(result, Vec3(2, 3, 4))
        self.assertIs(matrix.Data, matrix.data)

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
        self.assertEqual(Field.NearestSurface(Vec3(0, 0, 1950)).key, "Ground")
        self.assertEqual(Field.NearestSurface(Vec3(0, 0, 1950), ()).key, "Ceiling")

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
