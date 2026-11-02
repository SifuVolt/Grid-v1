"""Focused regression tests for Grid's interception speed model."""

import unittest

from physics.interception import (
    MAX_SPEED,
    _boost_distance,
    _speed_after_boost,
    _interpolate_speed_gain,
    BOOST_TIMES,
    BOOST_SPEEDS,
)


class InterceptionSpeedTests(unittest.TestCase):
    def test_gain_curve_starts_at_zero(self):
        self.assertEqual(_interpolate_speed_gain(BOOST_TIMES, BOOST_SPEEDS, 0.0), 0.0)

    def test_gain_curve_interpolates(self):
        self.assertAlmostEqual(
            _interpolate_speed_gain(BOOST_TIMES, BOOST_SPEEDS, 0.25),
            250.0,
            places=5,
        )

    def test_boost_speed_is_capped(self):
        self.assertEqual(_speed_after_boost(2200.0, 2.4), MAX_SPEED)

    def test_boost_distance_is_monotonic(self):
        short = _boost_distance(900.0, 0.5)
        long = _boost_distance(900.0, 1.0)
        self.assertGreater(long, short)

    def test_boost_distance_never_uses_negative_speed(self):
        self.assertGreaterEqual(_boost_distance(0.0, 0.5), 0.0)


if __name__ == "__main__":
    unittest.main()
