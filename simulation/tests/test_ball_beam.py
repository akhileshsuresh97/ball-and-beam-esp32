"""Tests for the ball-and-beam simulation. Run from simulation/: python -m unittest discover tests"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ball_beam import (Hardware, ImprovedController, OriginalController, State,  # noqa: E402
                       simulate, step_ball)


class TestPhysics(unittest.TestCase):
    def test_level_beam_ball_stays(self):
        hw, s = Hardware(), State(x=0.24)
        for _ in range(1000):
            step_ball(s, hw, 0.001)
        self.assertAlmostEqual(s.x, 0.24, places=6)

    def test_rolling_hollow_sphere_acceleration(self):
        hw = Hardware(friction=0.0)
        s = State(x=0.10, steps=round(math.radians(5) / hw.rad_per_step))
        theta = s.beam_angle(hw)
        for _ in range(100):  # 0.1 s
            step_ball(s, hw, 0.001)
        expected = 0.5 * 0.6 * 9.81 * math.sin(theta) * 0.1 ** 2  # x = ½·a·t², a = 3/5·g·sinθ
        self.assertAlmostEqual(s.x - 0.10, expected, delta=expected * 0.02)

    def test_tilt_direction(self):
        hw = Hardware()
        s = State(x=0.24, steps=50)  # positive angle: ball rolls away from the sensor
        for _ in range(200):
            step_ball(s, hw, 0.001)
        self.assertGreater(s.x, 0.24)

    def test_end_stop(self):
        hw = Hardware()
        s = State(x=0.40, v=2.0)
        for _ in range(500):
            step_ball(s, hw, 0.001)
        self.assertAlmostEqual(s.x, hw.beam_length - hw.ball_radius)
        self.assertEqual(s.v, 0.0)

    def test_angle_limited_by_mechanics(self):
        hw = Hardware()
        self.assertAlmostEqual(State(x=0.2, steps=10_000).beam_angle(hw), hw.max_beam_angle)


class TestImprovedController(unittest.TestCase):
    def test_settles_from_both_ends(self):
        for x0 in (0.05, 0.43):
            r = simulate(ImprovedController(), x0=x0, duration=8)
            self.assertIsNotNone(r.settled(), f"did not settle from x0={x0}")
            self.assertLess(r.settled(), 3.0)
            self.assertLess(r.rms_error(after=4), 0.005)
            self.assertEqual(r.end_hits(), 0)

    def test_recovers_from_push(self):
        r = simulate(ImprovedController(), x0=0.24, duration=12, disturbance=(5, 0.3))
        self.assertIsNotNone(r.settled())
        self.assertEqual(r.end_hits(), 0)

    def test_robust_to_hardware_variation(self):
        variants = [Hardware(sensor_noise=0.006), Hardware(sensor_period=0.033),
                    Hardware(max_step_rate=2000, max_step_accel=20000),
                    Hardware(steps_per_rev=3200), Hardware(friction=0.3)]
        for hw in variants:
            r = simulate(ImprovedController(), x0=0.05, duration=10, hw=hw)
            self.assertEqual(r.end_hits(), 0, hw)
            self.assertLess(r.rms_error(after=6), 0.01, hw)

    def test_respects_angle_limit(self):
        r = simulate(ImprovedController(), x0=0.05, duration=3)
        self.assertLessEqual(abs(r.angle_deg).max(), 12.5)

    def test_reproducible(self):
        a = simulate(ImprovedController(), x0=0.1, duration=2, seed=3)
        b = simulate(ImprovedController(), x0=0.1, duration=2, seed=3)
        self.assertTrue((a.x == b.x).all())


class TestOriginalController(unittest.TestCase):
    def test_slide_gains_do_not_balance(self):
        r = simulate(OriginalController(setpoint_mm=240), x0=0.10, duration=10)
        self.assertIsNone(r.settled())
        self.assertGreater(r.end_hits(), 0)

    def test_milliseconds_inflate_integral(self):
        """Integral accumulates error × 50 per 50 ms sample instead of error × 0.05."""
        c, hw, s = OriginalController(kp=0, ki=1, kd=0, setpoint_mm=240), Hardware(), State(x=0.24)
        c.update(0.0, 140.0, s, hw, 0.001)  # error = 100 mm
        self.assertAlmostEqual(c.integral, 100 * 50)

    def test_steps_are_relative(self):
        """The same output twice moves the motor twice as far: the motor integrates the output."""
        c, hw, s = OriginalController(kp=1, ki=0, kd=0, setpoint_mm=240), Hardware(), State(x=0.24)
        t = 0.0
        for _ in range(2):
            c.next_sample = t
            c.update(t, 230.0, s, hw, 0.001)  # error 10 mm → output 10 → 10 steps
            while c.pending_steps:
                t += 0.001
                c.update(t, 230.0, s, hw, 0.001)
        self.assertEqual(s.steps, 20)


if __name__ == "__main__":
    unittest.main()
