#!/usr/bin/env python3
"""Unit tests of wonik_navigation.safety (the code obstacle_detector.py and cmd_vel_gate.py run)."""
import math
import unittest

from wonik_navigation.safety import SensorTracker, gate_command

LIDAR, DEPTH = "/points", "/camera/depth/points"


class SensorTrackerTest(unittest.TestCase):
    def test_missing_required_sensor_is_not_ok(self):
        t = SensorTracker([LIDAR, DEPTH])
        t.update(LIDAR, 10.0, 10.0, 0, 0)
        detected, near, ok, problems = t.evaluate(10.1)
        self.assertFalse(ok)
        self.assertTrue(detected and near)  # fail-safe
        self.assertIn("required %s: no data" % DEPTH, problems)

    def test_required_depth_loss_is_not_hidden_by_the_lidar(self):
        # G: an obstacle only the depth camera sees, then the camera stops; the LiDAR keeps reporting clear
        t = SensorTracker([LIDAR, DEPTH])
        t.update(LIDAR, 10.0, 10.0, 0, 0)
        t.update(DEPTH, 10.0, 10.0, 40, 0)
        self.assertTrue(t.evaluate(10.05)[0])
        for k in range(1, 20):
            t.update(LIDAR, 10.0 + 0.1 * k, 10.0 + 0.1 * k, 0, 0)
        detected, _, ok, _ = t.evaluate(11.9)
        self.assertTrue(detected)
        self.assertFalse(ok)

    def test_optional_sensor_keeps_its_last_obstacle(self):
        t = SensorTracker([LIDAR], [DEPTH])
        t.update(LIDAR, 10.0, 10.0, 0, 0)
        t.update(DEPTH, 10.0, 10.0, 40, 0)
        for k in range(1, 20):
            t.update(LIDAR, 10.0 + 0.1 * k, 10.0 + 0.1 * k, 0, 0)
        detected, _, ok, problems = t.evaluate(11.9)
        self.assertTrue(detected)  # held: the stale camera cannot clear it
        self.assertTrue(ok)        # the LiDAR (required) is fine
        self.assertTrue(any("keeping its last obstacle" in p for p in problems))
        t.update(DEPTH, 11.9, 11.9, 0, 0)  # the camera is back and sees nothing
        self.assertFalse(t.evaluate(11.95)[0])

    def test_optional_sensor_without_obstacle_is_only_reported(self):
        t = SensorTracker([LIDAR], [DEPTH])
        t.update(DEPTH, 10.0, 10.0, 0, 0)
        t.update(LIDAR, 12.0, 12.0, 0, 0)
        detected, _, ok, problems = t.evaluate(12.0)
        self.assertFalse(detected)
        self.assertTrue(ok)
        self.assertEqual(len(problems), 1)

    def test_invalid_frame_is_not_a_clear_view(self):
        # an all-NaN depth frame after a frame with an obstacle (P1 of the second review)
        t = SensorTracker([LIDAR, DEPTH])
        t.update(LIDAR, 10.0, 10.0, 0, 0)
        t.update(DEPTH, 10.0, 10.0, 40, 0)
        self.assertTrue(t.update(DEPTH, 10.05, 10.05, 0, 0, valid=False))
        detected, _, ok, problems = t.evaluate(10.06)
        self.assertTrue(detected)
        self.assertFalse(ok)
        self.assertTrue(any("no valid measurement" in p for p in problems))
        t.update(DEPTH, 10.1, 10.1, 0, 0)  # a valid, clear frame again
        self.assertEqual(t.evaluate(10.1)[:3], (False, False, True))

    def test_invalid_frame_of_optional_sensor_keeps_its_obstacle(self):
        t = SensorTracker([LIDAR], [DEPTH])
        t.update(LIDAR, 10.0, 10.0, 0, 0)
        t.update(DEPTH, 10.0, 10.0, 40, 0)
        t.update(DEPTH, 10.05, 10.05, 0, 0, valid=False)
        detected, _, ok, _ = t.evaluate(10.06)
        self.assertTrue(detected)
        self.assertTrue(ok)

    def test_freshness_uses_the_measurement_time(self):
        t = SensorTracker([LIDAR], timeout=0.5)
        t.update(LIDAR, 9.0, 10.0, 0, 0)  # arrives at 10.0 but was measured at 9.0
        self.assertFalse(t.evaluate(10.0)[2])

    def test_future_old_and_invalid_stamps_are_rejected(self):
        t = SensorTracker([LIDAR], future_tolerance=0.05)
        self.assertFalse(t.update(LIDAR, 10.5, 10.0, 0, 0))   # from the future
        self.assertTrue(t.update(LIDAR, 10.0, 10.0, 0, 0))
        self.assertFalse(t.update(LIDAR, 9.9, 10.1, 40, 40))  # older than the stored one
        self.assertFalse(t.update(LIDAR, float("nan"), 10.1, 0, 0))
        self.assertFalse(t.update(LIDAR, 0.0, 10.1, 0, 0))
        self.assertFalse(t.update("/unknown", 10.1, 10.1, 0, 0))
        self.assertEqual(t.rejected, 5)
        self.assertFalse(t.evaluate(10.1)[0])  # the rejected obstacle did not count

    def test_reset_drops_everything(self):
        t = SensorTracker([LIDAR])
        t.update(LIDAR, 100.0, 100.0, 0, 0)
        t.reset()  # clock jumped back to 1.0
        self.assertFalse(t.evaluate(1.0)[2])
        self.assertTrue(t.update(LIDAR, 1.0, 1.0, 0, 0))
        self.assertTrue(t.evaluate(1.0)[2])


class GateCommandTest(unittest.TestCase):
    FWD, TURN, BOTH = (0.5, 0.0, 0.0), (0.0, 0.0, 0.4), (0.5, 0.0, 0.2)

    def gate(self, cmd, fresh=True, ok=True, detected=False, near=False, avoid=False):
        return gate_command(cmd, fresh, ok, detected, near, avoid)

    def test_clear(self):
        self.assertEqual(self.gate(self.BOTH), (self.BOTH, None))

    def test_obstacle_ahead_stops_translation_not_turning(self):
        self.assertEqual(self.gate(self.FWD, detected=True)[0], (0.0, 0.0, 0.0))
        self.assertEqual(self.gate(self.BOTH, detected=True)[0], (0.0, 0.0, 0.0))
        self.assertEqual(self.gate(self.TURN, detected=True), (self.TURN, None))
        self.assertEqual(self.gate((0.005, 0.0, 0.4), detected=True)[0], (0.0, 0.0, 0.4))  # no creeping

    def test_obstacle_within_turn_radius_stops_turning(self):
        # D at the gate: something beside the robot, nothing ahead
        out, reason = self.gate(self.TURN, near=True)
        self.assertEqual(out, (0.0, 0.0, 0.0))
        self.assertIn("turn radius", reason)
        self.assertEqual(self.gate(self.FWD, near=True), (self.FWD, None))  # driving on along it is fine

    def test_detour_passes_obstacles_but_not_sensor_faults(self):
        self.assertEqual(self.gate(self.FWD, detected=True, near=True, avoid=True), (self.FWD, None))
        self.assertEqual(self.gate(self.FWD, ok=False, avoid=True)[0], (0.0, 0.0, 0.0))
        self.assertEqual(self.gate(self.FWD, fresh=False, avoid=True)[0], (0.0, 0.0, 0.0))
        self.assertEqual(self.gate(self.TURN, ok=False)[0], (0.0, 0.0, 0.0))

    def test_invalid_command(self):
        for bad in ((math.nan, 0.0, 0.0), (0.0, math.inf, 0.0), (0.0, 0.0, -math.inf)):
            self.assertEqual(self.gate(bad, avoid=True), ((0.0, 0.0, 0.0), "invalid command"))


if __name__ == "__main__":
    unittest.main()
