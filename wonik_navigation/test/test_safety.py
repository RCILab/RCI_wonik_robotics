#!/usr/bin/env python3
"""Unit tests of wonik_navigation.safety (the code obstacle_detector.py and cmd_vel_gate.py run)."""
import math
import unittest

import numpy as np

from wonik_navigation.safety import (FREE_TURN, INF, N_DIRECTIONS, SensorTracker, TurnClearance, bounding_box,
                                     corridor_distances, gate_command, inside_polygon, stopping_distance,
                                     travel_directions)

LIDAR, DEPTH = "/points", "/camera/depth/points"
CLEAR = [INF] * N_DIRECTIONS
COBOT_V2 = [[0.452, 0.352], [0.452, -0.352], [-0.452, -0.352], [-0.452, 0.352]]
# SD: body ahead of the turning centre (fork roller axle), a narrow fork behind it (not convex)
SD = [[1.068, 0.403], [0.612, 0.403], [0.612, 0.172], [-0.132, 0.172], [-0.132, -0.172], [0.612, -0.172],
      [0.612, -0.403], [1.068, -0.403]]


def ahead(d):
    """An obstacle d metres in front of the body."""
    return [d] + [INF] * (N_DIRECTIONS - 1)


def dist_of(pts, min_points=1, footprint=COBOT_V2):
    """Corridor distances (side margin 0.1) for points [[x, y], ...]; cobot_v2: body +-0.452 x +-0.352."""
    p = np.array(pts, dtype=float).reshape(-1, 2)
    return corridor_distances(p[:, 0], p[:, 1], bounding_box(footprint), 0.1, 3.0, min_points)


def exact_turn(polygon, x, y, direction, step=math.radians(0.05)):
    """First in-place turn (direction +1 counterclockwise, -1 clockwise) that brings the polygon onto the
    point (x, y), by brute force (inf if none within a full turn)."""
    a = np.arange(0.0, 2 * math.pi, step)
    # the polygon turned by direction * a covers the point if the point turned back lies inside it
    c, s = np.cos(-direction * a), np.sin(-direction * a)
    hit = inside_polygon(c * x - s * y, s * x + c * y, polygon)
    return float(a[np.argmax(hit)]) if hit.any() else INF


class CorridorTest(unittest.TestCase):
    def test_directions(self):
        d = dist_of([[1.452, 0.0], [0.0, 0.652], [-0.752, 0.2], [0.0, -1.352]])
        self.assertAlmostEqual(d[0], 0.9)  # front: 1.452 - (0.452 + 0.1)
        self.assertAlmostEqual(d[2], 0.2)  # left
        self.assertAlmostEqual(d[4], 0.2)  # rear (0.2 m off centre, the body sweeps it)
        self.assertAlmostEqual(d[6], 0.9)  # right
        # moving diagonally front-left the body also sweeps into the point beside its left side
        self.assertAlmostEqual(d[1], 0.2 * math.sqrt(2))

    def test_beside_is_not_ahead(self):
        # a wall along the left side does not block driving forward or backward
        d = dist_of([[x, 0.55] for x in np.linspace(-2, 2, 50)])
        self.assertTrue(math.isinf(d[0]) and math.isinf(d[4]))
        self.assertAlmostEqual(d[2], 0.55 - 0.452)

    def test_noise_points_do_not_count(self):
        d = dist_of([[1.0, 0.0]] * 4 + [[2.0, 0.0]] * 10, min_points=5)
        self.assertAlmostEqual(d[0], 2.0 - 0.552)

    def test_diagonal_only_obstacle(self):
        # a point off the front-left corner: hit moving diagonally, not moving straight ahead or sideways
        d = dist_of([[1.2, 1.1]])
        self.assertTrue(math.isfinite(d[1]))
        self.assertTrue(math.isinf(d[0]) and math.isinf(d[2]))

    def test_body_off_the_turning_centre(self):
        # SD: the corridors start at the body's own edges, not at a box centred on base_footprint
        d = dist_of([[2.0, 0.0], [-1.0, 0.0], [0.3, 0.8]], footprint=SD)
        self.assertAlmostEqual(d[0], 2.0 - 1.168)   # ahead of the body front (1.068 + 0.1)
        self.assertAlmostEqual(d[4], 1.0 - 0.232)   # behind the fork tip (0.132 + 0.1)
        self.assertAlmostEqual(d[2], 0.8 - 0.503)   # beside the fork, within the body's width band


class TurnClearanceTest(unittest.TestCase):
    def check(self, polygon, points):
        tc = TurnClearance(polygon)
        for x, y in points:
            got = tc.clearance([x], [y])
            for k, direction in enumerate((1, -1)):
                exact = exact_turn(polygon, x, y, direction)
                if math.isinf(exact):
                    self.assertTrue(math.isinf(got[k]), (x, y, direction, got))
                else:
                    # conservative (never larger), and close
                    self.assertLessEqual(got[k], exact + 1e-9, (x, y, direction))
                    self.assertGreater(got[k], exact - math.radians(4.0), (x, y, direction))

    def test_centred_rectangle(self):
        # a point 0.5 m ahead of the centre: the corners at radius 0.5 reach it after acos(0.452/0.5) = 25 deg
        tc = TurnClearance(COBOT_V2)
        ccw, cw = tc.clearance([0.5], [0.0])
        self.assertAlmostEqual(math.degrees(ccw), 25.3, delta=3.0)
        self.assertAlmostEqual(ccw, cw, delta=math.radians(1.5))
        self.check(COBOT_V2, [[0.5, 0.0], [0.0, 0.45], [-0.4, 0.4], [0.3, -0.45], [0.6, 0.0]])

    def test_far_points_and_the_robot_itself_never_count(self):
        tc = TurnClearance(COBOT_V2)
        self.assertEqual(tc.clearance([0.6, 0.0, 0.2], [0.0, 0.58, 0.1]), FREE_TURN)  # beyond 0.572 m, inside

    def test_body_ahead_of_the_turning_centre(self):
        # SD: a crate 1 m right of the turning centre is reached after about 66 deg turning clockwise (the
        # body swings right onto it), only after about 246 deg turning counterclockwise
        tc = TurnClearance(SD)
        ccw, cw = tc.clearance([0.0], [-1.0])
        self.assertLess(math.degrees(cw), 100.0)
        self.assertGreater(math.degrees(ccw), 200.0)
        self.check(SD, [[0.0, -1.0], [0.0, 1.0], [1.15, 0.0], [-0.5, 0.0], [-0.3, 0.3], [0.3, 0.3], [0.9, 0.6]])

    def test_beside_the_fork_is_free_space(self):
        # inside the convex hull of the T shape but outside the footprint: an obstacle, not the robot
        tc = TurnClearance(SD)
        self.assertTrue(math.isfinite(tc.clearance([0.4], [0.3])[0]))

    def test_min_points(self):
        tc = TurnClearance(COBOT_V2)
        one = tc.clearance([0.5], [0.0])
        self.assertEqual(tc.clearance([0.5] + [0.0] * 4, [0.0] + [0.58] * 4, min_points=2), FREE_TURN)
        self.assertEqual(tc.clearance([0.5] * 5, [0.0] * 5, min_points=5), one)


class SensorTrackerTest(unittest.TestCase):
    def test_missing_required_sensor_is_not_ok(self):
        t = SensorTracker([LIDAR, DEPTH])
        t.update(LIDAR, 10.0, 10.0, CLEAR, FREE_TURN)
        dist, turn, ok, problems = t.evaluate(10.1)
        self.assertFalse(ok)
        self.assertEqual(dist, [0.0] * N_DIRECTIONS)  # fail-safe: obstacles everywhere
        self.assertEqual(turn, [0.0, 0.0])
        self.assertIn("required %s: no data" % DEPTH, problems)

    def test_required_depth_loss_is_not_hidden_by_the_lidar(self):
        # G: an obstacle only the depth camera sees, then the camera stops; the LiDAR keeps reporting clear
        t = SensorTracker([LIDAR, DEPTH])
        t.update(LIDAR, 10.0, 10.0, CLEAR, FREE_TURN)
        t.update(DEPTH, 10.0, 10.0, ahead(0.6), FREE_TURN)
        self.assertAlmostEqual(t.evaluate(10.05)[0][0], 0.6)
        for k in range(1, 20):
            t.update(LIDAR, 10.0 + 0.1 * k, 10.0 + 0.1 * k, CLEAR, FREE_TURN)
        dist, _, ok, _ = t.evaluate(11.9)
        self.assertEqual(dist[0], 0.0)
        self.assertFalse(ok)

    def test_optional_sensor_keeps_its_last_obstacle(self):
        t = SensorTracker([LIDAR], [DEPTH])
        t.update(LIDAR, 10.0, 10.0, CLEAR, FREE_TURN)
        t.update(DEPTH, 10.0, 10.0, ahead(0.6), FREE_TURN)
        for k in range(1, 20):
            t.update(LIDAR, 10.0 + 0.1 * k, 10.0 + 0.1 * k, CLEAR, FREE_TURN)
        dist, _, ok, problems = t.evaluate(11.9)
        self.assertAlmostEqual(dist[0], 0.6)  # held: the stale camera cannot clear it
        self.assertTrue(ok)                   # the LiDAR (required) is fine
        self.assertTrue(any("keeping its last obstacles" in p for p in problems))
        t.update(DEPTH, 11.9, 11.9, CLEAR, FREE_TURN)  # the camera is back and sees nothing
        self.assertTrue(math.isinf(t.evaluate(11.95)[0][0]))

    def test_invalid_frame_is_not_a_clear_view(self):
        # an all-NaN depth frame after a frame with an obstacle
        t = SensorTracker([LIDAR, DEPTH])
        t.update(LIDAR, 10.0, 10.0, CLEAR, FREE_TURN)
        t.update(DEPTH, 10.0, 10.0, ahead(0.6), FREE_TURN)
        self.assertTrue(t.update(DEPTH, 10.05, 10.05, CLEAR, FREE_TURN, valid=False))
        dist, _, ok, problems = t.evaluate(10.06)
        self.assertEqual(dist[0], 0.0)
        self.assertFalse(ok)
        self.assertTrue(any("no valid measurement" in p for p in problems))
        t.update(DEPTH, 10.1, 10.1, CLEAR, FREE_TURN)  # a valid, clear frame again
        dist, turn, ok, _ = t.evaluate(10.1)
        self.assertEqual((dist, turn, ok), (CLEAR, FREE_TURN, True))

    def test_invalid_frame_of_optional_sensor_keeps_its_obstacle(self):
        t = SensorTracker([LIDAR], [DEPTH])
        t.update(LIDAR, 10.0, 10.0, CLEAR, FREE_TURN)
        t.update(DEPTH, 10.0, 10.0, ahead(0.6), FREE_TURN)
        t.update(DEPTH, 10.05, 10.05, CLEAR, FREE_TURN, valid=False)
        dist, _, ok, _ = t.evaluate(10.06)
        self.assertAlmostEqual(dist[0], 0.6)
        self.assertTrue(ok)

    def test_freshness_uses_the_measurement_time(self):
        t = SensorTracker([LIDAR], timeout=0.5)
        t.update(LIDAR, 9.0, 10.0, CLEAR, FREE_TURN)  # arrives at 10.0 but was measured at 9.0
        self.assertFalse(t.evaluate(10.0)[2])

    def test_future_old_and_invalid_stamps_are_rejected(self):
        t = SensorTracker([LIDAR], future_tolerance=0.05)
        self.assertFalse(t.update(LIDAR, 10.5, 10.0, CLEAR, FREE_TURN))       # from the future
        self.assertTrue(t.update(LIDAR, 10.0, 10.0, CLEAR, FREE_TURN))
        self.assertFalse(t.update(LIDAR, 9.9, 10.1, ahead(0.1), [0.0, 0.0]))  # older than the stored one
        self.assertFalse(t.update(LIDAR, float("nan"), 10.1, CLEAR, FREE_TURN))
        self.assertFalse(t.update(LIDAR, 0.0, 10.1, CLEAR, FREE_TURN))
        self.assertFalse(t.update("/unknown", 10.1, 10.1, CLEAR, FREE_TURN))
        self.assertEqual(t.rejected, 5)
        self.assertEqual(t.evaluate(10.1)[0], CLEAR)  # the rejected obstacle did not count

    def test_reset_drops_everything(self):
        t = SensorTracker([LIDAR])
        t.update(LIDAR, 100.0, 100.0, CLEAR, FREE_TURN)
        t.reset()  # clock jumped back to 1.0
        self.assertFalse(t.evaluate(1.0)[2])
        self.assertTrue(t.update(LIDAR, 1.0, 1.0, CLEAR, FREE_TURN))
        self.assertTrue(t.evaluate(1.0)[2])


class GateCommandTest(unittest.TestCase):
    FWD, TURN, BOTH = (0.5, 0.0, 0.0), (0.0, 0.0, 0.4), (0.5, 0.0, 0.2)

    def gate(self, cmd, fresh=True, ok=True, dist=CLEAR, turn=FREE_TURN, avoid=False, speed=0.0, yaw_rate=0.0):
        return gate_command(cmd, fresh, ok, dist, turn, avoid, speed, yaw_rate)

    def test_clear(self):
        self.assertEqual(self.gate(self.BOTH), (self.BOTH, None))

    def test_protective_field_grows_with_speed(self):
        # 1.0 m/s needs 0.3 + 1.0 + 0.3 = 1.6 m; 0.14 m/s (creeping to a waypoint) needs about 0.36 m
        self.assertAlmostEqual(stopping_distance(1.0, 0.5, 0.3, 0.3), 1.6)
        wall = ahead(1.45)  # a wall 1.9 m from the centre, as at the (12, 6) corner of the test hall
        self.assertEqual(self.gate((1.0, 0.0, 0.0), dist=wall)[0], (0.0, 0.0, 0.0))
        self.assertEqual(self.gate((0.14, 0.0, 0.0), dist=wall), ((0.14, 0.0, 0.0), None))
        # still fast (the last command sent was 1.0 m/s): the faster speed counts
        self.assertEqual(self.gate((0.14, 0.0, 0.0), dist=wall, speed=1.0)[0], (0.0, 0.0, 0.0))

    def test_only_the_directions_of_travel_count(self):
        behind = dist_of([[-0.7, 0.0]])  # 0.15 m behind the (grown) body
        self.assertEqual(self.gate(self.FWD, dist=behind), (self.FWD, None))
        self.assertEqual(self.gate((-0.3, 0.0, 0.0), dist=behind)[0], (0.0, 0.0, 0.0))
        beside = dist_of([[x, 0.55] for x in np.linspace(-1, 1, 21)])  # a wall 0.2 m from the left side
        self.assertEqual(self.gate((0.0, 0.3, 0.0), dist=beside)[0], (0.0, 0.0, 0.0))   # sideways into it
        self.assertEqual(self.gate((0.0, -0.3, 0.0), dist=beside)[1], None)              # away from it
        self.assertEqual(self.gate((0.3, 0.0, 0.0), dist=beside)[1], None)               # along it
        self.assertEqual(self.gate((0.3, 0.3, 0.0), dist=beside)[0], (0.0, 0.0, 0.0))   # diagonally into it
        self.assertEqual(self.gate((0.3, 0.1, 0.0), dist=beside)[0], (0.0, 0.0, 0.0))   # between front and front-left

    def test_travel_directions(self):
        self.assertEqual(travel_directions(1.0, 0.0), [0])
        self.assertEqual(travel_directions(0.0, 1.0), [2])
        self.assertEqual(travel_directions(1.0, 0.3), [0, 1])
        self.assertEqual(travel_directions(1.0, -0.3), [7, 0])
        self.assertEqual(travel_directions(-1.0, 0.0), [4])

    def test_obstacle_in_the_way_of_a_turn_stops_it(self):
        # D at the gate: something the footprint touches 10 deg into a counterclockwise turn
        blocked_ccw = [math.radians(10.0), INF]
        out, reason = self.gate(self.TURN, turn=blocked_ccw)
        self.assertEqual(out, (0.0, 0.0, 0.0))
        self.assertIn("counterclockwise", reason)
        self.assertEqual(self.gate((0.0, 0.0, -0.4), turn=blocked_ccw), ((0.0, 0.0, -0.4), None))  # the other way
        self.assertEqual(self.gate(self.FWD, turn=blocked_ccw), (self.FWD, None))  # driving on is fine
        self.assertEqual(self.gate(self.TURN, dist=ahead(0.1)), (self.TURN, None))  # turning away from a wall

    def test_turn_protective_angle_grows_with_the_yaw_rate(self):
        # 0.4 rad/s needs 0.12 + 0.16 + 0.15 = 0.43 rad; 0.1 rad/s needs 0.03 + 0.01 + 0.15 = 0.19 rad
        turn = [0.3, INF]
        self.assertEqual(self.gate((0.0, 0.0, 0.4), turn=turn)[0], (0.0, 0.0, 0.0))
        self.assertEqual(self.gate((0.0, 0.0, 0.1), turn=turn), ((0.0, 0.0, 0.1), None))
        # still turning fast (the last command sent was 0.6 rad/s): the faster rate counts
        self.assertEqual(self.gate((0.0, 0.0, 0.1), turn=turn, yaw_rate=0.6)[0], (0.0, 0.0, 0.0))

    def test_detour_passes_obstacles_but_not_sensor_faults(self):
        self.assertEqual(self.gate(self.FWD, dist=ahead(0.1), turn=[0.0, 0.0], avoid=True), (self.FWD, None))
        self.assertEqual(self.gate(self.FWD, ok=False, avoid=True)[0], (0.0, 0.0, 0.0))
        self.assertEqual(self.gate(self.FWD, fresh=False, avoid=True)[0], (0.0, 0.0, 0.0))
        self.assertEqual(self.gate(self.TURN, ok=False)[0], (0.0, 0.0, 0.0))

    def test_invalid_command(self):
        for bad in ((math.nan, 0.0, 0.0), (0.0, math.inf, 0.0), (0.0, 0.0, -math.inf)):
            self.assertEqual(self.gate(bad, avoid=True), ((0.0, 0.0, 0.0), "invalid command"))


if __name__ == "__main__":
    unittest.main()
