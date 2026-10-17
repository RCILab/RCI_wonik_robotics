#!/usr/bin/env python3
"""rostest: cmd_vel_gate alone. A detour state (AVOID) opens the obstacle stop only while the supervisor
keeps publishing it, and never overrides missing sensor data or a silent detector. The command in force
is stopped as soon as the inputs require it, without a new cmd_vel_raw."""
import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, String


# an obstacle 0.5 m in front of the body: inside the protective field at 0.5 m/s (0.7 m)
AHEAD = [0.5] + [float("inf")] * 7


class GateTimeoutTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.last = None
        self.inputs = {"state": "AVOID", "detector": True, "sensors_ok": True, "near": False,
                       "raw": (0.5, 0.0, 0.0)}
        rospy.Subscriber("/cmd_vel", Twist, self.on_cmd)
        self.state = rospy.Publisher("/route_supervisor/state", String, queue_size=1)
        self.dist = rospy.Publisher("/obstacle_distances", Float32MultiArray, queue_size=1)
        self.near = rospy.Publisher("/obstacle_near", Bool, queue_size=1)
        self.ok = rospy.Publisher("/obstacle_detector/sensors_ok", Bool, queue_size=1)
        self.raw = rospy.Publisher("/cmd_vel_raw", Twist, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.tick)

    def on_cmd(self, msg):
        with self.lock:
            self.last = (msg.linear.x, msg.angular.z)

    def tick(self, _):
        if self.inputs["state"] is not None:
            self.state.publish(String(data=self.inputs["state"]))
        if self.inputs["detector"]:
            self.dist.publish(Float32MultiArray(data=AHEAD))  # an obstacle ahead the whole time
            self.near.publish(Bool(data=self.inputs["near"]))
            self.ok.publish(Bool(data=self.inputs["sensors_ok"]))
        if self.inputs["raw"] is not None:
            cmd = Twist()
            cmd.linear.x, _, cmd.angular.z = self.inputs["raw"]
            self.raw.publish(cmd)

    def out_after(self, seconds):
        rospy.sleep(seconds)
        with self.lock:
            return self.last

    def vx_after(self, seconds):
        return self.out_after(seconds)[0]

    def test_gate(self):
        rospy.sleep(1.0)
        self.assertEqual(self.vx_after(0.5), 0.5, "AVOID (fresh) must pass the detour command")

        self.inputs["state"] = None  # supervisor dies: its last AVOID must not keep the gate open
        self.assertEqual(self.vx_after(1.0), 0.0)

        self.inputs["state"] = "AVOID"
        self.assertEqual(self.vx_after(0.5), 0.5)
        self.inputs["sensors_ok"] = False  # a required sensor is stale: stop even on a detour
        self.assertEqual(self.vx_after(0.5), 0.0)

        self.inputs["sensors_ok"] = True
        self.assertEqual(self.vx_after(0.5), 0.5)
        self.inputs["detector"] = False  # the detector node dies
        self.assertEqual(self.vx_after(1.0), 0.0)

        self.inputs["detector"] = True
        self.inputs["state"] = "AVOID_PREP"  # avoidance wanted, detour not running yet
        self.assertEqual(self.vx_after(0.5), 0.0)

        # turning in place, then the raw commands stop and something comes within the turn radius:
        # the gate must stop the turn itself, well before the raw command times out (0.5 s)
        self.inputs["raw"] = (0.0, 0.0, 0.4)
        self.assertEqual(self.out_after(0.5), (0.0, 0.4))
        self.inputs["raw"] = None
        self.inputs["near"] = True
        self.assertEqual(self.out_after(0.2), (0.0, 0.0))

        # turning, then move_base_flex goes silent with nothing in the way: zero after raw_timeout
        self.inputs["near"] = False
        self.inputs["raw"] = (0.0, 0.0, 0.4)
        self.assertEqual(self.out_after(0.5), (0.0, 0.4))
        self.inputs["raw"] = None
        self.assertEqual(self.out_after(0.3), (0.0, 0.4))  # still within raw_timeout
        self.assertEqual(self.out_after(0.5), (0.0, 0.0))

        # a stop for one reason, then a sensor fault while the same command is in force: still stopped
        self.inputs["raw"] = (0.0, 0.0, 0.4)
        self.inputs["near"] = True
        self.assertEqual(self.out_after(0.3), (0.0, 0.0))
        self.inputs["sensors_ok"] = False
        self.assertEqual(self.out_after(0.3), (0.0, 0.0))


if __name__ == "__main__":
    rospy.init_node("test_gate_timeouts")
    rostest.rosrun("wonik_navigation", "test_gate_timeouts", GateTimeoutTest)
