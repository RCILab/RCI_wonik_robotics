#!/usr/bin/env python3
"""rostest: route_supervisor + cmd_vel_gate against mock_mbf.py, robot in an avoid zone with an obstacle
ahead (0.5 m in front of the body on obstacle_distances, sensors ok).

~expect no_avoid  the detour never runs (planning times out / fails / the detour goal is rejected):
                  the supervisor must not report AVOID and the gate must not let the route goal's
                  forward command through (case A)
~expect avoid     planning works: the supervisor reports AVOID once the detour goal runs, and the gate
                  passes the detour's command although the obstacle is detected
"""
import threading
import unittest

import rospy
import rostest
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, String


# an obstacle 0.5 m in front of the body: inside the protective field at 0.5 m/s (0.7 m)
AHEAD = [0.5] + [float("inf")] * 7


class SupervisorGateTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.states, self.cmds, self.controllers = [], [], []
        rospy.Subscriber("/route_supervisor/state", String, self.record, callback_args=self.states)
        rospy.Subscriber("/cmd_vel", Twist, lambda m: self.record((rospy.get_time(), m.linear.x), self.cmds))
        rospy.Subscriber("/move_base_flex/exe_controllers", String, self.record, callback_args=self.controllers)
        self.zone = rospy.Publisher("/current_zone", String, queue_size=1)
        self.dist = rospy.Publisher("/obstacle_distances", Float32MultiArray, queue_size=1)
        self.near = rospy.Publisher("/obstacle_near", Bool, queue_size=1)
        self.ok = rospy.Publisher("/obstacle_detector/sensors_ok", Bool, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self.publish_inputs)

    def record(self, msg, store):
        with self.lock:
            store.append(msg.data if hasattr(msg, "data") else msg)

    def publish_inputs(self, _):
        self.zone.publish(String(data="avoid"))
        self.dist.publish(Float32MultiArray(data=AHEAD))
        self.near.publish(Bool(data=False))
        self.ok.publish(Bool(data=True))

    def test_avoid_entry(self):
        expect = rospy.get_param("~expect")
        duration = rospy.get_param("~duration", 8.0)
        end = rospy.get_time() + duration
        while rospy.get_time() < end and not rospy.is_shutdown():
            rospy.sleep(0.1)
        with self.lock:
            states, cmds, controllers = set(self.states), list(self.cmds), list(self.controllers)
        rospy.loginfo("states %s, controllers %s, %d gate outputs", states, controllers, len(cmds))
        self.assertIn("AVOID_PREP", states)
        self.assertGreater(len(cmds), 10)
        if expect == "no_avoid":
            self.assertNotIn("AVOID", states)
            self.assertEqual(max(abs(vx) for _, vx in cmds), 0.0, "the gate let a forward command through")
            if rospy.get_param("~expect_route_resent", False):
                # the rejected detour goal is followed by the route goal again
                self.assertIn("avoid", controllers)
                self.assertIn("stop_go", controllers[controllers.index("avoid"):])
        else:
            self.assertIn("AVOID", states)
            self.assertIn("avoid", controllers)
            self.assertGreater(max(vx for _, vx in cmds), 0.4, "the detour's command did not pass")


if __name__ == "__main__":
    rospy.init_node("test_supervisor_gate")
    rostest.rosrun("wonik_navigation", "test_supervisor_gate", SupervisorGateTest)
