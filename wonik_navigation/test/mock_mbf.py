#!/usr/bin/env python3
"""Stand-in for move_base_flex in the supervisor/gate rostests.

exe_path: accepts goals (a newer goal preempts the running one), sends feedback at 10 Hz and publishes
  cmd_vel_raw = (0.5, 0, 0) while a goal runs, like a controller driving forward. Goals for a
  controller in ~reject_controllers are aborted with INVALID_PLUGIN before any feedback.
get_path: ~get_path_mode ok (a straight 2 m path), fail (aborted with NO_PATH_FOUND) or timeout (never
  answers). Every goal's controller is published on ~exe_controllers (std_msgs/String).
"""
import threading

import actionlib
import rospy
from geometry_msgs.msg import PoseStamped, Twist
from mbf_msgs.msg import (ExePathAction, ExePathFeedback, ExePathResult, GetPathAction, GetPathResult)
from std_msgs.msg import String


class MockMbf:
    def __init__(self):
        self.mode = rospy.get_param("~get_path_mode", "ok")
        self.reject = rospy.get_param("~reject_controllers", [])
        self.cmd_pub = rospy.Publisher("cmd_vel_raw", Twist, queue_size=1)
        self.ctrl_pub = rospy.Publisher("~exe_controllers", String, queue_size=10)
        self.exe = actionlib.SimpleActionServer("move_base_flex/exe_path", ExePathAction, self.on_exe, False)
        self.get = actionlib.SimpleActionServer("move_base_flex/get_path", GetPathAction, self.on_get, False)
        self.exe.start()
        self.get.start()

    def on_exe(self, goal):
        self.ctrl_pub.publish(String(data=goal.controller))
        if goal.controller in self.reject:
            self.exe.set_aborted(ExePathResult(outcome=ExePathResult.INVALID_PLUGIN, message="no such controller"))
            return
        rate = rospy.Rate(10)
        cmd = Twist()
        cmd.linear.x = 0.5
        while not rospy.is_shutdown():
            if self.exe.is_preempt_requested():
                self.exe.set_preempted(ExePathResult(outcome=ExePathResult.CANCELED, message="preempted"))
                return
            self.exe.publish_feedback(ExePathFeedback(outcome=ExePathResult.SUCCESS, message="driving"))
            self.cmd_pub.publish(cmd)
            rate.sleep()

    def on_get(self, goal):
        if self.mode == "timeout":
            while not rospy.is_shutdown() and not self.get.is_preempt_requested():
                rospy.sleep(0.05)
            self.get.set_preempted(GetPathResult(outcome=GetPathResult.CANCELED))
            return
        if self.mode == "fail":
            self.get.set_aborted(GetPathResult(outcome=GetPathResult.NO_PATH_FOUND, message="no path"))
            return
        result = GetPathResult(outcome=GetPathResult.SUCCESS, message="ok")
        result.path.header.frame_id = "map"
        for i in range(41):
            p = PoseStamped()
            p.header.frame_id = "map"
            p.pose.position.x = 0.05 * i
            p.pose.orientation.w = 1.0
            result.path.poses.append(p)
        self.get.set_succeeded(result)


if __name__ == "__main__":
    rospy.init_node("move_base_flex")
    MockMbf()
    rospy.spin()
