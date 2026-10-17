#!/usr/bin/env python3
"""cmd_vel gate between move_base_flex and the base: passes cmd_vel_raw to cmd_vel, but stops any
translation while an obstacle is detected, unless the robot is on a planned detour. Turning in place
still passes: the detection box looks ahead, and a robot that stops facing a wall at a waypoint must
be able to turn away from it.

"On a detour" is the route supervisor's AVOID state. Without a supervisor the zone decides instead:
the gate is open in zones whose type is not in ~stop_zone_types.

Parameters
  ~stop_zone_types  zone types in which obstacles stop the robot when no supervisor state is known
                    (default [stop, none]: everywhere except avoid zones)
  ~timeout          [s] without /obstacle_detected for this long, the gate closes (fail-safe)
Topics
  cmd_vel_raw (in), obstacle_detected (in), current_zone (in), route_supervisor/state (in), cmd_vel (out)
  ~blocked    (std_msgs/Bool, out, latched) gate state
"""
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class CmdVelGate:
    def __init__(self):
        self.stop_types = set(rospy.get_param("~stop_zone_types", ["stop", "none"]))
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.obstacle = True
        self.obstacle_stamp = None
        self.zone = "none"
        self.supervisor_state = None
        self.blocked = None
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.blocked_pub = rospy.Publisher("~blocked", Bool, queue_size=1, latch=True)
        rospy.Subscriber("obstacle_detected", Bool, self.on_obstacle, queue_size=1)
        rospy.Subscriber("current_zone", String, self.on_zone, queue_size=1)
        rospy.Subscriber("route_supervisor/state", String, self.on_state, queue_size=1)
        rospy.Subscriber("cmd_vel_raw", Twist, self.on_cmd, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), lambda _: self.update())

    def on_obstacle(self, msg):
        self.obstacle, self.obstacle_stamp = msg.data, rospy.get_time()
        self.update()

    def on_zone(self, msg):
        self.zone = msg.data
        self.update()

    def on_state(self, msg):
        self.supervisor_state = msg.data
        self.update()

    def stops_for_obstacles(self):
        if self.supervisor_state is not None:
            return self.supervisor_state != "AVOID"
        return self.zone in self.stop_types

    def is_blocked(self):
        if not self.stops_for_obstacles():
            return False
        stale = self.obstacle_stamp is None or rospy.get_time() - self.obstacle_stamp > self.timeout
        return self.obstacle or stale

    def update(self):
        blocked = self.is_blocked()
        if blocked != self.blocked:
            self.blocked = blocked
            self.blocked_pub.publish(Bool(data=blocked))
            rospy.loginfo("cmd_vel_gate: %s (zone %s, supervisor %s)", "STOP, obstacle ahead" if blocked else "pass",
                          self.zone, self.supervisor_state)
            if blocked:
                self.pub.publish(Twist())

    def on_cmd(self, msg):
        if not self.is_blocked():
            self.pub.publish(msg)
            return
        out = Twist()
        if abs(msg.linear.x) < 0.01 and abs(msg.linear.y) < 0.01:
            out.angular.z = msg.angular.z  # turning in place
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_gate")
    CmdVelGate()
    rospy.spin()
