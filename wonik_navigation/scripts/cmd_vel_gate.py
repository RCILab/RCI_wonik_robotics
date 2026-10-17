#!/usr/bin/env python3
"""cmd_vel gate between move_base_flex and the base: passes cmd_vel_raw to cmd_vel according to
wonik_navigation.safety.gate_command:

  - stops everything while the obstacle detector is silent or a required sensor is not current (in
    every state)
  - passes everything while the route supervisor reports a running detour (AVOID): the avoid
    controller checks its own motion against the costmap
  - otherwise keeps a protective field in the direction of travel: a translation stops when the nearest
    obstacle in the corridor(s) it moves into (obstacle_distances) is closer than the distance the base
    needs to stop from its speed (~latency at speed, braking at ~dec, plus ~margin); turning in place
    stops for an obstacle within the turn radius (obstacle_near)

The speed is the faster of the command and the last command sent (the base cannot be faster than it
was told). The supervisor state counts only while it is current: the supervisor publishes it at 10 Hz,
and a state older than ~state_timeout (supervisor dead or stuck) is treated as "no detour". Without a
supervisor the gate always applies the obstacle stops.

Every 50 ms the command in force (the last cmd_vel_raw, or none once it is older than ~raw_timeout) is
judged again with the current inputs; when the result is more restrictive than what was last sent
(an obstacle came within the turn radius during a turn, a sensor failed, move_base_flex went silent,
...), it is sent at once instead of waiting for the next raw command or the base's own timeout.

Parameters
  ~timeout        [s] detector messages older than this: detector silent (default 0.5)
  ~state_timeout  [s] supervisor state older than this is ignored (default 0.5)
  ~raw_timeout    [s] a cmd_vel_raw older than this is no longer in force: zero is sent (default 0.5)
  ~dec            [m/s^2] braking assumed for the protective field (default 0.5, below the base's 0.6)
  ~latency        [s] reaction time in the protective field (default 0.3)
  ~margin         [m] protective field beyond the stopping distance (default 0.3)
Topics
  cmd_vel_raw (in), obstacle_distances, obstacle_near, obstacle_detector/sensors_ok (in),
  route_supervisor/state (in), cmd_vel (out)
  ~blocked    (std_msgs/Bool, out, latched) a forward command at the current speed (at least 0.3 m/s) would
              be stopped now
  ~reason     (std_msgs/String, out, latched) why (empty when it would pass)
"""
import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, String

from wonik_navigation.safety import N_DIRECTIONS, gate_command


class CmdVelGate:
    def __init__(self):
        self.timeout = rospy.get_param("~timeout", 0.5)
        self.state_timeout = rospy.get_param("~state_timeout", 0.5)
        self.raw_timeout = rospy.get_param("~raw_timeout", 0.5)
        self.field = {k: rospy.get_param("~" + k, d) for k, d in (("dec", 0.5), ("latency", 0.3), ("margin", 0.3))}
        self.inputs = {}  # name -> (value, receive time)
        self.reason = ""
        self.lock = threading.Lock()
        self.raw = None             # (vx, vy, wz), receive time of the last cmd_vel_raw
        self.out = (0.0, 0.0, 0.0)  # last command sent
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.blocked_pub = rospy.Publisher("~blocked", Bool, queue_size=1, latch=True)
        self.reason_pub = rospy.Publisher("~reason", String, queue_size=1, latch=True)
        for name, topic, kind in (("dist", "obstacle_distances", Float32MultiArray), ("near", "obstacle_near", Bool),
                                  ("sensors_ok", "obstacle_detector/sensors_ok", Bool),
                                  ("state", "route_supervisor/state", String)):
            rospy.Subscriber(topic, kind, self.on_input, callback_args=name, queue_size=1)
        rospy.Subscriber("cmd_vel_raw", Twist, self.on_cmd, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), lambda _: self.check())

    def on_input(self, msg, name):
        value = msg.data
        if name == "dist" and len(value) != N_DIRECTIONS:
            rospy.logwarn_throttle(5.0, "cmd_vel_gate: obstacle_distances needs %d values", N_DIRECTIONS)
            return
        self.inputs[name] = (value, rospy.get_time())

    def current(self, name, timeout):
        """Value of an input if received within timeout, else None."""
        entry = self.inputs.get(name)
        if entry is None:
            return None
        age = rospy.get_time() - entry[1]
        return entry[0] if 0 <= age <= timeout else None

    def decide(self, cmd):
        dist = self.current("dist", self.timeout)
        near = self.current("near", self.timeout)
        sensors_ok = self.current("sensors_ok", self.timeout)
        fresh = dist is not None and near is not None and sensors_ok is not None
        avoid_active = self.current("state", self.state_timeout) == "AVOID"
        speed = (self.out[0] ** 2 + self.out[1] ** 2) ** 0.5
        return gate_command(cmd, fresh, bool(sensors_ok), list(dist) if fresh else [0.0] * N_DIRECTIONS,
                            bool(near), avoid_active, speed, **self.field)

    def report(self, reason):
        reason = reason or ""
        if reason != self.reason:
            self.reason = reason
            self.blocked_pub.publish(Bool(data=bool(reason)))
            self.reason_pub.publish(String(data=reason))
            state = self.current("state", self.state_timeout)
            rospy.loginfo("cmd_vel_gate: %s (supervisor %s)", ("STOP, " + reason) if reason else "pass", state)

    def send(self, out):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = out
        self.pub.publish(msg)
        self.out = out

    def check(self):
        """Periodic: report whether driving forward would pass, and judge the command in force again."""
        with self.lock:
            _, reason = self.decide((max(self.out[0], 0.3), 0.0, 0.0))
            self.report(reason)
            if self.raw is None or not 0 <= rospy.get_time() - self.raw[1] <= self.raw_timeout:
                out, why = (0.0, 0.0, 0.0), "no current cmd_vel_raw"
            else:
                out, why = self.decide(self.raw[0])
            # stops go out at once; moving (again) waits for the next raw command
            if out != self.out and all(abs(o) <= abs(last) + 1e-9 for o, last in zip(out, self.out)):
                rospy.loginfo_throttle(2.0, "cmd_vel_gate: command in force stopped, %s", why)
                self.send(out)

    def on_cmd(self, msg):
        cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        with self.lock:
            out, reason = self.decide(cmd)
            self.raw = (cmd, rospy.get_time())
            self.send(out)
        if reason:
            rospy.loginfo_throttle(2.0, "cmd_vel_gate: command stopped, %s", reason)


if __name__ == "__main__":
    rospy.init_node("cmd_vel_gate")
    CmdVelGate()
    rospy.spin()
