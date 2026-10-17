#!/usr/bin/env python3
"""cmd_vel gate between move_base_flex and the base: passes cmd_vel_raw to cmd_vel according to
wonik_navigation.safety.gate_command:

  - stops everything while the obstacle detector is silent or a required sensor is stale (every state)
  - passes everything while the route supervisor reports a running detour (AVOID): the avoid
    controller checks its own motion against the costmap
  - otherwise stops translation for an obstacle in the front box, and turning in place for an
    obstacle within the turn radius

The supervisor state counts only while it is current: the supervisor publishes it at 10 Hz, and a state
older than ~state_timeout (supervisor dead or stuck) is treated as "no detour". Without a supervisor the
gate always applies the obstacle stops.

Every 50 ms the command in force (the last cmd_vel_raw, or none once it is older than ~raw_timeout) is
judged again with the current inputs; when the result is more restrictive than what was last sent
(e.g. an obstacle came within the turn radius during a turn, a sensor failed, or move_base_flex went
silent), it is sent at once instead of waiting for the next raw command or the base's own timeout.

Parameters
  ~timeout        [s] detector messages older than this: detector silent (default 0.5)
  ~state_timeout  [s] supervisor state older than this is ignored (default 0.5)
  ~raw_timeout    [s] a cmd_vel_raw older than this is no longer in force: zero is sent (default 0.5)
Topics
  cmd_vel_raw (in), obstacle_detected, obstacle_near, obstacle_detector/sensors_ok (in),
  route_supervisor/state (in), cmd_vel (out)
  ~blocked    (std_msgs/Bool, out, latched) translation would be stopped now
  ~reason     (std_msgs/String, out, latched) why (empty when it would pass)
"""
import threading

import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from wonik_navigation.ros_node import ManagedNode, latched_qos, spin_node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from wonik_navigation.safety import gate_command


class CmdVelGate(ManagedNode):
    def __init__(self):
        super().__init__("cmd_vel_gate")
        self.timeout = self.param("~/timeout", 0.5)
        self.state_timeout = self.param("~/state_timeout", 0.5)
        self.raw_timeout = self.param("~/raw_timeout", 0.5)
        self.inputs = {}  # name -> (value, receive time)
        self.reason = ""
        self.lock = threading.Lock()
        self.raw = None             # (vx, vy, wz), receive time of the last cmd_vel_raw
        self.out = (0.0, 0.0, 0.0)  # last command sent
        self.pub = self.create_publisher(Twist, "cmd_vel", 1)
        self.blocked_pub = self.create_publisher(Bool, "~/blocked", latched_qos())
        self.reason_pub = self.create_publisher(String, "~/reason", latched_qos())
        for name, topic, kind in (("detected", "obstacle_detected", Bool), ("near", "obstacle_near", Bool),
                                  ("sensors_ok", "obstacle_detector/sensors_ok", Bool),
                                  ("state", "route_supervisor/state", String)):
            self.create_subscription(kind, topic, lambda msg, key=name: self.on_input(msg, key), 1)
        self.create_subscription(Twist, "cmd_vel_raw", self.on_cmd, 1)
        self.create_timer(0.05, self.check)

    def on_input(self, msg, name):
        self.inputs[name] = (msg.data, self.seconds())

    def current(self, name, timeout):
        """Value of an input if received within timeout, else None."""
        entry = self.inputs.get(name)
        if entry is None:
            return None
        age = self.seconds() - entry[1]
        return entry[0] if 0 <= age <= timeout else None

    def decide(self, cmd):
        detected = self.current("detected", self.timeout)
        near = self.current("near", self.timeout)
        sensors_ok = self.current("sensors_ok", self.timeout)
        fresh = detected is not None and near is not None and sensors_ok is not None
        avoid_active = self.current("state", self.state_timeout) == "AVOID"
        return gate_command(cmd, fresh, bool(sensors_ok), bool(detected), bool(near), avoid_active)

    def report(self, reason):
        reason = reason or ""
        if reason != self.reason:
            self.reason = reason
            self.blocked_pub.publish(Bool(data=bool(reason)))
            self.reason_pub.publish(String(data=reason))
            state = self.current("state", self.state_timeout)
            self.info("cmd_vel_gate: %s (supervisor %s)", ("STOP, " + reason) if reason else "pass", state)

    def send(self, out):
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = out
        self.pub.publish(msg)
        self.out = out

    def check(self):
        """Periodic: report whether translation would pass, and judge the command in force again."""
        _, reason = self.decide((1.0, 0.0, 0.0))
        self.report(reason)
        with self.lock:
            if self.raw is None or not 0 <= self.seconds() - self.raw[1] <= self.raw_timeout:
                out, why = (0.0, 0.0, 0.0), "no current cmd_vel_raw"
            else:
                out, why = self.decide(self.raw[0])
            # stops go out at once; moving (again) waits for the next raw command
            if out != self.out and all(abs(o) <= abs(last) + 1e-9 for o, last in zip(out, self.out)):
                self.info_throttle(2.0, "cmd_vel_gate: command in force stopped, %s", why)
                self.send(out)

    def on_cmd(self, msg):
        cmd = (msg.linear.x, msg.linear.y, msg.angular.z)
        out, reason = self.decide(cmd)
        with self.lock:
            self.raw = (cmd, self.seconds())
            self.send(out)
        if reason and reason != self.reason:
            self.info_throttle(2.0, "cmd_vel_gate: command stopped, %s", reason)


if __name__ == "__main__":
    spin_node(CmdVelGate)
