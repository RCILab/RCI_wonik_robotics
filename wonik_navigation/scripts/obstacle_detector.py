#!/usr/bin/env python3
"""Obstacle detector: from the LiDAR and depth-camera clouds, the distance from the body to the nearest
obstacle in each of 8 corridors around the robot (front, front-left, left, ... at 45 deg steps), whether
there is an obstacle within the circle the corners sweep when turning in place, and whether the sensors
deliver current, valid data (wonik_navigation.safety). cmd_vel_gate turns this into stops.

Freshness is judged by the measurement time (header stamp), not by when a cloud arrives: a cloud older
than ~timeout is not current, one stamped in the future (clock offset) is rejected, and a cloud with
fewer than ~min_valid_points finite points is no measurement.

Parameters
  ~required_topics  PointCloud2 topics that must be current (default [/points]); ~cloud_topics is an
                    older name for this list
  ~optional_topics  topics used while current (default []); see SensorTracker for a stale one
  ~base_frame       frame of the regions (default base_footprint)
  ~body             {x_min, x_max, y_min, y_max} [m] the robot (symmetric about base_frame)
  ~box              {x_min, x_max, y_min, y_max, z_min, z_max} [m]: z_min/z_max are the obstacle heights;
                    an obstacle in front closer than x_max (from base_frame) sets obstacle_detected
  ~side_margin      [m] corridors are this much wider than the body on each side (default 0.1)
  ~max_range        [m] corridor length from the body (default 3.0)
  ~turn_radius      [m] obstacle points closer than this to the robot centre (outside ~body) stop turning
                    in place (default 0.70: corner radius 0.57 + margin)
  ~min_points       points that count as an obstacle (per cloud and region)
  ~min_valid_points a cloud with fewer finite points is not a valid measurement (sensor not current)
  ~timeout          [s] maximum age of a cloud's measurement time
  ~future_tolerance [s] a stamp may lead this node's clock by this much
Topics (at every cloud and at 20 Hz)
  obstacle_distances  (std_msgs/Float32MultiArray) 8 corridor distances [m] (inf: none); all 0 while a
                      required sensor is not current
  obstacle_near       (std_msgs/Bool) obstacle within the turn radius (also while a required sensor is not current)
  obstacle_detected   (std_msgs/Bool) obstacle in the front box (for RViz and monitoring)
  ~sensors_ok         (std_msgs/Bool) every required sensor current
  ~status             (std_msgs/String) problems, for humans
  ~box_marker         (visualization_msgs/Marker, latched) front box and turn radius, for RViz
If the ROS clock jumps back (simulation reset) all data is dropped.
"""
import math
import threading

import numpy as np
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from wonik_navigation.ros_node import ManagedNode, latched_qos, spin_node
import tf2_ros
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32MultiArray, String
from tf_transformations import quaternion_matrix
from visualization_msgs.msg import Marker

from wonik_navigation.safety import SensorTracker, corridor_distances

FLOAT32 = 7  # sensor_msgs/PointField.FLOAT32


def cloud_xyz(msg):
    """N x 3 float array of the finite x, y, z of a PointCloud2 (float32 fields)."""
    fields = {f.name: f for f in msg.fields}
    if any(k not in fields or fields[k].datatype != FLOAT32 for k in "xyz"):
        raise ValueError("x/y/z float32 fields required")
    n = msg.width * msg.height
    if len(msg.data) < n * msg.point_step:
        raise ValueError("data shorter than width * height * point_step")
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                      "offsets": [fields[k].offset for k in "xyz"], "itemsize": msg.point_step})
    pts = np.frombuffer(msg.data, dtype=dtype, count=n)
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


class ObstacleDetector(ManagedNode):
    def __init__(self):
        super().__init__("obstacle_detector")
        self.base_frame = self.param("~base_frame", "base_footprint")
        box = {k: self.param("box." + k, v) for k, v in dict(x_min=0.45, x_max=1.95, y_min=-0.45, y_max=0.45, z_min=0.06, z_max=1.80).items()}
        self.lo = np.array([box.get("x_min", 0.45), box.get("y_min", -0.45), box.get("z_min", 0.06)])
        self.hi = np.array([box.get("x_max", 1.95), box.get("y_max", 0.45), box.get("z_max", 1.80)])
        body = {k: self.param("body." + k, v) for k, v in dict(x_min=-0.452, x_max=0.452, y_min=-0.352, y_max=0.352).items()}
        self.half_length = max(abs(body.get("x_min", -0.452)), abs(body.get("x_max", 0.452)))
        self.half_width = max(abs(body.get("y_min", -0.352)), abs(body.get("y_max", 0.352)))
        self.side_margin = self.param("~side_margin", 0.1)
        self.max_range = self.param("~max_range", 3.0)
        self.turn_radius = self.param("~turn_radius", 0.70)
        self.min_points = self.param("~min_points", 5)
        self.min_valid = self.param("~min_valid_points", 100)
        required = self.param("~required_topics", self.param("~cloud_topics", ["/points"]))
        optional = self.param("~optional_topics", [])
        self.tracker = SensorTracker(required, optional, timeout=self.param("~timeout", 0.5),
                                     future_tolerance=self.param("~future_tolerance", 0.05),
                                     min_points=self.min_points)

        self.lock = threading.Lock()
        self.updates = 0
        self.bad_clouds = 0
        self.last_now = None
        self.last_problems = None
        self.tf = tf2_ros.Buffer(node=self)
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.dist_pub = self.create_publisher(Float32MultiArray, "obstacle_distances", 1)
        self.near_pub = self.create_publisher(Bool, "obstacle_near", 1)
        self.pub = self.create_publisher(Bool, "obstacle_detected", 1)
        self.ok_pub = self.create_publisher(Bool, "~/sensors_ok", 1)
        self.status_pub = self.create_publisher(String, "~/status", 1)
        self.marker_pub = self.create_publisher(Marker, "~/box_marker", latched_qos())
        for t in list(required) + list(optional):
            self.create_subscription(PointCloud2, t, lambda msg, topic=t: self.on_cloud(msg, topic), qos_profile_sensor_data)
        self.create_timer(0.05, self.publish)
        self.create_timer(10.0, self.log_rate)
        self.last_log = self.seconds()
        self.publish_marker()
        self.info("obstacle_detector: body +-%.3f x +-%.3f m, corridors +%.2f m wide, %.1f m long, heights "
                      "[%.2f, %.2f], turn radius %.2f m; required %s, optional %s", self.half_length, self.half_width,
                      self.side_margin, self.max_range, self.lo[2], self.hi[2], self.turn_radius, required, optional)

    def on_cloud(self, msg, topic):
        try:
            t = self.tf.lookup_transform(self.base_frame, msg.header.frame_id, Time())
            q, p = t.transform.rotation, t.transform.translation
            m = quaternion_matrix([q.x, q.y, q.z, q.w])
            xyz = cloud_xyz(msg) @ m[:3, :3].T + np.array([p.x, p.y, p.z])
        except (tf2_ros.TransformException, ValueError) as e:
            self.bad_clouds += 1
            self.warn_throttle(5.0, "obstacle_detector: %s cloud not usable: %s", topic, e)
            return
        valid = len(xyz) >= self.min_valid
        if not valid:
            self.warn_throttle(2.0, "obstacle_detector: %s frame has only %d valid points: no measurement",
                                   topic, len(xyz))
        pts = xyz[(xyz[:, 2] >= self.lo[2]) & (xyz[:, 2] <= self.hi[2])]
        x, y = pts[:, 0], pts[:, 1]
        dist = corridor_distances(x, y, self.half_length, self.half_width, self.side_margin, self.max_range,
                                  self.min_points)
        in_body = (np.abs(x) <= self.half_length) & (np.abs(y) <= self.half_width)
        near = int(np.count_nonzero((np.hypot(x, y) <= self.turn_radius) & ~in_body))
        with self.lock:
            if self.tracker.update(topic, (Time.from_msg(msg.header.stamp).nanoseconds * 1e-9), self.seconds(), dist, near, valid):
                self.updates += 1
            else:
                self.warn_throttle(5.0, "obstacle_detector: %s cloud rejected (stamp %.3f, now %.3f)", topic,
                                       (Time.from_msg(msg.header.stamp).nanoseconds * 1e-9), self.seconds())
        self.publish()

    def evaluate(self):
        now = self.seconds()
        with self.lock:
            if self.last_now is not None and now < self.last_now - 1.0:
                self.warn("obstacle_detector: clock jumped back, dropping all sensor data")
                self.tracker.reset()
            self.last_now = now
            return self.tracker.evaluate(now)

    def publish(self):
        dist, near, sensors_ok, problems = self.evaluate()
        self.dist_pub.publish(Float32MultiArray(data=dist))
        self.near_pub.publish(Bool(data=near))
        self.pub.publish(Bool(data=bool(dist[0] <= self.hi[0] - self.half_length)))
        self.ok_pub.publish(Bool(data=sensors_ok))
        status = "; ".join(problems) if problems else "ok"
        self.status_pub.publish(String(data=status))
        if status != self.last_problems:
            (self.info if status == "ok" else self.warn)("obstacle_detector: sensors %s", status)
            self.last_problems = status

    def log_rate(self):
        now = self.seconds()
        with self.lock:
            n, self.updates = self.updates, 0
        dist, near, sensors_ok, _ = self.evaluate()
        self.info("obstacle_detector: %.1f cloud updates/s, distances %s, near=%s, sensors_ok=%s, rejected %d, "
                      "unusable %d", n / max(now - self.last_log, 1e-6), " ".join("%.2f" % d for d in dist), near,
                      sensors_ok, self.tracker.rejected, self.bad_clouds)
        self.last_log = now

    def publish_marker(self):
        mk = Marker()
        mk.header.frame_id = self.base_frame
        mk.ns, mk.id, mk.type, mk.action = "obstacle_box", 0, Marker.LINE_LIST, Marker.ADD
        mk.scale.x = 0.02
        mk.color.r, mk.color.g, mk.color.b, mk.color.a = 1.0, 0.3, 0.0, 1.0
        mk.pose.orientation.w = 1.0
        c = [[self.lo[0], self.lo[1]], [self.hi[0], self.lo[1]], [self.hi[0], self.hi[1]], [self.lo[0], self.hi[1]]]
        for z in (self.lo[2], self.hi[2]):
            for i in range(4):
                a, b = c[i], c[(i + 1) % 4]
                mk.points += [Point(x=float(a[0]), y=float(a[1]), z=float(z)), Point(x=float(b[0]), y=float(b[1]), z=float(z))]
        for x, y in c:
            mk.points += [Point(x=float(x), y=float(y), z=float(self.lo[2])), Point(x=float(x), y=float(y), z=float(self.hi[2]))]
        n = 48  # turn radius circle at the bottom of the box
        for i in range(n):
            a, b = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
            mk.points += [Point(x=self.turn_radius * math.cos(a), y=self.turn_radius * math.sin(a), z=float(self.lo[2])),
                          Point(x=self.turn_radius * math.cos(b), y=self.turn_radius * math.sin(b), z=float(self.lo[2]))]
        self.marker_pub.publish(mk)


if __name__ == "__main__":
    spin_node(ObstacleDetector)
