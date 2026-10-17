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
import rospy
import tf2_ros
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, Float32MultiArray, String
from tf.transformations import quaternion_matrix
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


class ObstacleDetector:
    def __init__(self):
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        box = rospy.get_param("~box", {})
        self.lo = np.array([box.get("x_min", 0.45), box.get("y_min", -0.45), box.get("z_min", 0.06)])
        self.hi = np.array([box.get("x_max", 1.95), box.get("y_max", 0.45), box.get("z_max", 1.80)])
        body = rospy.get_param("~body", {})
        self.half_length = max(abs(body.get("x_min", -0.452)), abs(body.get("x_max", 0.452)))
        self.half_width = max(abs(body.get("y_min", -0.352)), abs(body.get("y_max", 0.352)))
        self.side_margin = rospy.get_param("~side_margin", 0.1)
        self.max_range = rospy.get_param("~max_range", 3.0)
        self.turn_radius = rospy.get_param("~turn_radius", 0.70)
        self.min_points = rospy.get_param("~min_points", 5)
        self.min_valid = rospy.get_param("~min_valid_points", 100)
        required = rospy.get_param("~required_topics", rospy.get_param("~cloud_topics", ["/points"]))
        optional = rospy.get_param("~optional_topics", [])
        self.tracker = SensorTracker(required, optional, timeout=rospy.get_param("~timeout", 0.5),
                                     future_tolerance=rospy.get_param("~future_tolerance", 0.05),
                                     min_points=self.min_points)

        self.lock = threading.Lock()
        self.updates = 0
        self.bad_clouds = 0
        self.last_now = None
        self.last_problems = None
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf)
        self.dist_pub = rospy.Publisher("obstacle_distances", Float32MultiArray, queue_size=1)
        self.near_pub = rospy.Publisher("obstacle_near", Bool, queue_size=1)
        self.pub = rospy.Publisher("obstacle_detected", Bool, queue_size=1)
        self.ok_pub = rospy.Publisher("~sensors_ok", Bool, queue_size=1)
        self.status_pub = rospy.Publisher("~status", String, queue_size=1)
        self.marker_pub = rospy.Publisher("~box_marker", Marker, queue_size=1, latch=True)
        for t in list(required) + list(optional):
            rospy.Subscriber(t, PointCloud2, self.on_cloud, callback_args=t, queue_size=1, buff_size=2 ** 24)
        rospy.Timer(rospy.Duration(0.05), lambda _: self.publish())
        rospy.Timer(rospy.Duration(10.0), self.log_rate)
        self.last_log = rospy.get_time()
        self.publish_marker()
        rospy.loginfo("obstacle_detector: body +-%.3f x +-%.3f m, corridors +%.2f m wide, %.1f m long, heights "
                      "[%.2f, %.2f], turn radius %.2f m; required %s, optional %s", self.half_length, self.half_width,
                      self.side_margin, self.max_range, self.lo[2], self.hi[2], self.turn_radius, required, optional)

    def on_cloud(self, msg, topic):
        try:
            t = self.tf.lookup_transform(self.base_frame, msg.header.frame_id, rospy.Time(0))
            q, p = t.transform.rotation, t.transform.translation
            m = quaternion_matrix([q.x, q.y, q.z, q.w])
            xyz = cloud_xyz(msg) @ m[:3, :3].T + np.array([p.x, p.y, p.z])
        except (tf2_ros.TransformException, ValueError) as e:
            self.bad_clouds += 1
            rospy.logwarn_throttle(5.0, "obstacle_detector: %s cloud not usable: %s", topic, e)
            return
        valid = len(xyz) >= self.min_valid
        if not valid:
            rospy.logwarn_throttle(2.0, "obstacle_detector: %s frame has only %d valid points: no measurement",
                                   topic, len(xyz))
        pts = xyz[(xyz[:, 2] >= self.lo[2]) & (xyz[:, 2] <= self.hi[2])]
        x, y = pts[:, 0], pts[:, 1]
        dist = corridor_distances(x, y, self.half_length, self.half_width, self.side_margin, self.max_range,
                                  self.min_points)
        in_body = (np.abs(x) <= self.half_length) & (np.abs(y) <= self.half_width)
        near = int(np.count_nonzero((np.hypot(x, y) <= self.turn_radius) & ~in_body))
        with self.lock:
            if self.tracker.update(topic, msg.header.stamp.to_sec(), rospy.get_time(), dist, near, valid):
                self.updates += 1
            else:
                rospy.logwarn_throttle(5.0, "obstacle_detector: %s cloud rejected (stamp %.3f, now %.3f)", topic,
                                       msg.header.stamp.to_sec(), rospy.get_time())
        self.publish()

    def evaluate(self):
        now = rospy.get_time()
        with self.lock:
            if self.last_now is not None and now < self.last_now - 1.0:
                rospy.logwarn("obstacle_detector: clock jumped back, dropping all sensor data")
                self.tracker.reset()
            self.last_now = now
            return self.tracker.evaluate(now)

    def publish(self):
        dist, near, sensors_ok, problems = self.evaluate()
        self.dist_pub.publish(Float32MultiArray(data=dist))
        self.near_pub.publish(Bool(data=near))
        self.pub.publish(Bool(data=dist[0] <= self.hi[0] - self.half_length))
        self.ok_pub.publish(Bool(data=sensors_ok))
        status = "; ".join(problems) if problems else "ok"
        self.status_pub.publish(String(data=status))
        if status != self.last_problems:
            (rospy.loginfo if status == "ok" else rospy.logwarn)("obstacle_detector: sensors %s", status)
            self.last_problems = status

    def log_rate(self, _):
        now = rospy.get_time()
        with self.lock:
            n, self.updates = self.updates, 0
        dist, near, sensors_ok, _ = self.evaluate()
        rospy.loginfo("obstacle_detector: %.1f cloud updates/s, distances %s, near=%s, sensors_ok=%s, rejected %d, "
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
                mk.points += [Point(a[0], a[1], z), Point(b[0], b[1], z)]
        for x, y in c:
            mk.points += [Point(x, y, self.lo[2]), Point(x, y, self.hi[2])]
        n = 48  # turn radius circle at the bottom of the box
        for i in range(n):
            a, b = 2 * math.pi * i / n, 2 * math.pi * (i + 1) / n
            mk.points += [Point(self.turn_radius * math.cos(a), self.turn_radius * math.sin(a), self.lo[2]),
                          Point(self.turn_radius * math.cos(b), self.turn_radius * math.sin(b), self.lo[2])]
        self.marker_pub.publish(mk)


if __name__ == "__main__":
    rospy.init_node("obstacle_detector")
    ObstacleDetector()
    rospy.spin()
