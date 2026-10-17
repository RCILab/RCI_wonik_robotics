#!/usr/bin/env python3
"""Obstacle detector: counts the points of the LiDAR and depth-camera clouds that fall inside a
detection box in front of the robot and publishes /obstacle_detected (std_msgs/Bool).

Parameters
  ~cloud_topics  PointCloud2 topics to check (default [/points, /camera/depth/points])
  ~base_frame    frame of the box (default base_footprint)
  ~box           {x_min, x_max, y_min, y_max, z_min, z_max} [m] in base_frame
  ~min_points    points inside the box that count as an obstacle (per cloud)
  ~timeout       [s] a cloud older than this is ignored; if no cloud is fresh the detector reports an
                 obstacle (fail-safe)
Topics
  obstacle_detected  (std_msgs/Bool) at every cloud and at 20 Hz
  ~box_marker        (visualization_msgs/Marker, latched) the box, for RViz
The detection rate (fresh cloud updates per second) is logged every 10 s.
"""
import threading

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool
from tf.transformations import quaternion_matrix
from visualization_msgs.msg import Marker

FLOAT32 = 7  # sensor_msgs/PointField.FLOAT32


def cloud_xyz(msg):
    """N x 3 float array of the finite x, y, z of a PointCloud2 (float32 fields)."""
    fields = {f.name: f for f in msg.fields}
    if any(fields[k].datatype != FLOAT32 for k in "xyz"):
        raise ValueError("x/y/z must be float32")
    dtype = np.dtype({"names": ["x", "y", "z"], "formats": ["<f4"] * 3,
                      "offsets": [fields[k].offset for k in "xyz"], "itemsize": msg.point_step})
    pts = np.frombuffer(msg.data, dtype=dtype, count=msg.width * msg.height)
    xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(axis=1)]


class ObstacleDetector:
    def __init__(self):
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        box = rospy.get_param("~box", {})
        self.lo = np.array([box.get("x_min", 0.45), box.get("y_min", -0.45), box.get("z_min", 0.06)])
        self.hi = np.array([box.get("x_max", 1.95), box.get("y_max", 0.45), box.get("z_max", 1.80)])
        self.min_points = rospy.get_param("~min_points", 5)
        self.timeout = rospy.get_param("~timeout", 0.5)
        topics = rospy.get_param("~cloud_topics", ["/points", "/camera/depth/points"])

        self.lock = threading.Lock()
        self.latest = {}  # topic -> (stamp, count)
        self.updates = 0
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf)
        self.pub = rospy.Publisher("obstacle_detected", Bool, queue_size=1)
        self.marker_pub = rospy.Publisher("~box_marker", Marker, queue_size=1, latch=True)
        for t in topics:
            rospy.Subscriber(t, PointCloud2, self.on_cloud, callback_args=t, queue_size=1, buff_size=2 ** 24)
        rospy.Timer(rospy.Duration(0.05), lambda _: self.publish())
        rospy.Timer(rospy.Duration(10.0), self.log_rate)
        self.last_log = rospy.get_time()
        self.publish_marker()
        rospy.loginfo("obstacle_detector: box x[%.2f, %.2f] y[%.2f, %.2f] z[%.2f, %.2f] on %s",
                      self.lo[0], self.hi[0], self.lo[1], self.hi[1], self.lo[2], self.hi[2], ", ".join(topics))

    def on_cloud(self, msg, topic):
        try:
            t = self.tf.lookup_transform(self.base_frame, msg.header.frame_id, rospy.Time(0))
        except tf2_ros.TransformException as e:
            rospy.logwarn_throttle(5.0, "obstacle_detector: %s", e)
            return
        q, p = t.transform.rotation, t.transform.translation
        m = quaternion_matrix([q.x, q.y, q.z, q.w])
        xyz = cloud_xyz(msg) @ m[:3, :3].T + np.array([p.x, p.y, p.z])
        count = int(np.count_nonzero(np.all((xyz >= self.lo) & (xyz <= self.hi), axis=1)))
        with self.lock:
            self.latest[topic] = (rospy.get_time(), count)
            self.updates += 1
        self.publish()

    def detected(self):
        now = rospy.get_time()
        with self.lock:
            fresh = [c for stamp, c in self.latest.values() if now - stamp <= self.timeout]
        if not fresh:
            return True  # no sensor data: assume blocked
        return any(c >= self.min_points for c in fresh)

    def publish(self):
        self.pub.publish(Bool(data=self.detected()))

    def log_rate(self, _):
        now = rospy.get_time()
        with self.lock:
            n, self.updates = self.updates, 0
        rospy.loginfo("obstacle_detector: %.1f cloud updates/s, detected=%s", n / max(now - self.last_log, 1e-6),
                      self.detected())
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
        self.marker_pub.publish(mk)


if __name__ == "__main__":
    rospy.init_node("obstacle_detector")
    ObstacleDetector()
    rospy.spin()
