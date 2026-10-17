#!/usr/bin/env python3
"""Obstacle detector: from the LiDAR and depth-camera clouds, the distance from the body to the nearest
obstacle in each of 8 corridors around the robot (front, front-left, left, ... at 45 deg steps), how far
the footprint can turn in place each way before it touches an obstacle, and whether the sensors deliver
current, valid data (wonik_navigation.safety). cmd_vel_gate turns this into stops.

Freshness is judged by the measurement time (header stamp), not by when a cloud arrives: a cloud older
than ~timeout is not current, one stamped in the future (clock offset) is rejected, and a cloud with
fewer than ~min_valid_points finite points is no measurement.

Parameters
  ~required_topics  PointCloud2 topics that must be current (default [/points_self_filtered]: the LiDAR
                    without the robot's own points, scan.launch); ~cloud_topics is an
                    older name for this list
  ~optional_topics  topics used while current (default []); see SensorTracker for a stale one
  ~base_frame       frame of the regions (default base_footprint); the robot turns in place about its
                    origin
  ~footprint        [[x, y], ...] [m] the robot's outline in base_frame, as in the costmaps (any polygon,
                    need not be centred on base_frame). The corridors start at its bounding box; turns
                    use the polygon itself. Points inside it are the robot and are left out.
  ~box              {x_min, x_max, y_min, y_max, z_min, z_max} [m]: z_min/z_max are the obstacle heights;
                    an obstacle in front closer than x_max (from base_frame) sets obstacle_detected
  ~side_margin      [m] corridors are this much wider than the body on each side (default 0.1)
  ~max_range        [m] corridor length from the body (default 3.0)
  ~min_points       points that count as an obstacle (per cloud and region)
  ~min_valid_points a cloud with fewer finite points is not a valid measurement (sensor not current)
  ~timeout          [s] maximum age of a cloud's measurement time
  ~future_tolerance [s] a stamp may lead this node's clock by this much
Topics (at every cloud and at 20 Hz)
  obstacle_distances  (std_msgs/Float32MultiArray) 8 corridor distances [m] (inf: none); all 0 while a
                      required sensor is not current
  obstacle_turn_clearance (std_msgs/Float32MultiArray) [counterclockwise, clockwise] turn in place [rad]
                      until the footprint touches an obstacle (inf: never); 0 while a required sensor is
                      not current
  obstacle_detected   (std_msgs/Bool) obstacle in the front box (for RViz and monitoring)
  ~sensors_ok         (std_msgs/Bool) every required sensor current
  ~status             (std_msgs/String) problems, for humans
  ~box_marker         (visualization_msgs/Marker, latched) front box and footprint, for RViz
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

from wonik_navigation.safety import SensorTracker, TurnClearance, bounding_box, corridor_distances

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
        self.footprint = rospy.get_param("~footprint")
        self.body = bounding_box(self.footprint)
        self.turns = TurnClearance(self.footprint)
        self.side_margin = rospy.get_param("~side_margin", 0.1)
        self.max_range = rospy.get_param("~max_range", 3.0)
        self.min_points = rospy.get_param("~min_points", 5)
        self.min_valid = rospy.get_param("~min_valid_points", 100)
        required = rospy.get_param("~required_topics", rospy.get_param("~cloud_topics", ["/points_self_filtered"]))
        optional = rospy.get_param("~optional_topics", [])
        self.tracker = SensorTracker(required, optional, timeout=rospy.get_param("~timeout", 0.5),
                                     future_tolerance=rospy.get_param("~future_tolerance", 0.05))

        self.lock = threading.Lock()
        self.updates = 0
        self.bad_clouds = 0
        self.last_now = None
        self.last_problems = None
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf)
        self.dist_pub = rospy.Publisher("obstacle_distances", Float32MultiArray, queue_size=1)
        self.turn_pub = rospy.Publisher("obstacle_turn_clearance", Float32MultiArray, queue_size=1)
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
        rospy.loginfo("obstacle_detector: body x [%.3f, %.3f] y [%.3f, %.3f] m, corridors +%.2f m wide, %.1f m "
                      "long, heights [%.2f, %.2f], footprint reaches %.2f m from the turning centre; required %s, "
                      "optional %s", *self.body, self.side_margin, self.max_range, self.lo[2], self.hi[2],
                      self.turns.r_max, required, optional)

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
        dist = corridor_distances(x, y, self.body, self.side_margin, self.max_range, self.min_points)
        turn = self.turns.clearance(x, y, self.min_points)
        with self.lock:
            if self.tracker.update(topic, msg.header.stamp.to_sec(), rospy.get_time(), dist, turn, valid):
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
        dist, turn, sensors_ok, problems = self.evaluate()
        self.dist_pub.publish(Float32MultiArray(data=dist))
        self.turn_pub.publish(Float32MultiArray(data=turn))
        self.pub.publish(Bool(data=dist[0] <= self.hi[0] - self.body[1]))
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
        dist, turn, sensors_ok, _ = self.evaluate()
        rospy.loginfo("obstacle_detector: %.1f cloud updates/s, distances %s, turn clearance %s deg, sensors_ok=%s, "
                      "rejected %d, unusable %d", n / max(now - self.last_log, 1e-6),
                      " ".join("%.2f" % d for d in dist), " ".join("%.0f" % math.degrees(a) for a in turn),
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
        fp = self.footprint  # footprint outline at the bottom of the box
        for a, b in zip(fp, fp[1:] + fp[:1]):
            mk.points += [Point(a[0], a[1], self.lo[2]), Point(b[0], b[1], self.lo[2])]
        self.marker_pub.publish(mk)


if __name__ == "__main__":
    rospy.init_node("obstacle_detector")
    ObstacleDetector()
    rospy.spin()
