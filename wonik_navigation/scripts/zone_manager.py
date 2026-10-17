#!/usr/bin/env python3
"""Zone manager: publishes the type of the zone the robot is in, from map-frame polygons.

Parameters
  ~zones       zones yaml: frame_id, zones: [{name, type (avoid | stop), polygon: [[x, y], ...]}]
  ~base_frame  robot frame (default base_footprint)
Topics
  current_zone       (std_msgs/String) zone type, or "none" outside every zone (10 Hz)
  ~current_zone_name (std_msgs/String) zone name, or "none"
  ~markers           (visualization_msgs/MarkerArray, latched) the polygons for RViz
"""
import rclpy
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from wonik_navigation.ros_node import ManagedNode, latched_qos, spin_node
import tf2_ros
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from wonik_navigation.zones import load_zones, zone_at

COLORS = {"avoid": (1.0, 0.6, 0.0), "stop": (0.9, 0.1, 0.1)}


class ZoneManager(ManagedNode):
    def __init__(self):
        super().__init__("zone_manager")
        self.frame, self.zones = load_zones(self.param("~/zones"))
        self.base_frame = self.param("~/base_frame", "base_footprint")
        self.tf = tf2_ros.Buffer(node=self)
        tf2_ros.TransformListener(self.tf, self)
        self.type_pub = self.create_publisher(String, "current_zone", 1)
        self.name_pub = self.create_publisher(String, "~/current_zone_name", 1)
        self.marker_pub = self.create_publisher(MarkerArray, "~/markers", latched_qos())
        self.last = None
        self.publish_markers()
        self.create_timer(0.1, self.update)
        self.info("zone_manager: %d zones (%s)", len(self.zones),
                      ", ".join("%s:%s" % (z["name"], z["type"]) for z in self.zones))

    def update(self):
        try:
            t = self.tf.lookup_transform(self.frame, self.base_frame, Time())
        except tf2_ros.TransformException:
            return
        z = zone_at(t.transform.translation.x, t.transform.translation.y, self.zones)
        ztype, name = (z["type"], z["name"]) if z else ("none", "none")
        self.type_pub.publish(String(data=ztype))
        self.name_pub.publish(String(data=name))
        if name != self.last:
            self.info("zone_manager: entered %s (%s)", name, ztype)
            self.last = name

    def publish_markers(self):
        arr = MarkerArray()
        for i, z in enumerate(self.zones):
            mk = Marker()
            mk.header.frame_id = self.frame
            mk.ns, mk.id, mk.type, mk.action = "zones", i, Marker.LINE_STRIP, Marker.ADD
            mk.scale.x = 0.05
            mk.color.r, mk.color.g, mk.color.b = COLORS.get(z["type"], (0.5, 0.5, 0.5))
            mk.color.a = 1.0
            mk.pose.orientation.w = 1.0
            mk.points = [Point(x=float(x), y=float(y), z=0.02) for x, y in z["polygon"] + z["polygon"][:1]]
            arr.markers.append(mk)
        self.marker_pub.publish(arr)


if __name__ == "__main__":
    spin_node(ZoneManager)
