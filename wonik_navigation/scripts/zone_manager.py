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
import rospy
import tf2_ros
from geometry_msgs.msg import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from wonik_navigation.zones import load_zones, zone_at

COLORS = {"avoid": (1.0, 0.6, 0.0), "stop": (0.9, 0.1, 0.1)}


class ZoneManager:
    def __init__(self):
        self.frame, self.zones = load_zones(rospy.get_param("~zones"))
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf)
        self.type_pub = rospy.Publisher("current_zone", String, queue_size=1)
        self.name_pub = rospy.Publisher("~current_zone_name", String, queue_size=1)
        self.marker_pub = rospy.Publisher("~markers", MarkerArray, queue_size=1, latch=True)
        self.last = None
        self.publish_markers()
        rospy.Timer(rospy.Duration(0.1), self.update)
        rospy.loginfo("zone_manager: %d zones (%s)", len(self.zones),
                      ", ".join("%s:%s" % (z["name"], z["type"]) for z in self.zones))

    def update(self, _):
        try:
            t = self.tf.lookup_transform(self.frame, self.base_frame, rospy.Time(0))
        except tf2_ros.TransformException:
            return
        z = zone_at(t.transform.translation.x, t.transform.translation.y, self.zones)
        ztype, name = (z["type"], z["name"]) if z else ("none", "none")
        self.type_pub.publish(String(data=ztype))
        self.name_pub.publish(String(data=name))
        if name != self.last:
            rospy.loginfo("zone_manager: entered %s (%s)", name, ztype)
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
            mk.points = [Point(x, y, 0.02) for x, y in z["polygon"] + z["polygon"][:1]]
            arr.markers.append(mk)
        self.marker_pub.publish(arr)


if __name__ == "__main__":
    rospy.init_node("zone_manager")
    ZoneManager()
    rospy.spin()
