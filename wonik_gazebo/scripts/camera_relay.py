#!/usr/bin/env python3
"""Switchable depth stream for the Gazebo fault-injection bench."""
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import SetBool
from wonik_navigation.ros_node import ManagedNode, spin_node


class CameraRelay(ManagedNode):
    def __init__(self):
        super().__init__("camera_relay")
        self.enabled = True
        self.pub = self.create_publisher(PointCloud2, "~/points", qos_profile_sensor_data)
        self.create_subscription(PointCloud2, "/camera/depth/points", self.forward, qos_profile_sensor_data)
        self.create_service(SetBool, "~/enabled", self.set_enabled)

    def forward(self, msg):
        if self.enabled:
            self.pub.publish(msg)

    def set_enabled(self, request, response):
        self.enabled = request.data
        response.success = True
        response.message = "stream enabled" if self.enabled else "stream disabled"
        return response


if __name__ == "__main__":
    spin_node(CameraRelay)
