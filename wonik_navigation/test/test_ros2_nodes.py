"""Regressions at the ROS2 callback boundary."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, String

def script(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).parents[1] / "scripts" / (name + ".py"))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module

class Capture:
    def __init__(self):
        self.messages = []
    def publish(self, msg):
        self.messages.append(msg)

@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()

def gate():
    node = script("cmd_vel_gate").CmdVelGate()
    node.pub = Capture()
    for key, value in dict(detected=False, near=False, sensors_ok=True, state="FOLLOW").items():
        node.on_input(String(data=value) if isinstance(value, str) else Bool(data=value), key)
    return node

def test_gate_stops_existing_turn_on_near_obstacle(ros):
    node = gate()
    try:
        cmd = Twist()
        cmd.angular.z = 0.5
        node.on_cmd(cmd)
        node.on_input(Bool(data=True), "near")
        node.check()
        assert node.pub.messages[-1].angular.z == 0
        assert len(node.pub.messages) == 2
    finally:
        node.destroy_node()

def test_gate_stops_on_sensor_failure_even_if_translation_already_blocked(ros):
    node = gate()
    try:
        node.on_input(Bool(data=True), "detected")
        node.check()
        cmd = Twist()
        cmd.angular.z = 0.5
        node.on_cmd(cmd)
        node.on_input(Bool(data=False), "sensors_ok")
        node.check()
        assert node.pub.messages[-1].angular.z == 0
    finally:
        node.destroy_node()

def test_gate_raw_timeout_and_no_automatic_restart(ros):
    node = gate()
    try:
        cmd = Twist()
        cmd.linear.x = 0.5
        node.on_cmd(cmd)
        node.raw = (node.raw[0], node.seconds() - 1.0)
        node.check()
        assert node.pub.messages[-1].linear.x == 0
        node.raw = (node.raw[0], node.seconds())
        node.check()
        assert node.pub.messages[-1].linear.x == 0
    finally:
        node.destroy_node()

def test_all_nan_cloud_is_invalid_and_does_not_clear_obstacle(ros):
    node = script("obstacle_detector").ObstacleDetector()
    try:
        class TF:
            def lookup_transform(self, *args):
                tf = TransformStamped()
                tf.transform.rotation.w = 1.0
                return tf
        node.tf = TF()
        def cloud(points):
            msg = PointCloud2()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = "base_footprint"
            msg.fields = [PointField(name=k, offset=i*4, datatype=7, count=1) for i,k in enumerate("xyz")]
            msg.width, msg.height, msg.point_step = len(points), 1, 12
            msg.row_step = len(points)*12
            msg.data = np.asarray(points, dtype="<f4").tobytes()
            return msg
        node.on_cloud(cloud([[1.0,0.0,0.3]]*200), "/points")
        assert node.evaluate()[:3] == (True, False, True)
        node.on_cloud(cloud([[float("nan")]*3]*200), "/points")
        assert node.evaluate()[:3] == (True, True, False)
    finally:
        node.destroy_node()
