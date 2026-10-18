"""Regressions at the ROS2 callback boundary."""
import importlib.util
from pathlib import Path
import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Float32MultiArray, String

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
    rclpy.init(args=["--ros-args", "-p", "footprint:="+'"[[0.452,0.352],[-0.452,0.352],[-0.452,-0.352],[0.452,-0.352]]"'])
    yield
    rclpy.shutdown()

def gate():
    node = script("cmd_vel_gate").CmdVelGate()
    node.pub = Capture()
    for key, value in dict(sensors_ok=True, state="FOLLOW").items():
        node.on_input(String(data=value) if isinstance(value, str) else Bool(data=value), key)
    node.on_input(Float32MultiArray(data=[float("inf")]*8), "dist")
    node.on_input(Float32MultiArray(data=[float("inf")]*2), "turn")
    return node

def test_gate_stops_existing_turn_on_near_obstacle(ros):
    node = gate()
    try:
        cmd = Twist()
        cmd.angular.z = 0.5
        node.on_cmd(cmd)
        node.on_input(Float32MultiArray(data=[0.0, 0.0]), "turn")
        node.check()
        assert node.pub.messages[-1].angular.z == 0
        assert len(node.pub.messages) == 2
    finally:
        node.destroy_node()

def test_gate_stops_on_sensor_failure_even_if_translation_already_blocked(ros):
    node = gate()
    try:
        node.on_input(Float32MultiArray(data=[0.1] + [float("inf")]*7), "dist")
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
        node.on_cloud(cloud([[1.0,0.0,0.3]]*200), "/points_self_filtered")
        dist, turn, ok, _ = node.evaluate()
        assert dist[0] < 0.5 and all(a > 0 for a in turn) and ok
        node.on_cloud(cloud([[float("nan")]*3]*200), "/points_self_filtered")
        assert node.evaluate()[:3] == ([0.0]*8, [0.0, 0.0], False)
    finally:
        node.destroy_node()

@pytest.mark.parametrize("endian", ["<", ">"])
def test_self_filter_preserves_fields_stamps_and_padded_rows(endian):
    import struct
    msg = PointCloud2()
    msg.header.frame_id = "lidar"
    msg.header.stamp.sec = 42
    msg.height, msg.width, msg.point_step, msg.row_step = 2, 2, 16, 36
    msg.is_bigendian = endian == ">"
    msg.fields = [PointField(name=k, offset=i*4, datatype=7, count=1) for i,k in enumerate(("x","y","z","intensity"))]
    # Transform by +1 m in X. The point at -1 is on the body; the NaN is not a measurement.
    rows = [[(-1.,0.,0.,11.), (2.,0.,0.,22.)], [(float("nan"),0.,0.,33.),(-3.,0.,0.,44.)]]
    msg.data = b"".join(b"".join(struct.pack(endian+"ffff", *p) for p in row)+b"PAD!" for row in rows)
    matrix = np.eye(4)
    matrix[0,3] = 1.
    out = script("self_filter").crop_cloud(msg, matrix, [-.5]*3, [.5]*3, True, "base_footprint")
    assert out.header.stamp == msg.header.stamp
    assert out.header.frame_id == "base_footprint"
    assert (out.height,out.width,out.row_step,out.point_step)==(1,2,32,16)
    values = list(struct.iter_unpack(endian+"ffff",bytes(out.data)))
    assert values == [(3.,0.,0.,22.),(-2.,0.,0.,44.)]
    assert out.is_bigendian == msg.is_bigendian and out.fields == msg.fields
