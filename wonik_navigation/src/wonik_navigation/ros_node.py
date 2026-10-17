"""Small shared helpers for the ROS 2 nodes (parameters, time and formatted logging)."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rcl_interfaces.msg import ParameterDescriptor

def latched_qos():
    return QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                      reliability=ReliabilityPolicy.RELIABLE)

class ManagedNode(Node):
    def param(self, name, default=None):
        name = name.lstrip("~").lstrip("/").replace("/", ".")
        if not self.has_parameter(name):
            self.declare_parameter(name, default, ParameterDescriptor(dynamic_typing=True))
        value = self.get_parameter(name).value
        return default if value is None else value

    def seconds(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def info(self, fmt, *args):
        self.get_logger().info(fmt % args if args else fmt)

    def warn(self, fmt, *args):
        self.get_logger().warning(fmt % args if args else fmt)

    def warn_throttle(self, period, fmt, *args):
        self.get_logger().warning(fmt % args if args else fmt, throttle_duration_sec=float(period))

    def info_throttle(self, period, fmt, *args):
        self.get_logger().info(fmt % args if args else fmt, throttle_duration_sec=float(period))

def spin_node(cls):
    rclpy.init()
    node = cls()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
