"""Failed/timed-out/rejected Nav2 detours must never open the obstacle gate."""
import importlib.util
from pathlib import Path
import threading
import time
import pytest
import rclpy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import Twist, TransformStamped, PoseStamped
from nav2_msgs.action import FollowPath, ComputePathToPose
from std_msgs.msg import Bool, Float32MultiArray, String

ROOT = Path(__file__).parents[1]
def script(name):
    spec = importlib.util.spec_from_file_location(name, ROOT / "scripts" / (name + ".py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod

class MockNav2(Node):
    def __init__(self, mode):
        super().__init__("mock_nav2")
        self.mode, self.stopping = mode, False
        self.current = None
        self.controllers, self.states, self.output = [], [], []
        self.raw = self.create_publisher(Twist, "cmd_vel_raw", 1)
        self.dist = self.create_publisher(Float32MultiArray, "obstacle_distances", 1)
        self.near = self.create_publisher(Bool, "obstacle_near", 1)
        self.ok = self.create_publisher(Bool, "obstacle_detector/sensors_ok", 1)
        self.zone = self.create_publisher(String, "current_zone", 1)
        self.create_subscription(String, "route_supervisor/state", lambda m: self.states.append(m.data), 10)
        self.create_subscription(Twist, "cmd_vel", lambda m: self.output.append(m.linear.x), 10)
        self.create_timer(0.05, self.sensors)
        group = ReentrantCallbackGroup()
        self.follow = ActionServer(self, FollowPath, "follow_path", self.execute,
            goal_callback=self.accept, cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        self.plan = ActionServer(self, ComputePathToPose, "compute_path_to_pose", self.compute,
            cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=group)
        self.tf = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = "map", "base_footprint"
        tf.transform.rotation.w = 1.0
        tf.header.stamp = self.get_clock().now().to_msg()
        self.tf.sendTransform(tf)

    def sensors(self):
        self.dist.publish(Float32MultiArray(data=[0.1] + [float("inf")] * 7))
        self.near.publish(Bool(data=False))
        self.ok.publish(Bool(data=True))
        self.zone.publish(String(data="avoid"))

    def accept(self, request):
        self.controllers.append(request.controller_id)
        return GoalResponse.REJECT if self.mode == "rejected" and request.controller_id == "avoid" else GoalResponse.ACCEPT

    def execute(self, goal):
        self.current = goal
        while not self.stopping and rclpy.ok() and goal == self.current:
            if goal.is_cancel_requested:
                goal.canceled()
                return FollowPath.Result()
            goal.publish_feedback(FollowPath.Feedback())
            cmd = Twist()
            cmd.linear.x = 0.5
            self.raw.publish(cmd)
            time.sleep(0.05)
        if goal.is_active:
            goal.abort()
        return FollowPath.Result()

    def compute(self, goal):
        if self.mode == "timeout":
            while not self.stopping and not goal.is_cancel_requested and rclpy.ok():
                time.sleep(0.05)
            if goal.is_active:
                goal.canceled() if goal.is_cancel_requested else goal.abort()
            return ComputePathToPose.Result()
        if self.mode == "fail":
            goal.abort()
            return ComputePathToPose.Result()
        result = ComputePathToPose.Result()
        result.path.header.frame_id = "map"
        for i in range(41):
            p = PoseStamped()
            p.header.frame_id = "map"
            p.pose.position.x = 0.05*i
            p.pose.orientation.w = 1.0
            result.path.poses.append(p)
        goal.succeed()
        return result

@pytest.mark.parametrize("mode", ["timeout", "fail", "rejected", "ok"])
def test_supervisor_gate(mode):
    rclpy.init(args=["--ros-args", "-p", "route:=" + str(ROOT / "test/route_test.yaml"),
                    "-p", "zones:=" + str(ROOT / "test/zones_test.yaml"), "-p", "plan_timeout:=0.4",
                    "-p", "replan_period:=0.6"])
    mock = MockNav2(mode)
    supervisor = script("route_supervisor").RouteSupervisor()
    gate = script("cmd_vel_gate").CmdVelGate()
    executor = MultiThreadedExecutor(num_threads=8)
    for node in [mock, supervisor, gate]:
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        deadline = time.monotonic()+8
        while time.monotonic() < deadline:
            if mode == "ok" and "AVOID" in mock.states and any(v > 0.1 for v in mock.output):
                break
            if mode != "ok" and "AVOID_PREP" in mock.states and len(mock.output) > 30:
                break
            time.sleep(0.05)
        assert "stop_go" in mock.controllers
        assert "AVOID_PREP" in mock.states
        assert mock.output
        if mode == "ok":
            assert "AVOID" in mock.states
            assert any(v > 0.1 for v in mock.output)
        else:
            assert "AVOID" not in mock.states
            assert all(v == 0 for v in mock.output)
        if mode == "rejected":
            assert "avoid" in mock.controllers
            assert mock.controllers.count("stop_go") >= 2
    finally:
        supervisor.done = True
        supervisor.goal_seq += 1
        supervisor.cancel_plan()
        mock.stopping = True
        # Let in-flight action executions return while their publishers still exist.
        time.sleep(0.3)
        executor.shutdown(timeout_sec=5)
        thread.join(timeout=2)
        supervisor.destroy_node()
        gate.destroy_node()
        mock.destroy_node()
        rclpy.shutdown()
