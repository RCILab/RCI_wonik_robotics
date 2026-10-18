"""Exercise the real command gate over DDS when publishers become silent."""
import importlib.util
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray, String


def test_gate_timeouts():
    spec = importlib.util.spec_from_file_location(
        "timeout_gate", Path(__file__).parents[1] / "scripts" / "cmd_vel_gate.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init()
    gate = module.CmdVelGate()
    mock = rclpy.create_node("gate_timeout_inputs")
    executor = SingleThreadedExecutor()
    executor.add_node(gate)
    executor.add_node(mock)
    inputs = dict(state="AVOID", detector=True, sensors_ok=True, turn_blocked=False,
                  raw=(0.5, 0.0, 0.0))
    received = []
    state = mock.create_publisher(String, "/route_supervisor/state", 1)
    dist = mock.create_publisher(Float32MultiArray, "/obstacle_distances", 1)
    turn = mock.create_publisher(Float32MultiArray, "/obstacle_turn_clearance", 1)
    ok = mock.create_publisher(Bool, "/obstacle_detector/sensors_ok", 1)
    raw = mock.create_publisher(Twist, "/cmd_vel_raw", 1)
    mock.create_subscription(Twist, "/cmd_vel",
                             lambda m: received.append((m.linear.x, m.angular.z)), 10)

    def tick():
        if inputs["state"] is not None:
            state.publish(String(data=inputs["state"]))
        if inputs["detector"]:
            # Obstacle inside the protective field at 0.5 m/s.
            dist.publish(Float32MultiArray(data=[0.5] + [float("inf")] * 7))
            turn.publish(Float32MultiArray(data=[0.0]*2 if inputs["turn_blocked"] else [float("inf")]*2))
            ok.publish(Bool(data=inputs["sensors_ok"]))
        if inputs["raw"] is not None:
            cmd = Twist()
            cmd.linear.x, cmd.linear.y, cmd.angular.z = inputs["raw"]
            raw.publish(cmd)

    mock.create_timer(0.05, tick)

    def after(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=min(0.01, max(0.0, deadline - time.monotonic())))
        assert received, "gate never published a command"
        return received[-1]

    try:
        assert after(1.5) == (0.5, 0.0), "fresh AVOID must pass"
        inputs["state"] = None
        assert after(1.0) == (0.0, 0.0), "dead supervisor must close the gate"
        inputs["state"] = "AVOID"
        assert after(0.5) == (0.5, 0.0)
        inputs["sensors_ok"] = False
        assert after(0.5) == (0.0, 0.0), "sensor failure also stops a detour"
        inputs["sensors_ok"] = True
        assert after(0.5) == (0.5, 0.0)
        inputs["detector"] = False
        assert after(1.0) == (0.0, 0.0), "silent detector must stop motion"
        inputs.update(detector=True, state="AVOID_PREP")
        assert after(0.5) == (0.0, 0.0), "planning is not an active detour"

        inputs["raw"] = (0.0, 0.0, 0.4)
        assert after(0.5) == (0.0, 0.4)
        inputs.update(raw=None, turn_blocked=True)
        assert after(0.2) == (0.0, 0.0), "stop an active turn before raw timeout"

        inputs.update(turn_blocked=False, raw=(0.0, 0.0, 0.4))
        assert after(0.5) == (0.0, 0.4)
        inputs["raw"] = None
        assert after(0.3) == (0.0, 0.4), "command is still current"
        assert after(0.5) == (0.0, 0.0), "silent controller must time out"

        inputs.update(raw=(0.0, 0.0, 0.4), turn_blocked=True)
        assert after(0.3) == (0.0, 0.0)
        inputs["sensors_ok"] = False
        assert after(0.3) == (0.0, 0.0), "a second stop reason must remain stopped"
    finally:
        executor.shutdown()
        mock.destroy_node()
        gate.destroy_node()
        rclpy.shutdown()
