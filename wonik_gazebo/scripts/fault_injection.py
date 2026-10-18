#!/usr/bin/env python3
"""Fault injection on cobot_v2 in Gazebo (kinematic mode), one case per run:
    ros2 launch wonik_gazebo fault_injection.launch.py          # empty world, detector, gate, camera relay
    ros2 run wonik_gazebo fault_injection.py <case>          # another terminal
  b1  joint_states stop while driving         -> zero wheel speed within joint_state_timeout, resumes
                                                 through the measured start sequence
  b2  cmd_vel stops while driving             -> stop after cmd_timeout and braking
  c   drive controller not responding         -> no DRIVE from converged commands, FAULT latched until
                                                 ~clear_fault
  h   bad joint_states injected while driving -> NaN, future, late (50 ms, 2 s), duplicate stamps and
                                                 missing joints rejected; odometry and commands unaffected
  d   box beside the robot, turn in place through the gate -> turn stopped, driving along it allowed
  g   low box only the depth camera sees, then the camera stream is lost (relay disabled) -> stays stopped
Each case prints its measurements and PASS/FAIL. The depth camera needs an X display (Xvfb works).
"""
import math
import re
import argparse
import sys
import threading
import time

import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from wonik_navigation.ros_node import ManagedNode, latched_qos
from controller_manager_msgs.srv import SwitchController
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray, String
from std_srvs.srv import Trigger, SetBool
from tf_transformations import euler_from_quaternion

S = {"gt": None, "speed": 0.0, "mode": "", "fault": "", "wheel": None, "odom": None, "js": None, "reason": "",
     "sensors_ok": None, "detected": None, "turn": None, "drive_cmd": [], "bad": []}
lock = threading.Lock()


def yaw_of(q):
    return euler_from_quaternion([q.x, q.y, q.z, q.w])[2]


def on_gt(m):
    p = m.pose.pose
    S["gt"] = (p.position.x, p.position.y, yaw_of(p.orientation))
    S["speed"] = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)


def on_dbg(m):
    g = re.search(r"mode=(\S+)", m.data)
    S["mode"] = g.group(1) if g else ""
    S["dbg"] = m.data


def on_odom(key):
    def cb(m):
        p = m.pose.pose
        vals = (p.position.x, p.position.y, yaw_of(p.orientation), m.twist.twist.linear.x)
        if not all(math.isfinite(v) for v in vals):
            S["bad"].append(key)
        S[key] = vals
    return cb


def on_drive_cmd(m):
    if not all(math.isfinite(v) for v in m.data):
        S["bad"].append("drive_cmd")
    S["drive_cmd"].append(max(abs(v) for v in m.data) if m.data else 0.0)


def call(kind, name, request):
    client = node.create_client(kind, name)
    try:
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("Service unavailable: " + name)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if not future.done():
            raise TimeoutError("Service timed out: " + name)
        response = future.result()
        if hasattr(response, "ok") and not response.ok:
            raise RuntimeError("Controller switch rejected: " + name)
        return response
    finally:
        node.destroy_client(client)


def switch(start_controllers=(), stop_controllers=(), strictness=2):
    request = SwitchController.Request()
    request.activate_controllers = list(start_controllers)
    request.deactivate_controllers = list(stop_controllers)
    request.strictness = strictness
    request.timeout = Duration(seconds=5.0).to_msg()
    return call(SwitchController, "/controller_manager/switch_controller", request)


def camera_enabled(enabled):
    return call(SetBool, "/camera_relay/enabled", SetBool.Request(data=enabled))


def twist(vx=0.0, vy=0.0, wz=0.0):
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
    return t


def drive(cmd, seconds, pub=None, watch=None):
    """Publish cmd at 20 Hz for seconds; call watch(t) each tick; return modes seen."""
    pub = pub or cmd_pub
    modes = []
    t0 = node.seconds()
    next_tick = t0
    wall_deadline = time.monotonic() + max(15.0, seconds * 10.0)
    while rclpy.ok() and node.seconds() - t0 < seconds:
        if time.monotonic() > wall_deadline:
            raise TimeoutError("Simulation clock stopped or too slow")
        rclpy.spin_once(node, timeout_sec=0.01)
        now = node.seconds()
        if now < next_tick:
            continue
        next_tick = now + 0.05
        if cmd is not None:
            pub.publish(cmd)
        if not modes or modes[-1] != S["mode"]:
            modes.append(S["mode"])
        if watch:
            watch(now - t0)
    return modes


def result(ok, text):
    results.append(bool(ok))
    print("%s: %s" % ("PASS" if ok else "FAIL", text), flush=True)


def case_b1():
    drive(twist(0.5), 4.0)
    v0 = S["speed"]
    switch(start_controllers=[], stop_controllers=["joint_state_broadcaster"], strictness=2)
    stop_at = []
    modes = drive(twist(0.5), 3.0, watch=lambda t: stop_at.append(t) if S["speed"] < 0.02 and not stop_at else None)
    switch(start_controllers=["joint_state_broadcaster"], stop_controllers=[], strictness=2)
    modes2 = drive(twist(0.5), 8.0)
    v_after = S["speed"]
    drive(twist(), 2.0)
    t_stop = stop_at[0] if stop_at else float("inf")
    print("speed %.2f m/s; joint_states stopped -> modes %s, base stopped after %.2f s; restarted -> modes %s, "
          "speed %.2f m/s" % (v0, modes, t_stop, modes2, v_after))
    result("FEEDBACK_TIMEOUT" in modes and t_stop < 0.5 and v_after > 0.4,
           "stops within joint_state_timeout (0.2 s) + one control step, resumes with fresh feedback")


def case_b2():
    drive(twist(0.5), 4.0)
    p0, v0 = S["gt"], S["speed"]
    stop_at = []
    drive(None, 3.0, watch=lambda t: stop_at.append(t) if S["speed"] < 0.02 and not stop_at else None)
    p1 = S["gt"]
    t_stop = stop_at[0] if stop_at else float("inf")
    dist = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
    print("speed %.2f m/s; cmd_vel stopped -> base stopped after %.2f s, %.2f m (cmd_timeout 0.5 s + %.2f s braking "
          "at 0.6 m/s^2 = %.2f s)" % (v0, t_stop, dist, v0 / 0.6, 0.5 + v0 / 0.6))
    result(t_stop < 0.5 + v0 / 0.6 + 0.3, "stops after cmd_timeout and braking")


def case_c():
    drive(twist(), 2.0)
    switch(start_controllers=[], stop_controllers=["drive_controller"], strictness=2)
    p0 = S["gt"]
    modes = drive(twist(0.0, 0.2), 16.0)
    fault = S["fault"]
    modes_latched = drive(twist(0.3), 3.0)  # a new command must not clear it
    moved = math.hypot(S["gt"][0] - p0[0], S["gt"][1] - p0[1])
    switch(start_controllers=["drive_controller"], stop_controllers=[], strictness=2)
    drive(twist(), 1.0)
    cleared = call(Trigger, "/kinematics_node/clear_fault", Trigger.Request())
    p1 = S["gt"]
    modes_after = drive(twist(0.0, 0.2), 8.0)
    moved_after = math.hypot(S["gt"][0] - p1[0], S["gt"][1] - p1[1])
    drive(twist(), 2.0)
    started = modes.index("WAIT_STOP") if "WAIT_STOP" in modes else 0
    print("drives not responding: modes %s\n  fault: %s\n  with a new command: %s; moved %.3f m\n"
          "  clear_fault: %s (%s); after: modes %s, moved %.2f m" % (modes, fault, modes_latched, moved,
                                                                     cleared.success, cleared.message, modes_after,
                                                                     moved_after))
    result("DRIVE" not in modes[started + 1:] and modes[-1] == "FAULT" and set(modes_latched) == {"FAULT"}
           and moved < 0.01 and cleared.success and "DRIVE" in modes_after and moved_after > 0.5,
           "no DRIVE without measured wheel speed, FAULT latched until clear_fault")


def case_h():
    real = {}
    def on_real(m):
        if m.name and m.header.frame_id != "fake" and math.isfinite(sum(m.velocity)):
            real["m"], real["t"] = m, node.seconds()
    node.create_subscription(JointState, "/joint_states", on_real, qos_profile_sensor_data)
    fake = node.create_publisher(JointState, "/joint_states", 10)
    drive(twist(), 1.0)
    w0, g0 = S["wheel"], S["gt"]
    sent = {"nan": 0, "future": 0, "old": 0, "very_old": 0, "duplicate": 0, "missing": 0}
    kinds = list(sent)
    k = [0]

    def inject(_t):
        m = real.get("m")
        if m is None:
            return
        f = JointState()
        f.header.frame_id = "fake"
        f.name = list(m.name)
        f.position = list(m.position)
        f.velocity = [10.0] * len(m.velocity)  # wheels at 10 rad/s: would show up in odometry
        kind = kinds[k[0] % len(kinds)]
        k[0] += 1
        if kind == "nan":
            f.header.stamp = node.get_clock().now().to_msg()
            # JointState order differs between ros_control and ros2_control.
            # Invalidate every velocity so a required drive joint is always affected.
            f.velocity = [float("nan")] * len(f.velocity)
        elif kind == "future":
            f.header.stamp = (node.get_clock().now() + Duration(seconds=1.0)).to_msg()
        elif kind == "old":
            f.header.stamp = (Time.from_msg(m.header.stamp) - Duration(seconds=0.05)).to_msg()
        elif kind == "very_old":
            f.header.stamp = (Time.from_msg(m.header.stamp) - Duration(seconds=2.0)).to_msg()  # 2 s late, not a clock reset
        elif kind == "duplicate":
            time.sleep(0.005)  # let the kinematics node process the real sample first
            m = real["m"]
            f.header.stamp = m.header.stamp  # same stamp as an already processed real sample
        else:
            f.header.stamp = node.get_clock().now().to_msg()
            f.name, f.position, f.velocity = f.name[:1], f.position[:1], f.velocity[:1]
        fake.publish(f)
        sent[kind] += 1

    drive(twist(0.5), 6.0, watch=inject)
    drive(twist(), 2.0, watch=inject)
    w1, g1 = S["wheel"], S["gt"]
    dw = math.hypot(w1[0] - w0[0], w1[1] - w0[1])
    dg = math.hypot(g1[0] - g0[0], g1[1] - g0[1])
    counters = re.search(r"rejected\S+ stamps\S+", S.get("dbg", ""))
    print("injected %s\n  wheel odometry %.3f m vs ground truth %.3f m (error %.1f cm); max drive command %.2f rad/s; "
          "non-finite outputs: %s\n  %s" % (sent, dw, dg, 100 * abs(dw - dg), max(S["drive_cmd"][-200:]),
                                            S["bad"] or "none", counters.group(0) if counters else ""))
    result(abs(dw - dg) < 0.02 and not S["bad"] and max(S["drive_cmd"][-200:]) < 0.5 / 0.076 + 0.01,
           "bad samples rejected, odometry and commands unaffected")


def spawn(name, x, y, sx, sy, sz, yaw=0.0):
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, sz / 2
    p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    sdf = ("<?xml version='1.0'?><sdf version='1.6'><model name='{n}'><static>true</static><link name='l'>"
           "<collision name='c'><geometry><box><size>{a} {b} {c}</size></box></geometry></collision>"
           "<visual name='v'><geometry><box><size>{a} {b} {c}</size></box></geometry></visual></link></model></sdf>"
           ).format(n=name, a=sx, b=sy, c=sz)
    response = call(SpawnEntity, "/spawn_entity", SpawnEntity.Request(name=name, xml=sdf, initial_pose=p, reference_frame="world"))
    if not response.success:
        raise RuntimeError(response.status_message)
    spawned.add(name)


def delete(name):
    response = call(DeleteEntity, "/delete_entity", DeleteEntity.Request(name=name))
    if not response.success:
        raise RuntimeError(response.status_message)
    spawned.discard(name)


def case_d():
    x, y, yaw = S["gt"]
    # box along the left side, 0.15 m from the body (the corners reach it turning either way)
    spawn("side_box", x - 0.70 * math.sin(yaw), y + 0.70 * math.cos(yaw), 2.0, 0.4, 1.0, yaw)
    drive(None, 2.0)
    y0 = S["gt"][2]
    reasons = []
    drive(twist(wz=0.4), 6.0, pub=raw_pub, watch=lambda t: reasons.append(S["reason"]) if S["reason"] not in reasons else None)
    turned_blocked = math.degrees(abs(math.remainder(S["gt"][2] - y0, 2 * math.pi)))
    turn = S["turn"]
    p0 = S["gt"]
    drive(twist(0.3), 6.0, pub=raw_pub)  # includes re-aligning the wheels if they point elsewhere
    along = math.hypot(S["gt"][0] - p0[0], S["gt"][1] - p0[1])
    drive(twist(), 1.0, pub=raw_pub)
    delete("side_box")
    drive(None, 2.0)
    y1 = S["gt"][2]
    drive(twist(wz=0.4), 8.0, pub=raw_pub)
    turned_free = math.degrees(abs(math.remainder(S["gt"][2] - y1, 2 * math.pi)))
    drive(twist(), 1.0, pub=raw_pub)
    print("box beside: turn clearance %s deg, gate %s, turned %.1f deg; drove along it %.2f m; box removed: "
          "turned %.1f deg" % (["%.0f" % math.degrees(a) for a in turn] if turn else None,
                               [r for r in reasons if r], turned_blocked, along, turned_free))
    result(turn is not None and turn[0] < math.radians(30.0) and turned_blocked < 1.0 and along > 0.3 and
           turned_free > 30.0,
           "turning in place stopped next to the box, driving along it and turning without it allowed")


def case_g():
    x, y, yaw = S["gt"]
    spawn("low_box", x + 1.1 * math.cos(yaw), y + 1.1 * math.sin(yaw), 0.6, 0.6, 0.12, yaw)
    drive(None, 3.0)
    seen = (S["detected"], S["sensors_ok"])
    p0 = S["gt"]
    drive(twist(0.3), 2.0, pub=raw_pub)
    reason1 = S["reason"]
    camera_enabled(False)
    drive(None, 1.5)
    lost = (S["detected"], S["sensors_ok"])
    drive(twist(0.3), 3.0, pub=raw_pub)
    reason2 = S["reason"]
    moved = math.hypot(S["gt"][0] - p0[0], S["gt"][1] - p0[1])
    delete("low_box")
    print("camera sees the low box: detected/sensors_ok %s, gate '%s'; camera stream lost: detected/sensors_ok %s, "
          "gate '%s'; robot moved %.3f m" % (seen, reason1, lost, reason2, moved))
    result(seen == (True, True) and lost == (True, False) and moved < 0.02,
           "the LiDAR alone (clear) does not release the obstacle only the camera saw")


def main():
    global node, cmd_pub, raw_pub, results, spawned
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("case", choices=["b1", "b2", "c", "h", "d", "g", "all"])
    args = parser.parse_args(rclpy.utilities.remove_ros_args()[1:])
    rclpy.init()
    node = ManagedNode("fault_injection")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    results, spawned = [], set()
    cmd_pub = node.create_publisher(Twist, "/cmd_vel", 1)
    raw_pub = node.create_publisher(Twist, "/cmd_vel_raw", 1)
    subscriptions = [
        node.create_subscription(Odometry, "/ground_truth/odom", on_gt, qos_profile_sensor_data),
        node.create_subscription(String, "/kinematics_node/debug_state", on_dbg, 10),
        node.create_subscription(String, "/kinematics_node/fault", lambda m: S.update(fault=m.data), latched_qos()),
        node.create_subscription(Odometry, "/wheel/odom", on_odom("wheel"), qos_profile_sensor_data),
        node.create_subscription(Odometry, "/odom", on_odom("odom"), qos_profile_sensor_data),
        node.create_subscription(Float64MultiArray, "/drive_controller/commands", on_drive_cmd, 10),
        node.create_subscription(String, "/cmd_vel_gate/reason", lambda m: S.update(reason=m.data), latched_qos()),
        node.create_subscription(Bool, "/obstacle_detector/sensors_ok", lambda m: S.update(sensors_ok=m.data), 1),
        node.create_subscription(Bool, "/obstacle_detected", lambda m: S.update(detected=m.data), 1),
        node.create_subscription(Float32MultiArray, "/obstacle_turn_clearance", lambda m: S.update(turn=list(m.data)), 1),
    ]
    cases = dict(b1=case_b1, b2=case_b2, c=case_c, h=case_h, d=case_d, g=case_g)
    try:
        deadline = time.monotonic() + 30.0
        while S["gt"] is None or S["wheel"] is None or not S["mode"] or node.seconds() <= 0:
            if time.monotonic() > deadline:
                raise TimeoutError("Start fault_injection.launch.py in the same ROS domain first")
            rclpy.spin_once(node, timeout_sec=0.1)
        selected = cases if args.case == "all" else [args.case]
        for name in selected:
            S["drive_cmd"], S["bad"] = [], []
            print("CASE " + name, flush=True)
            cases[name]()
            if not results[-1]:
                break
    except Exception as exc:
        result(False, str(exc))
    finally:
        # Restore the test bench even if a case or a service assertion fails.
        if rclpy.ok():
            cmd_pub.publish(twist())
            raw_pub.publish(twist())
            for operation in (
                lambda: switch(start_controllers=["joint_state_broadcaster", "drive_controller"], strictness=1),
                lambda: camera_enabled(True),
                *[lambda name=name: delete(name) for name in list(spawned)],
            ):
                try:
                    operation()
                except Exception as exc:
                    print("Cleanup: " + str(exc), file=sys.stderr)
            try:
                drive(twist(), 1.0)
            except Exception:
                pass
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if results and all(results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
