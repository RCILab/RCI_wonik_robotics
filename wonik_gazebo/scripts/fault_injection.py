#!/usr/bin/env python3
"""Fault injection on cobot_v2 in Gazebo (kinematic mode), one case per run:
    roslaunch wonik_gazebo fault_injection.launch          # empty world, detector, gate, camera relay
    rosrun wonik_gazebo fault_injection.py <case>          # another terminal
  b1  joint_states stop while driving         -> zero wheel speed within joint_state_timeout, resumes
                                                 through the measured start sequence
  b2  cmd_vel stops while driving             -> stop after cmd_timeout and braking
  c   drive controller not responding         -> no DRIVE from converged commands, FAULT latched until
                                                 ~clear_fault
  h   bad joint_states injected while driving -> NaN, future, late (50 ms, 2 s), duplicate stamps and
                                                 missing joints rejected; odometry and commands unaffected
  d   box beside the robot, turn in place through the gate -> turn stopped, driving along it allowed
  g   low box only the depth camera sees, then the camera stream is lost (relay killed) -> stays stopped
Each case prints its measurements and PASS/FAIL. The depth camera needs an X display (Xvfb works).
"""
import math
import re
import subprocess
import sys
import threading
import time

import rospy
from controller_manager_msgs.srv import SwitchController
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger
from tf.transformations import euler_from_quaternion

S = {"gt": None, "speed": 0.0, "mode": "", "fault": "", "wheel": None, "odom": None, "js": None, "reason": "",
     "sensors_ok": None, "detected": None, "near": None, "drive_cmd": [], "bad": []}
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


rospy.init_node("fault_injection", anonymous=True)
rospy.Subscriber("/ground_truth/odom", Odometry, on_gt)
rospy.Subscriber("/kinematics_node/debug_state", String, on_dbg)
rospy.Subscriber("/kinematics_node/fault", String, lambda m: S.__setitem__("fault", m.data))
rospy.Subscriber("/wheel/odom", Odometry, on_odom("wheel"))
rospy.Subscriber("/odom", Odometry, on_odom("odom"))
rospy.Subscriber("/drive_controller/command", Float64MultiArray, on_drive_cmd)
rospy.Subscriber("/cmd_vel_gate/reason", String, lambda m: S.__setitem__("reason", m.data))
rospy.Subscriber("/obstacle_detector/sensors_ok", Bool, lambda m: S.__setitem__("sensors_ok", m.data))
rospy.Subscriber("/obstacle_detected", Bool, lambda m: S.__setitem__("detected", m.data))
rospy.Subscriber("/obstacle_near", Bool, lambda m: S.__setitem__("near", m.data))
cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
raw_pub = rospy.Publisher("/cmd_vel_raw", Twist, queue_size=1)
switch = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
while not rospy.is_shutdown() and S["gt"] is None:
    rospy.sleep(0.1)


def twist(vx=0.0, vy=0.0, wz=0.0):
    t = Twist()
    t.linear.x, t.linear.y, t.angular.z = vx, vy, wz
    return t


def drive(cmd, seconds, pub=None, watch=None):
    """Publish cmd at 20 Hz for seconds; call watch(t) each tick; return modes seen."""
    pub = pub or cmd_pub
    modes = []
    t0 = rospy.get_time()
    rate = rospy.Rate(20)
    while not rospy.is_shutdown() and rospy.get_time() - t0 < seconds:
        if cmd is not None:
            pub.publish(cmd)
        if not modes or modes[-1] != S["mode"]:
            modes.append(S["mode"])
        if watch:
            watch(rospy.get_time() - t0)
        rate.sleep()
    return modes


def result(ok, text):
    print("%s: %s" % ("PASS" if ok else "FAIL", text), flush=True)


def case_b1():
    drive(twist(0.5), 4.0)
    v0 = S["speed"]
    switch(start_controllers=[], stop_controllers=["joint_state_controller"], strictness=2)
    stop_at = []
    modes = drive(twist(0.5), 3.0, watch=lambda t: stop_at.append(t) if S["speed"] < 0.02 and not stop_at else None)
    switch(start_controllers=["joint_state_controller"], stop_controllers=[], strictness=2)
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
    cleared = rospy.ServiceProxy("/kinematics_node/clear_fault", Trigger)()
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
            real["m"], real["t"] = m, rospy.get_time()
    rospy.Subscriber("/joint_states", JointState, on_real)
    fake = rospy.Publisher("/joint_states", JointState, queue_size=10)
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
            f.header.stamp = rospy.Time.now()
            f.velocity[0] = float("nan")
        elif kind == "future":
            f.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
        elif kind == "old":
            f.header.stamp = m.header.stamp - rospy.Duration(0.05)
        elif kind == "very_old":
            f.header.stamp = m.header.stamp - rospy.Duration(2.0)  # 2 s late, not a clock reset
        elif kind == "duplicate":
            time.sleep(0.005)  # let the kinematics node process the real sample first
            m = real["m"]
            f.header.stamp = m.header.stamp  # same stamp as an already processed real sample
        else:
            f.header.stamp = rospy.Time.now()
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
    rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)(name, sdf, "", p, "world")


def delete(name):
    rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)(name)


def case_d():
    x, y, yaw = S["gt"]
    # box along the left side, 0.15 m from the body (inside the 0.57 m corner circle)
    spawn("side_box", x - 0.70 * math.sin(yaw), y + 0.70 * math.cos(yaw), 2.0, 0.4, 1.0, yaw)
    rospy.sleep(2.0)
    y0 = S["gt"][2]
    reasons = []
    drive(twist(wz=0.4), 6.0, pub=raw_pub, watch=lambda t: reasons.append(S["reason"]) if S["reason"] not in reasons else None)
    turned_blocked = math.degrees(abs(math.remainder(S["gt"][2] - y0, 2 * math.pi)))
    near = S["near"]
    p0 = S["gt"]
    drive(twist(0.3), 6.0, pub=raw_pub)  # includes re-aligning the wheels if they point elsewhere
    along = math.hypot(S["gt"][0] - p0[0], S["gt"][1] - p0[1])
    drive(twist(), 1.0, pub=raw_pub)
    delete("side_box")
    rospy.sleep(2.0)
    y1 = S["gt"][2]
    drive(twist(wz=0.4), 8.0, pub=raw_pub)
    turned_free = math.degrees(abs(math.remainder(S["gt"][2] - y1, 2 * math.pi)))
    drive(twist(), 1.0, pub=raw_pub)
    print("box beside: obstacle_near %s, gate %s, turned %.1f deg; drove along it %.2f m; box removed: turned %.1f deg"
          % (near, [r for r in reasons if r], turned_blocked, along, turned_free))
    result(near and turned_blocked < 1.0 and along > 0.3 and turned_free > 30.0,
           "turning in place stopped next to the box, driving along it and turning without it allowed")


def case_g():
    x, y, yaw = S["gt"]
    spawn("low_box", x + 1.1 * math.cos(yaw), y + 1.1 * math.sin(yaw), 0.6, 0.6, 0.12, yaw)
    rospy.sleep(3.0)
    seen = (S["detected"], S["sensors_ok"])
    p0 = S["gt"]
    drive(twist(0.3), 2.0, pub=raw_pub)
    reason1 = S["reason"]
    subprocess.call(["rosnode", "kill", "/camera_relay"])
    rospy.sleep(1.5)
    lost = (S["detected"], S["sensors_ok"])
    drive(twist(0.3), 3.0, pub=raw_pub)
    reason2 = S["reason"]
    moved = math.hypot(S["gt"][0] - p0[0], S["gt"][1] - p0[1])
    delete("low_box")
    print("camera sees the low box: detected/sensors_ok %s, gate '%s'; camera stream lost: detected/sensors_ok %s, "
          "gate '%s'; robot moved %.3f m" % (seen, reason1, lost, reason2, moved))
    result(seen == (True, True) and lost == (True, False) and moved < 0.02,
           "the LiDAR alone (clear) does not release the obstacle only the camera saw")


{"b1": case_b1, "b2": case_b2, "c": case_c, "h": case_h, "d": case_d, "g": case_g}[sys.argv[1]]()
