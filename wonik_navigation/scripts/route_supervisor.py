#!/usr/bin/env python3
"""Route/zone supervisor using Nav2 FollowPath and ComputePathToPose.

FOLLOW -> AVOID_PREP -> AVOID -> FOLLOW, with AVOID_HOLD on a failed running detour.
Only feedback from the current accepted detour opens the gate. Planning and goal
acceptance are asynchronous: deadlines and safety heartbeats keep running.
"""
import math
import yaml
import rclpy
from rclpy.time import Time
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
import tf2_ros
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import FollowPath, ComputePathToPose
from std_msgs.msg import String
from tf_transformations import quaternion_from_euler
from wonik_navigation.ros_node import ManagedNode, latched_qos, spin_node
from wonik_navigation.zones import inside, load_zones

SPACING = 0.05

def densify(waypoints, frame_id):
    """Straight lines between the waypoints, one pose every SPACING, oriented along each line."""
    path = Path()
    path.header.frame_id = frame_id

    def pose(x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = frame_id
        p.pose.position.x, p.pose.position.y = float(x), float(y)
        (p.pose.orientation.x, p.pose.orientation.y,
         p.pose.orientation.z, p.pose.orientation.w) = quaternion_from_euler(0, 0, yaw)
        return p

    yaw = 0.0
    for (x0, y0), (x1, y1) in zip(waypoints, waypoints[1:]):
        length = math.hypot(x1 - x0, y1 - y0)
        yaw = math.atan2(y1 - y0, x1 - x0)
        n = max(1, int(math.ceil(length / SPACING)))
        path.poses += [pose(x0 + (x1 - x0) * k / n, y0 + (y1 - y0) * k / n, yaw) for k in range(n)]
    path.poses.append(pose(waypoints[-1][0], waypoints[-1][1], yaw))
    return path


class RouteSupervisor(ManagedNode):
    def __init__(self):
        super().__init__("route_supervisor")
        with open(self.param("route", "")) as f:
            route = yaml.safe_load(f)
        self.path = densify(route["waypoints"], route.get("frame_id", "map"))
        zones_file = self.param("zones", "")
        self.avoid_zones = [z for z in load_zones(zones_file)[1] if z["type"] == "avoid"] if zones_file else []
        for key, value in dict(controller="stop_go", avoid_controller="avoid", planner="global_planner",
                               replan_period=1.0, plan_timeout=2.0, zone_timeout=1.0,
                               rejoin_margin=1.0, rejoin_tail=1.5).items():
            setattr(self, key, self.param(key, value))
        self.state, self.mode = "FOLLOW", "auto"
        self.zone = ("none", None)
        self.start_index = self.rejoin_index = self.handover_index = 0
        self.last_plan = -float("inf")
        self.goal_seq = self.plan_seq = 0
        self.goal_kind = self.goal_controller = None
        self.goal_confirmed = False
        self.goal_handle = self.plan_handle = None
        self.plan_pending = False
        self.started = self.done = False
        self.tf = tf2_ros.Buffer(node=self)
        self.tf_listener = tf2_ros.TransformListener(self.tf, self)
        self.route_pub = self.create_publisher(Path, "~/route_path", latched_qos())
        self.active_pub = self.create_publisher(String, "~/active_controller", latched_qos())
        self.state_pub = self.create_publisher(String, "~/state", 1)
        self.exe = ActionClient(self, FollowPath, "follow_path")
        self.get_path = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.create_subscription(String, "current_zone", self.on_zone, 1)
        self.create_subscription(String, "~/mode", self.on_mode, 1)
        self.create_subscription(String, "~/controller", self.on_controller, 1)
        self.route_pub.publish(self.path)
        self.create_timer(0.1, self.tick)

    def robot_xy(self):
        t = self.tf.lookup_transform(self.path.header.frame_id, "base_footprint", Time())
        return t.transform.translation.x, t.transform.translation.y

    def robot_index(self, from_index):
        """Index of the route pose closest to the robot, searched forward from from_index."""
        x, y = self.robot_xy()
        poses = self.path.poses
        return min(range(from_index, len(poses)),
                   key=lambda i: (poses[i].pose.position.x - x) ** 2 + (poses[i].pose.position.y - y) ** 2)

    def exit_index(self, index):
        """First route index after the avoid zone around route index 'index', plus rejoin_margin."""
        poses = self.path.poses
        zone = next((z for z in self.avoid_zones
                     if inside(poses[index].pose.position.x, poses[index].pose.position.y, z["polygon"])), None)
        i = index
        if zone is not None:
            while i < len(poses) - 1 and inside(poses[i].pose.position.x, poses[i].pose.position.y, zone["polygon"]):
                i += 1
        return min(i + int(round(self.rejoin_margin / SPACING)), len(poses) - 1)

    def current_zone(self):
        ztype, stamp = self.zone
        if stamp is None or not 0 <= self.seconds() - stamp <= self.zone_timeout:
            return "none"
        return ztype

    def want_avoid(self):
        return self.mode == "avoid" or (self.mode == "auto" and self.current_zone() == "avoid" and bool(self.avoid_zones))

    def set_state(self, state):
        if state != self.state:
            self.info("state %s -> %s", self.state, state)
        self.state = state
        self.state_pub.publish(String(data=state))

    def cancel_plan(self):
        self.plan_seq += 1
        self.plan_pending = False
        if self.plan_handle is not None and rclpy.ok():
            self.plan_handle.cancel_goal_async()
        self.plan_handle = None

    def send_exe(self, poses, controller, kind):
        self.goal_seq += 1
        seq = self.goal_seq
        self.goal_kind, self.goal_controller, self.goal_confirmed = kind, controller, False
        goal = FollowPath.Goal()
        goal.path.header.frame_id = self.path.header.frame_id
        goal.path.header.stamp = self.get_clock().now().to_msg()
        goal.path.poses = poses
        goal.controller_id = controller
        goal.goal_checker_id = "avoid_goal" if kind == "detour" else "route_goal"
        request = self.exe.send_goal_async(goal, feedback_callback=lambda fb: self.on_feedback(seq, fb))
        request.add_done_callback(lambda future: self.on_accepted(seq, future))

    def on_accepted(self, seq, future):
        try:
            handle = future.result()
        except Exception as exc:
            if seq == self.goal_seq:
                self.on_done(seq, GoalStatus.STATUS_ABORTED, str(exc))
            return
        if seq != self.goal_seq or self.done:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.on_done(seq, GoalStatus.STATUS_ABORTED, "goal rejected")
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_result(seq, f))

    def on_result(self, seq, future):
        try:
            wrapped = future.result()
            self.on_done(seq, wrapped.status, "")
        except Exception as exc:
            self.on_done(seq, GoalStatus.STATUS_ABORTED, str(exc))

    def follow(self, index, state="FOLLOW"):
        self.cancel_plan()
        self.set_state(state)
        self.start_index = index
        self.send_exe(self.path.poses[index:], self.controller, "route")
        self.info("route from index %d with %s", index, self.controller)

    def start_avoid(self):
        index = self.robot_index(self.start_index)
        last = len(self.path.poses) - 1
        self.rejoin_index = last if self.mode == "avoid" else self.exit_index(index)
        self.handover_index = min(self.rejoin_index + int(round(0.5 * self.rejoin_tail / SPACING)), last)
        self.set_state("AVOID_PREP")
        self.try_detour()

    def try_detour(self):
        if self.plan_pending or not self.get_path.server_is_ready():
            return
        self.last_plan = self.seconds()
        self.plan_pending = True
        self.plan_seq += 1
        seq = self.plan_seq
        goal = ComputePathToPose.Goal()
        goal.use_start = False
        goal.goal = self.path.poses[self.rejoin_index]
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.planner_id = self.planner
        self.get_path.send_goal_async(goal).add_done_callback(lambda f: self.on_plan_accepted(seq, f))

    def on_plan_accepted(self, seq, future):
        try:
            handle = future.result()
        except Exception as exc:
            if seq == self.plan_seq:
                self.plan_pending = False
                self.warn("planning request failed: %s", exc)
            return
        if seq != self.plan_seq:
            if handle.accepted:
                handle.cancel_goal_async()
            return
        if not handle.accepted:
            self.plan_pending = False
            self.warn("planning goal rejected")
            return
        self.plan_handle = handle
        handle.get_result_async().add_done_callback(lambda f: self.on_plan_result(seq, f))

    def on_plan_result(self, seq, future):
        if seq != self.plan_seq:
            return
        self.plan_pending = False
        self.plan_handle = None
        try:
            wrapped = future.result()
            poses = wrapped.result.path.poses
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED or len(poses) < 2:
                self.warn("no detour: retaining the current route/hold")
                return
        except Exception as exc:
            self.warn("planning result failed: %s", exc)
            return
        if self.state not in ("AVOID_PREP", "AVOID", "AVOID_HOLD"):
            return
        end = min(self.rejoin_index + int(round(self.rejoin_tail / SPACING)), len(self.path.poses) - 1)
        self.send_exe(poses + self.path.poses[self.rejoin_index + 1:end + 1], self.avoid_controller, "detour")

    def tick(self):
        self.state_pub.publish(String(data=self.state))
        if self.done:
            return
        try:
            if not self.started:
                if not self.exe.server_is_ready():
                    return
                self.robot_xy()
                self.started = True
                self.follow(0)
            if self.plan_pending and self.seconds() - self.last_plan > self.plan_timeout:
                self.cancel_plan()
                self.warn("detour planning timed out; retrying behind the gate")
            want = self.want_avoid()
            if self.state == "FOLLOW" and want:
                self.start_avoid()
            elif self.state == "AVOID_PREP":
                if not want:
                    self.cancel_plan()
                    if self.goal_kind == "detour":
                        self.follow(self.robot_index(self.start_index))
                    else:
                        self.set_state("FOLLOW")
                elif self.seconds() - self.last_plan >= self.replan_period:
                    self.try_detour()
            elif self.state in ("AVOID", "AVOID_HOLD"):
                if self.mode == "stop_go":
                    self.follow(self.robot_index(self.start_index))
                    return
                if self.state == "AVOID":
                    x, y = self.robot_xy()
                    p = self.path.poses[self.handover_index].pose.position
                    if math.hypot(p.x - x, p.y - y) < 0.35:
                        self.follow(self.robot_index(self.rejoin_index))
                        return
                if self.seconds() - self.last_plan >= self.replan_period:
                    self.try_detour()
        except tf2_ros.TransformException as exc:
            self.warn_throttle(5.0, "no robot pose: %s", exc)

    def on_zone(self, msg):
        self.zone = (msg.data, self.seconds())

    def on_mode(self, msg):
        if msg.data in ("auto", "stop_go", "avoid"):
            self.info("mode: %s -> %s", self.mode, msg.data)
            self.mode = msg.data
        else:
            self.warn("unknown mode: %s", msg.data)

    def on_controller(self, msg):
        if not self.done and msg.data != self.controller:
            self.controller = msg.data
            if self.started and self.state == "FOLLOW":
                try:
                    self.follow(self.robot_index(self.start_index))
                except tf2_ros.TransformException as exc:
                    self.warn("cannot switch controller: %s", exc)

    def on_feedback(self, seq, _feedback):
        if seq != self.goal_seq or self.goal_confirmed:
            return
        self.goal_confirmed = True
        self.active_pub.publish(String(data=self.goal_controller))
        if self.goal_kind == "detour" and self.state in ("AVOID_PREP", "AVOID_HOLD"):
            self.set_state("AVOID")

    def on_done(self, seq, status, message):
        if seq != self.goal_seq or self.done:
            return
        self.goal_handle = None
        success = status == GoalStatus.STATUS_SUCCEEDED
        if self.goal_kind == "detour":
            try:
                if success:
                    self.follow(self.robot_index(self.rejoin_index))
                elif self.state == "AVOID_PREP":
                    self.follow(self.robot_index(self.start_index), "AVOID_PREP")
                else:
                    self.set_state("AVOID_HOLD")
            except tf2_ros.TransformException:
                self.set_state("AVOID_HOLD")
            return
        self.info("route finished: status %d %s", status, message)
        self.cancel_plan()
        self.set_state("DONE")
        self.done = True

    def destroy_node(self):
        self.done = True
        self.goal_seq += 1
        self.cancel_plan()
        if self.goal_handle is not None and rclpy.ok():
            self.goal_handle.cancel_goal_async()
        return super().destroy_node()

if __name__ == "__main__":
    spin_node(RouteSupervisor)
