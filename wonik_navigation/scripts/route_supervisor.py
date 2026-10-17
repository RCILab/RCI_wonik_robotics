#!/usr/bin/env python3
"""Route supervisor: drives a fixed route (waypoints from config/routes/<route>.yaml) through
move_base_flex and switches between following the route and driving around obstacles, by zone or by
topic.

States (published on ~state at 10 Hz; cmd_vel_gate opens its obstacle stop only in AVOID):
  FOLLOW      the route with the route controller (stop_go: stops and turns at waypoints; the gate stops
              the robot for obstacles)
  AVOID_PREP  avoidance wanted (avoid zone or mode avoid) but no detour running yet: a detour to the
              route point just past the zone exit is being planned. The route goal keeps running behind
              the closed gate, so the robot stops in front of an obstacle instead of driving into it.
              Planning failures and timeouts are retried every replan_period; a detour goal that
              move_base_flex rejects or aborts before it runs restarts the route goal (still AVOID_PREP).
  AVOID       a detour goal is confirmed running with the avoid controller (move_base_flex feedback
              from that goal). Replanned every replan_period; a failed replan keeps the current detour.
              At the handover point (halfway along the route tail appended to the detour), or when the
              detour succeeds, the robot rejoins the route in FOLLOW.
  AVOID_HOLD  a running detour ended without success: the robot stands (no goal, gate closed) and the
              detour is replanned every replan_period until one runs (AVOID) or the mode is stop_go.
  DONE        the route goal finished (succeeded or failed); the node exits.
Results and feedback of goals that were replaced are ignored. A zone older than zone_timeout counts as
"none" (no avoidance is started on stale zone data; a confirmed detour is finished).

Parameters
  ~route             route yaml (frame_id, waypoints: [[x, y], ...])
  ~zones             zones yaml (see wonik_navigation/zones.py); optional
  ~controller        route controller (default stop_go)
  ~avoid_controller  detour controller (default avoid)
  ~planner           move_base_flex planner for detours (default global_planner)
  ~replan_period     [s] detour replanning period (default 1.0)
  ~plan_timeout      [s] a detour plan that takes longer is cancelled (default 2.0)
  ~zone_timeout      [s] (default 1.0)
  ~rejoin_margin     [m] the detour reaches the route this far past the avoid zone exit (default 1.0)
  ~rejoin_tail       [m] of route appended to the detour; the route controller takes over halfway
                     along it, once the robot is back on the route line and heading (default 1.5)
Topics
  current_zone       (std_msgs/String, in)  from zone_manager
  ~mode              (std_msgs/String, in)  auto (zones decide, default) | stop_go | avoid
  ~controller        (std_msgs/String, in)  route controller for the rest of the route
  ~active_controller (std_msgs/String, out, latched) controller of the goal move_base_flex runs
  ~state             (std_msgs/String, out) see above, 10 Hz
  ~route_path        (nav_msgs/Path, out, latched)  the whole route, for RViz
"""
import math
import threading

import actionlib
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped
from mbf_msgs.msg import ExePathAction, ExePathGoal, ExePathResult, GetPathAction, GetPathGoal, GetPathResult
from nav_msgs.msg import Path
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

from wonik_navigation.zones import inside, load_zones

SPACING = 0.05  # [m] between route poses
OUTCOMES = {v: k for k, v in vars(ExePathResult).items() if k.isupper() and isinstance(v, int)}


def densify(waypoints, frame_id):
    """Straight lines between the waypoints, one pose every SPACING, oriented along each line."""
    path = Path()
    path.header.frame_id = frame_id

    def pose(x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = frame_id
        p.pose.position.x, p.pose.position.y = x, y
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


class RouteSupervisor:
    def __init__(self):
        with open(rospy.get_param("~route")) as f:
            route = yaml.safe_load(f)
        self.path = densify(route["waypoints"], route.get("frame_id", "map"))
        zones_file = rospy.get_param("~zones", "")
        self.avoid_zones = [z for z in load_zones(zones_file)[1] if z["type"] == "avoid"] if zones_file else []
        self.controller = rospy.get_param("~controller", "stop_go")
        self.avoid_controller = rospy.get_param("~avoid_controller", "avoid")
        self.planner = rospy.get_param("~planner", "global_planner")
        self.replan_period = rospy.get_param("~replan_period", 1.0)
        self.plan_timeout = rospy.get_param("~plan_timeout", 2.0)
        self.zone_timeout = rospy.get_param("~zone_timeout", 1.0)
        self.rejoin_margin = rospy.get_param("~rejoin_margin", 1.0)
        self.rejoin_tail = rospy.get_param("~rejoin_tail", 1.5)

        self.lock = threading.RLock()
        self.state = "FOLLOW"
        self.mode = "auto"
        self.zone = ("none", None)  # (type, receive time)
        self.start_index = 0     # first route pose of the current route goal
        self.rejoin_index = 0    # route index where the current detour reaches the route
        self.handover_index = 0  # route index where the route controller takes over again
        self.last_plan = None
        self.goal_seq = 0        # the goal move_base_flex should be running
        self.goal_kind = None    # "route" | "detour"
        self.goal_controller = None
        self.goal_confirmed = False
        self.done = threading.Event()

        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf)
        self.route_pub = rospy.Publisher("~route_path", Path, queue_size=1, latch=True)
        self.active_pub = rospy.Publisher("~active_controller", String, queue_size=1, latch=True)
        self.state_pub = rospy.Publisher("~state", String, queue_size=1)
        self.exe = actionlib.SimpleActionClient("move_base_flex/exe_path", ExePathAction)
        self.get_path = actionlib.SimpleActionClient("move_base_flex/get_path", GetPathAction)
        rospy.Subscriber("current_zone", String, self.on_zone, queue_size=1)
        rospy.Subscriber("~mode", String, self.on_mode)
        rospy.Subscriber("~controller", String, self.on_controller)
        # heartbeat: the gate trusts AVOID only while this keeps coming
        rospy.Timer(rospy.Duration(0.1), lambda _: self.state_pub.publish(String(data=self.state)))

    # ---- helpers ----
    def robot_xy(self):
        t = self.tf.lookup_transform(self.path.header.frame_id, "base_footprint", rospy.Time(0), rospy.Duration(1.0))
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
        if stamp is None or not 0 <= rospy.get_time() - stamp <= self.zone_timeout:
            return "none"
        return ztype

    def want_avoid(self):
        return self.mode == "avoid" or (self.mode == "auto" and self.current_zone() == "avoid" and bool(self.avoid_zones))

    def send_exe(self, poses, controller, kind):
        goal = ExePathGoal()
        goal.path.header.frame_id = self.path.header.frame_id
        goal.path.header.stamp = rospy.Time.now()
        goal.path.poses = poses
        goal.controller = controller
        goal.concurrency_slot = 1
        self.goal_seq += 1
        self.goal_kind, self.goal_controller, self.goal_confirmed = kind, controller, False
        seq = self.goal_seq
        self.exe.send_goal(goal, done_cb=lambda s, r: self.on_done(seq, s, r),
                           feedback_cb=lambda fb: self.on_feedback(seq, fb))

    def set_state(self, state):
        if state != self.state:
            rospy.loginfo("state %s -> %s", self.state, state)
        self.state = state
        self.state_pub.publish(String(data=state))

    def plan_detour(self):
        """Detour from the robot to the rejoin point: list of poses, or (None, reason)."""
        self.last_plan = rospy.get_time()
        goal = GetPathGoal()
        goal.use_start_pose = False
        goal.target_pose = self.path.poses[self.rejoin_index]
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.tolerance = 0.2
        goal.planner = self.planner
        goal.concurrency_slot = 2
        self.get_path.send_goal(goal)
        if not self.get_path.wait_for_result(rospy.Duration(self.plan_timeout)):
            self.get_path.cancel_goal()  # its late result is dropped with the goal handle
            return None, "planning timed out after %.1f s" % self.plan_timeout
        result = self.get_path.get_result()
        if result is None or result.outcome != GetPathResult.SUCCESS or len(result.path.poses) < 2:
            return None, "planner: %s" % (result.message if result else "no result")
        tail_end = min(self.rejoin_index + int(round(self.rejoin_tail / SPACING)), len(self.path.poses) - 1)
        return result.path.poses + self.path.poses[self.rejoin_index + 1:tail_end + 1], None

    # ---- behaviours ----
    def follow(self, index, state="FOLLOW"):
        """Route goal from index. state AVOID_PREP keeps the gate closed while a detour is pending."""
        self.set_state(state)
        self.start_index = index
        self.send_exe(self.path.poses[index:], self.controller, "route")
        rospy.loginfo("route goal from index %d (%d poses) with '%s'", index, len(self.path.poses) - index,
                      self.controller)

    def start_avoid(self):
        index = self.robot_index(self.start_index)
        last = len(self.path.poses) - 1
        self.rejoin_index = last if self.mode == "avoid" else self.exit_index(index)
        self.handover_index = min(self.rejoin_index + int(round(0.5 * self.rejoin_tail / SPACING)), last)
        rospy.loginfo("avoidance from route index %d, rejoin at index %d, hand over at index %d",
                      index, self.rejoin_index, self.handover_index)
        self.set_state("AVOID_PREP")
        self.try_detour()

    def try_detour(self):
        poses, why = self.plan_detour()
        if poses is None:
            if self.state == "AVOID":
                rospy.logwarn("detour replanning failed (%s): keeping the current detour", why)
            elif self.state == "AVOID_PREP":
                rospy.logwarn("no detour (%s): the route goal continues behind the gate, retrying", why)
            else:
                rospy.logwarn("no detour (%s): holding, retrying", why)
            return
        self.send_exe(poses, self.avoid_controller, "detour")

    def step(self):
        """Called at 10 Hz: zone/mode driven switching and detour replanning."""
        with self.lock:
            if self.done.is_set():
                return
            want = self.want_avoid()
            if self.state == "FOLLOW":
                if want:
                    self.start_avoid()
            elif self.state == "AVOID_PREP":
                if not want:
                    rospy.loginfo("avoidance no longer wanted (zone %s, mode %s)", self.current_zone(), self.mode)
                    if self.goal_kind == "detour":  # replace the pending detour by the route
                        self.follow(self.robot_index(self.start_index))
                    else:
                        self.set_state("FOLLOW")
                elif rospy.get_time() - self.last_plan >= self.replan_period:
                    self.try_detour()
            elif self.state in ("AVOID", "AVOID_HOLD"):
                if self.mode == "stop_go":
                    self.follow(self.robot_index(self.start_index))
                    return
                if self.state == "AVOID":
                    x, y = self.robot_xy()
                    handover = self.path.poses[self.handover_index].pose.position
                    if math.hypot(handover.x - x, handover.y - y) < 0.35:
                        self.follow(self.robot_index(self.rejoin_index))
                        return
                if rospy.get_time() - self.last_plan >= self.replan_period:
                    self.try_detour()

    # ---- callbacks ----
    def on_zone(self, msg):
        self.zone = (msg.data, rospy.get_time())

    def on_mode(self, msg):
        if msg.data not in ("auto", "stop_go", "avoid"):
            rospy.logwarn("unknown mode '%s' (auto | stop_go | avoid)", msg.data)
            return
        with self.lock:
            rospy.loginfo("mode: %s -> %s", self.mode, msg.data)
            self.mode = msg.data

    def on_controller(self, msg):
        with self.lock:
            if self.done.is_set() or msg.data == self.controller:
                return
            rospy.loginfo("route controller '%s' -> '%s'", self.controller, msg.data)
            self.controller = msg.data
            if self.state == "FOLLOW":
                self.follow(self.robot_index(self.start_index))

    def on_feedback(self, seq, _feedback):
        """First feedback of a goal: move_base_flex runs it with its controller."""
        with self.lock:
            if seq != self.goal_seq or self.goal_confirmed:
                return
            self.goal_confirmed = True
            self.active_pub.publish(String(data=self.goal_controller))
            if self.goal_kind == "detour" and self.state in ("AVOID_PREP", "AVOID_HOLD"):
                rospy.loginfo("detour running with '%s'", self.goal_controller)
                self.set_state("AVOID")

    def on_done(self, seq, state, result):
        with self.lock:
            if seq != self.goal_seq:
                return  # result of a replaced goal
            if result is not None and result.outcome == ExePathResult.CANCELED:
                return  # preempted by our own newer goal
            outcome = OUTCOMES.get(result.outcome, result.outcome) if result else "no result (state %d)" % state
            message = result.message if result else ""
            success = result is not None and result.outcome == ExePathResult.SUCCESS
            if self.goal_kind == "detour":
                if success:
                    self.follow(self.robot_index(self.rejoin_index))
                elif self.state == "AVOID":
                    rospy.logwarn("detour ended: %s (%s): holding, replanning", outcome, message)
                    self.set_state("AVOID_HOLD")
                elif self.state == "AVOID_PREP":
                    rospy.logwarn("detour did not start: %s (%s): route goal again, retrying", outcome, message)
                    self.follow(self.robot_index(self.start_index), state="AVOID_PREP")
                else:
                    rospy.logwarn("detour did not start: %s (%s): holding", outcome, message)
                return
            rospy.loginfo("route finished: %s (%s)", outcome, message)
            self.set_state("DONE")
            self.done.set()

    def run(self):
        self.route_pub.publish(self.path)
        rospy.loginfo("waiting for move_base_flex ...")
        self.exe.wait_for_server()
        if self.avoid_zones:
            self.get_path.wait_for_server()
        with self.lock:
            self.follow(0)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.done.is_set():
            try:
                self.step()
            except tf2_ros.TransformException as e:
                rospy.logwarn_throttle(5.0, "no robot pose: %s", e)
            try:
                rate.sleep()
            except rospy.ROSTimeMovedBackwardsException:
                pass


if __name__ == "__main__":
    rospy.init_node("route_supervisor")
    RouteSupervisor().run()
