#!/usr/bin/env python3
"""Route supervisor: sends a fixed route (waypoints from config/routes/<route>.yaml) to move_base_flex
as one ExePath goal, and switches the controller mid-route on request by resending the rest of the
route with the new controller name.

Parameters
  ~route        route yaml (frame_id, waypoints: [[x, y], ...])
  ~controller   move_base_flex controller for the route (default stop_go)
Topics
  ~controller        (std_msgs/String, in)   switch to this controller for the rest of the route
  ~active_controller (std_msgs/String, out, latched)
  ~route_path        (nav_msgs/Path, out, latched)  the whole route, for RViz
"""
import math
import threading

import actionlib
import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped
from mbf_msgs.msg import ExePathAction, ExePathGoal, ExePathResult
from nav_msgs.msg import Path
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler

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
        self.controller = rospy.get_param("~controller", "stop_go")
        self.lock = threading.Lock()
        self.start_index = 0
        self.done = threading.Event()

        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf)
        self.route_pub = rospy.Publisher("~route_path", Path, queue_size=1, latch=True)
        self.active_pub = rospy.Publisher("~active_controller", String, queue_size=1, latch=True)
        self.client = actionlib.SimpleActionClient("move_base_flex/exe_path", ExePathAction)
        rospy.Subscriber("~controller", String, self.on_controller)

    def robot_index(self):
        """Index of the route pose closest to the robot, searched forward from the current start."""
        t = self.tf.lookup_transform(self.path.header.frame_id, "base_footprint", rospy.Time(0), rospy.Duration(1.0))
        x, y = t.transform.translation.x, t.transform.translation.y
        poses = self.path.poses
        return min(range(self.start_index, len(poses)),
                   key=lambda i: (poses[i].pose.position.x - x) ** 2 + (poses[i].pose.position.y - y) ** 2)

    def send(self, start_index, controller):
        goal = ExePathGoal()
        goal.path.header.frame_id = self.path.header.frame_id
        goal.path.header.stamp = rospy.Time.now()
        goal.path.poses = self.path.poses[start_index:]
        goal.controller = controller
        goal.concurrency_slot = 1
        self.start_index, self.controller = start_index, controller
        self.client.send_goal(goal, done_cb=self.on_done)
        self.active_pub.publish(String(data=controller))
        rospy.loginfo("route: %d poses from index %d with controller '%s'",
                      len(goal.path.poses), start_index, controller)

    def on_controller(self, msg):
        with self.lock:
            if self.done.is_set() or msg.data == self.controller:
                return
            try:
                index = self.robot_index()
            except tf2_ros.TransformException as e:
                rospy.logwarn("cannot switch controller, no robot pose: %s", e)
                return
            rospy.loginfo("switching controller '%s' -> '%s' at route index %d", self.controller, msg.data, index)
            self.send(index, msg.data)

    def on_done(self, state, result):
        if result is not None and result.outcome == ExePathResult.CANCELED:
            return  # replaced by a newer goal
        rospy.loginfo("route finished: %s (%s)", OUTCOMES.get(result.outcome, result.outcome) if result else state,
                      result.message if result else "")
        self.done.set()

    def run(self):
        self.route_pub.publish(self.path)
        rospy.loginfo("waiting for move_base_flex ...")
        self.client.wait_for_server()
        with self.lock:
            self.send(0, self.controller)
        while not rospy.is_shutdown() and not self.done.wait(0.5):
            pass


if __name__ == "__main__":
    rospy.init_node("route_supervisor")
    RouteSupervisor().run()
