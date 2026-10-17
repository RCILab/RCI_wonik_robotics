import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from wonik_navigation.launch_support import share, config, localization_nodes, navigation_params, rviz_node

def setup(context):
    val = lambda key: LaunchConfiguration(key).perform(context)
    sim, depth = val("use_sim_time").lower() == "true", val("depth_camera").lower() == "true"
    map_file = val("map_file") or os.path.join(share(), "maps", val("map") + ".yaml")
    actions = localization_nodes(val("robot"), map_file, sim, float(val("x")), float(val("y")), float(val("yaw")))
    params = navigation_params(sim, depth)
    actions += [
        Node(package="nav2_controller", executable="controller_server", name="controller_server",
             parameters=[params], remappings=[("cmd_vel", "cmd_vel_raw")], output="screen"),
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             parameters=[params], output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             parameters=[{"use_sim_time": sim, "autostart": True, "node_names": ["controller_server", "planner_server"]}])]
    obstacle = config("config/" + val("robot") + "/obstacle_detector.yaml")
    # launch_ros cannot infer the type of empty arrays; the node defaults to [].
    obstacle = {key: value for key, value in obstacle.items() if value != []}
    if not depth:
        obstacle["required_topics"] = ["/points"]
    actions += [
        Node(package="wonik_navigation", executable="obstacle_detector.py", name="obstacle_detector",
             parameters=[obstacle, {"use_sim_time": sim}], output="screen"),
        Node(package="wonik_navigation", executable="zone_manager.py", name="zone_manager",
             parameters=[{"use_sim_time": sim, "zones": os.path.join(share(), "config", "zones", val("zones") + ".yaml")}]),
        Node(package="wonik_navigation", executable="cmd_vel_gate.py", name="cmd_vel_gate",
             parameters=[{"use_sim_time": sim}], output="screen")]
    if val("rviz").lower() == "true":
        actions.append(rviz_node(sim))
    return actions

def generate_launch_description():
    args = dict(robot="cobot_v2", map="wonik_hall", map_file="", zones="wonik_hall", use_sim_time="true",
                depth_camera="true", rviz="true", x="0.0", y="0.0", yaw="0.0")
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in args.items()] +
                             [OpaqueFunction(function=setup)])
