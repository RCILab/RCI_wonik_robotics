"""Empty-world test bench with a switchable camera stream; no navigation controller."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from wonik_navigation.launch_support import config


def generate_launch_description():
    gazebo = get_package_share_directory("wonik_gazebo")
    obstacle = config("config/cobot_v2/obstacle_detector.yaml")
    obstacle = {key: value for key, value in obstacle.items() if value != []}
    obstacle["required_topics"] = ["/points", "/camera_relay/points"]
    obstacle["use_sim_time"] = True
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(gazebo, "launch", "cobot_v2.launch.py")),
            launch_arguments={"gui": LaunchConfiguration("gui"), "rviz": "false"}.items()),
        Node(package="wonik_gazebo", executable="camera_relay.py", name="camera_relay",
             parameters=[{"use_sim_time": True}]),
        Node(package="wonik_navigation", executable="obstacle_detector.py", name="obstacle_detector",
             parameters=[obstacle], output="screen"),
        Node(package="wonik_navigation", executable="cmd_vel_gate.py", name="cmd_vel_gate",
             parameters=[{"use_sim_time": True}], output="screen"),
    ])
