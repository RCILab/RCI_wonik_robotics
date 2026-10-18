"""Empty-world test bench with a switchable camera stream; no navigation controller."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from wonik_navigation.launch_support import obstacle_params, self_filter_node


def generate_launch_description():
    gazebo = get_package_share_directory("wonik_gazebo")
    obstacle = obstacle_params("cobot_v2", True, camera_topic="/camera_relay/points")
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="false"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(gazebo, "launch", "cobot_v2.launch.py")),
            launch_arguments={"gui": LaunchConfiguration("gui"), "rviz": "false"}.items()),
        self_filter_node("cobot_v2", True),
        Node(package="wonik_gazebo", executable="camera_relay.py", name="camera_relay",
             parameters=[{"use_sim_time": True}]),
        Node(package="wonik_navigation", executable="obstacle_detector.py", name="obstacle_detector",
             parameters=[obstacle], output="screen"),
        Node(package="wonik_navigation", executable="cmd_vel_gate.py", name="cmd_vel_gate",
             parameters=[{"use_sim_time": True}], output="screen"),
    ])
