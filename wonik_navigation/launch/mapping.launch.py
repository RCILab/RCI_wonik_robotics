import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from wonik_navigation.launch_support import share, scan_node, rviz_node

def setup(context):
    sim = LaunchConfiguration("use_sim_time").perform(context)
    robot = LaunchConfiguration("robot").perform(context)
    nodes = [scan_node(robot, sim.lower() == "true"), IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("slam_toolbox"), "launch", "online_async_launch.py")),
        launch_arguments={"use_sim_time": sim, "slam_params_file": os.path.join(share(), "config", "slam_toolbox_mapping.yaml")}.items())]
    if LaunchConfiguration("rviz").perform(context).lower() == "true":
        nodes.append(rviz_node(sim.lower() == "true"))
    return nodes

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("robot", default_value="cobot_v2"),
        DeclareLaunchArgument("use_sim_time", default_value="true"), DeclareLaunchArgument("rviz", default_value="true"),
        OpaqueFunction(function=setup)])
