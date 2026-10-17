from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from wonik_navigation.launch_support import scan_node

def setup(context):
    return [scan_node(LaunchConfiguration("robot").perform(context),
                      LaunchConfiguration("use_sim_time").perform(context).lower() == "true")]

def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument("robot", default_value="cobot_v2"),
        DeclareLaunchArgument("use_sim_time", default_value="true"), OpaqueFunction(function=setup)])
