"""Launch the cobot_v2 robot using the shared launch description."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("wonik_cobot_bringup"), "launch", "base.launch.py")),
        launch_arguments={"robot": "cobot_v2"}.items())])
