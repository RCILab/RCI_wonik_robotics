import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from wonik_navigation.launch_support import share

def setup(context):
    val = lambda key: LaunchConfiguration(key).perform(context)
    return [Node(package="wonik_navigation", executable="route_supervisor.py", name="route_supervisor",
        parameters=[{"use_sim_time": val("use_sim_time").lower() == "true",
                     "route": val("route_file") or os.path.join(share(), "config", "routes", val("route") + ".yaml"),
                     "zones": os.path.join(share(), "config", "zones", val("zones") + ".yaml"),
                     "controller": val("controller")}], output="screen")]

def generate_launch_description():
    args = dict(route="wonik_hall_u", route_file="", zones="wonik_hall", controller="stop_go", use_sim_time="true")
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in args.items()] + [OpaqueFunction(function=setup)])
