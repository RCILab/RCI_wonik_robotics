import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from wonik_navigation.launch_support import share, localization_nodes, rviz_node

def setup(context):
    val = lambda key: LaunchConfiguration(key).perform(context)
    sim = val("use_sim_time").lower() == "true"
    map_file = val("map_file") or os.path.join(share(), "maps", val("map") + ".yaml")
    nodes = localization_nodes(val("robot"), map_file, sim, float(val("x")), float(val("y")), float(val("yaw")))
    return nodes + ([rviz_node(sim)] if val("rviz").lower() == "true" else [])

def generate_launch_description():
    args = dict(robot="cobot_v2", map="wonik_hall", map_file="", use_sim_time="true", rviz="true",
                x="0.0", y="0.0", yaw="0.0")
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in args.items()] + [OpaqueFunction(function=setup)])
