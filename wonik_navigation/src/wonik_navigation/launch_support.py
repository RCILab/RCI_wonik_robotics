"""Shared launch construction; configuration files remain in the package share directory."""
import os
import atexit
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def share():
    return get_package_share_directory("wonik_navigation")

def config(relative):
    with open(os.path.join(share(), relative)) as f:
        return yaml.safe_load(f)

def scan_node(robot, sim):
    return Node(package="pointcloud_to_laserscan", executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan", parameters=[config("config/" + robot + "/pointcloud_to_laserscan.yaml"),
                                                  {"use_sim_time": sim}],
        remappings=[("cloud_in", "/points"), ("scan", "/scan")])

def rviz_node(sim):
    return Node(package="rviz2", executable="rviz2", parameters=[{"use_sim_time": sim}],
                arguments=["-d", os.path.join(share(), "rviz", "navigation.rviz")])

def localization_nodes(robot, map_file, sim, x, y, yaw):
    amcl = config("config/" + robot + "/amcl.yaml")["amcl"]["ros__parameters"]
    amcl.update(use_sim_time=sim, initial_pose={"x": x, "y": y, "z": 0.0, "yaw": yaw})
    return [scan_node(robot, sim),
        Node(package="nav2_map_server", executable="map_server", name="map_server",
             parameters=[{"use_sim_time": sim, "yaml_filename": map_file}], output="screen"),
        Node(package="nav2_amcl", executable="amcl", name="amcl", parameters=[amcl], output="screen"),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_localization",
             parameters=[{"use_sim_time": sim, "autostart": True, "node_names": ["map_server", "amcl"]}])]

def navigation_params(sim, depth):
    data = config("config/navigation.yaml")
    for name in ("controller_server", "planner_server"):
        data[name]["ros__parameters"]["use_sim_time"] = sim
    for name in ("local_costmap", "global_costmap"):
        p = data[name][name]["ros__parameters"]
        p["use_sim_time"] = sim
        if not depth:
            p["obstacle_layer"]["observation_sources"] = "scan"
    handle = tempfile.NamedTemporaryFile(prefix="wonik_nav2_", suffix=".yaml", mode="w", delete=False)
    yaml.safe_dump(data, handle, sort_keys=False)
    handle.close()
    atexit.register(lambda: os.path.exists(handle.name) and os.unlink(handle.name))
    return handle.name
