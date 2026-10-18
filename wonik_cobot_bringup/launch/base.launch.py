import os
import re
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def setup(context):
    bringup = get_package_share_directory("wonik_cobot_bringup")
    desc = get_package_share_directory("wonik_cobot_description")
    val = lambda key: LaunchConfiguration(key).perform(context)
    sim = val("use_sim_time").lower() == "true"
    ekf = val("ekf").lower() == "true"
    robot_name = val("robot")
    geometry = val("geometry") or os.path.join(desc, "config", robot_name, "geometry.yaml")
    with open(geometry) as f:
        g = yaml.safe_load(f)
    with open(os.path.join(bringup, "config", robot_name, "kinematics.yaml")) as f:
        kin = yaml.safe_load(f)
    keys = ("drive_type", "module_x", "module_y", "wheel_diameter", "steer_joints", "drive_joints", "fixed_axle_x")
    kin.update({k: g[k] for k in keys if k in g})
    kin.update(use_sim_time=sim, publish_tf=not ekf)
    robot = xacro.process_file(os.path.join(desc, "urdf", robot_name + ".urdf.xacro"),
        mappings={"geometry": geometry, **{key: val(key) for key in ("sim_drive", "lidar_gpu", "depth_camera", "hardware_plugin")}}).toxml()
    # gazebo_ros2_control forwards this string through the ROS parameter parser.
    # XML comments containing ': ' are otherwise interpreted as YAML mappings.
    robot = re.sub(r"<!--.*?-->", "", robot, flags=re.DOTALL)
    nodes = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"use_sim_time": sim, "robot_description": ParameterValue(robot, value_type=str)}]),
        Node(package="qd_kinematics", executable="kinematics_node", name="kinematics_node",
             parameters=[kin], remappings=[("odom", "wheel/odom")] if ekf else [], output="screen")]
    if ekf:
        nodes.append(Node(package="robot_localization", executable="ekf_node", name="ekf_odom",
            parameters=[os.path.join(bringup, "config", robot_name, "ekf.yaml"), {"use_sim_time": sim}],
            remappings=[("odometry/filtered", "odom")], output="screen"))
    return nodes

def generate_launch_description():
    desc = get_package_share_directory("wonik_cobot_description")
    args = dict(robot="cobot_v2", use_sim_time="false", ekf="true", sim_drive="kinematic", lidar_gpu="false",
                depth_camera="true", hardware_plugin="gazebo_ros2_control/GazeboSystem",
                geometry="")
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k, v in args.items()] +
                             [OpaqueFunction(function=setup)])
