import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def setup(context):
    val = lambda key: LaunchConfiguration(key).perform(context)
    pkg = get_package_share_directory("wonik_gazebo")
    bringup = get_package_share_directory("wonik_cobot_v2_bringup")
    world = val("world_file") or os.path.join(pkg, "worlds", val("world") + ".world")
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")),
        launch_arguments={"world": world, "gui": val("gui"), "verbose": "false",
                          "params_file": os.path.join(pkg, "config", "gazebo.yaml")}.items())
    base = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "base.launch.py")),
        launch_arguments={**{k: val(k) for k in ("sim_drive", "lidar_gpu", "depth_camera", "ekf")},
                          "use_sim_time": "true"}.items())
    spawn = Node(package="gazebo_ros", executable="spawn_entity.py", output="screen",
        arguments=["-topic", "robot_description", "-entity", "wonik_cobot_v2",
                   "-x", val("x"), "-y", val("y"), "-z", "0.02", "-Y", val("yaw"), "-timeout", "120"])
    controllers = Node(package="controller_manager", executable="spawner", output="screen",
        arguments=["joint_state_broadcaster", "steer_controller", "drive_controller",
                   "--controller-manager-timeout", "120"])
    paths = os.path.join(get_package_prefix("wonik_gazebo"), "lib") + os.pathsep + os.environ.get("GAZEBO_PLUGIN_PATH", "")
    models = os.path.dirname(get_package_share_directory("wonik_cobot_v2_description"))
    models += os.pathsep + os.environ.get("GAZEBO_MODEL_PATH", "")
    actions = [SetEnvironmentVariable("GAZEBO_PLUGIN_PATH", paths),
               SetEnvironmentVariable("GAZEBO_MODEL_PATH", models), gazebo, base,
               RegisterEventHandler(OnProcessExit(target_action=spawn,
                   on_exit=lambda event, context: [controllers] if event.returncode == 0 else [])), spawn]
    if val("rviz").lower() == "true":
        actions.append(Node(package="rviz2", executable="rviz2", parameters=[{"use_sim_time": True}],
            arguments=["-d", os.path.join(bringup, "rviz", "cobot_v2.rviz")]))
    return actions

def generate_launch_description():
    args = dict(world="empty", world_file="", sim_drive="kinematic", gui="true", rviz="true",
                lidar_gpu="false", depth_camera="true", ekf="true", x="0.0", y="0.0", yaw="0.0")
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in args.items()] +
                             [OpaqueFunction(function=setup)])
