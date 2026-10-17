# RCI_wonik_robotics2 - ROS 2 Humble

ROS 2 Humble / Ubuntu 22.04 port of the reviewed Wonik QD AGV simulation.
Branch: **humble-devel**. The original ROS 1 working tree is separate and unchanged.
This directory was created as a Git worktree: its Git metadata lives in the original repository,
so keep that repository when moving or backing up this worktree.
ROS package names are retained; only the project directory has the suffix **2**.

## What is preserved

- Two diagonally arranged steer/drive modules, shared geometry, forward/inverse kinematics.
- Stop -> align -> creep -> drive control, feedback/command timeouts, latched fault and explicit reset.
- Wheel odometry + IMU EKF; AMCL localization and slam_toolbox mapping.
- StopGo route following and PurePursuit detours, stop/avoid zones, route rejoining.
- Full-footprint collision checks, predicted PurePursuit yaw-rate transitions, 8-direction protective
  fields with stopping distances, sensor freshness and NaN checks, and the final command gate.
- Controller handover at corners skips the completed segment stub.
- Six Gazebo fault-injection scenarios, including feedback loss and depth-camera loss.
- Kinematic and contact-physics simulation modes, the hall/map/U route, LiDAR and depth camera.

ROS interfaces use rclcpp/rclpy, ament/colcon, ros2_control, and Nav2 Controller plugins.
The route supervisor now calls Nav2 FollowPath and ComputePathToPose asynchronously.
Its planning timeout and safety heartbeat continue while an action is pending.

## Build on Ubuntu 22.04 with ROS 2 Humble

Place this directory under a workspace's src directory. Run in that workspace:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --parallel-workers 2
source install/setup.bash
colcon test --executor sequential
colcon test-result --verbose
```

Source install/setup.bash in each new terminal. Gazebo Classic 11 is deliberately retained
for parity with the original simulation. The project does not require a ROS 1 installation or bridge.

## Run the existing hall mission

Run these in three separate sourced terminals:

```bash
# 1: simulator, robot, joint controllers and EKF
ros2 launch wonik_gazebo cobot_v2.launch.py world:=wonik_hall rviz:=false

# 2: localization, Nav2, zones, obstacle detector and command gate
ros2 launch wonik_navigation navigation.launch.py

# 3: U route (0,0) -> (12,0) -> (12,6) -> (0,6)
ros2 launch wonik_navigation route.launch.py
```

For a headless display use gui:=false rviz:=false on the simulator and rviz:=false on navigation.
A depth camera still requires rendering; the Docker helpers start Xvfb automatically.
On native Linux without X, start Xvfb and set DISPLAY, or explicitly set depth_camera:=false
**on both** simulator and navigation launches to run with LiDAR only.

The empty world is the simulator default. Other simulator options:
sim_drive:=kinematic (default), sim_drive:=physics, ekf:=false, lidar_gpu:=true,
world_file:=/absolute/path.world, x:=0.0 y:=0.0 yaw:=0.0.
When changing the spawn pose, give navigation the same x/y/yaw for AMCL initialization.
The test worlds and mesh search paths resolve locally without downloading Gazebo models.

Navigation options include map:=wonik_hall, map_file:=/absolute/path.yaml, zones:=wonik_hall.
Route options include route:=wonik_hall_u, route_file:=/absolute/path.yaml,
controller:=stop_go or stop_go_slow. Custom route files contain frame_id and waypoints.

```bash
# Manual mode selection while the route supervisor is running
ros2 topic pub --once /route_supervisor/mode std_msgs/msg/String "{data: stop_go}"
ros2 topic pub --once /route_supervisor/mode std_msgs/msg/String "{data: avoid}"
ros2 topic pub --once /route_supervisor/mode std_msgs/msg/String "{data: auto}"

# Change the route-following controller
ros2 topic pub --once /route_supervisor/controller std_msgs/msg/String "{data: stop_go_slow}"

# Manual driving: use only when autonomous route execution is stopped
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_raw
```

RViz shows the map, route, plans, zones and detection box. Use 2D Pose Estimate for localization.
Missions are started by route.launch.py; a free-space RViz goal is not a route command.

## Fault injection

Stop the mission and simulator first, then run the dedicated empty-world bench in two terminals:

~~~bash
ros2 launch wonik_gazebo fault_injection.launch.py
ros2 run wonik_gazebo fault_injection.py all
~~~

Individual cases: b1 feedback loss, b2 command loss, c unresponsive drive controller,
h invalid/stale joint states, d side obstacle during rotation, g low obstacle followed by camera loss.
The runner prints measurements and PASS/FAIL and returns a nonzero exit code on failure.
It restores controllers and the camera relay and removes its obstacles on exit.
The default is headless; rendering still needs a display (Docker supplies Xvfb).
Use gui:=true for the Gazebo window.

The detector publishes /obstacle_distances (Float32MultiArray): front, front-left, left, rear-left,
rear, rear-right, right, front-right. The gate selects the directions of translation and computes
a protective distance using the faster of the current and last output command speeds, with
dec=0.5 m/s^2, latency=0.3 s and margin=0.3 m. These are cmd_vel_gate ROS parameters.
The legacy /obstacle_detected Boolean remains available for display.

See [safety behaviour](docs/safety.md) for timeout, collision-check and fault-reset details.

## Mapping

Start the simulator in the hall, then run mapping instead of navigation/localization:

```bash
ros2 launch wonik_navigation mapping.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f /absolute/output/map
```

The mapping launch supplies the cloud-to-scan conversion and slam_toolbox.
The teleop command above addresses the base directly; the navigation command gate is not launched
during this mapping-only session. Localization alone is available through localization.launch.py.

## Docker on Windows / Linux

From the RCI_wonik_robotics2 directory. If PowerShell blocks local scripts, use
powershell -NoProfile -ExecutionPolicy Bypass -File docker/run.ps1 followed by the same arguments;
this applies only to that process.

```powershell
docker build -t rci_wonik_humble:latest docker
.\docker\run.ps1 colcon build --symlink-install --parallel-workers 2
.\docker\run.ps1 colcon test --executor sequential
.\docker\run.ps1 colcon test-result --verbose
```

Then use three PowerShell terminals:

```powershell
.\docker\run.ps1 ros2 launch wonik_gazebo cobot_v2.launch.py world:=wonik_hall gui:=false rviz:=false
.\docker\run.ps1 ros2 launch wonik_navigation navigation.launch.py rviz:=false
.\docker\run.ps1 ros2 launch wonik_navigation route.launch.py
```

On Linux / WSL, use bash docker/run.sh with the same arguments.
The Bash helper forwards an existing X11/WSLg display; the PowerShell helper uses a headless display.
All helper terminals share the persistent container rci_wonik_humble, so Docker Desktop
does not need cross-container DDS discovery. Build artifacts stay in volume rci_wonik_humble_ws.
The source is a live bind mount. IMAGE, CONTAINER and ROS_DOMAIN_ID can be set before first creation.
The default domain is 42. Changing image, mount or display settings requires recreating the container.
Use docker stop rci_wonik_humble to stop the session; the build volume remains.

## Active configuration and hardware boundary

| File | Purpose |
| --- | --- |
| wonik_cobot_v2_description/config/geometry.yaml | Module locations, wheel size, joints and body geometry |
| wonik_cobot_v2_bringup/config/kinematics.yaml | Limits, alignment, timeouts and fault handling |
| wonik_cobot_v2_bringup/config/controllers.yaml | ros2_control joints and controller types |
| wonik_cobot_v2_bringup/config/ekf.yaml | Wheel/IMU fusion |
| wonik_navigation/config/navigation.yaml | Nav2 costmaps, all controller variants, goal/progress checks |
| wonik_navigation/config/cobot_v2/obstacle_detector.yaml | Required sensors, freshness and detection regions |
| wonik_navigation/config/routes and config/zones | Route vertices and zone polygons |

The retired MBF and split ROS 1 planner/costmap files have been consolidated into navigation.yaml.
StopGo may wait behind an obstacle indefinitely; RouteProgressChecker intentionally preserves this
behavior. SettledGoalChecker enforces the configured pose tolerance and stopped hold time.

Command flow:

```text
Nav2 controller -> /cmd_vel_raw -> cmd_vel_gate -> /cmd_vel
  -> qd_kinematics -> /steer_controller/commands (rad)
                   -> /drive_controller/commands (rad/s)
joint_states -> wheel/odom -> EKF (+ /imu/data) -> /odom and odom -> base_footprint TF
AMCL or SLAM -> map -> odom TF
```

The two command arrays follow geometry.yaml joint order. Both feedback position and velocity
must be supplied with measurement timestamps. Joint controller command topics have the ROS 2
plural suffix /commands. LiDAR is /points and depth is /camera/depth/points.

For hardware, provide the ros2_control SystemInterface (or a driver exposing the same commands
and joint_states), actual sensor drivers, and controller-manager startup. base.launch.py accepts
hardware_plugin and use_sim_time:=false; it does not itself start hardware controller_manager.
Use use_sim_time:=false for navigation and route as well.
Gazebo-only plugins are confined to the description/simulator; the planner and kinematics core
have no Gazebo API dependency. Physical steering/braking/slip commissioning remains a hardware task.

```bash
ros2 topic echo /kinematics_node/fault --qos-durability transient_local
ros2 service call /kinematics_node/clear_fault std_srvs/srv/Trigger "{}"
ros2 topic echo /cmd_vel_gate/reason
ros2 topic echo /obstacle_detector/status
```

Fault reset requires fresh feedback and returns through the stop/align sequence.

## Validation

See [Humble validation](docs/humble-validation.md) for builds, tests, mission and fault-injection results.
The later ROS 1 directional safety, controller and fault-injection changes are included.
The existing GIFs under docs/gifs are inherited ROS 1 demonstrations.

## Contact

Maintainer: yoonhyuck Jung (wjddbsgur501@khu.ac.kr)  
Lab: [RCI Lab @ Kyung Hee University](https://rcilab.khu.ac.kr)
