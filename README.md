ROS 2 Humble-based simulation, kinematics, SLAM, and autonomous navigation for Wonik Robotics AGVs.

## Overview

This repository contains ROS 2 packages for Wonik Robotics AGVs, developed through a joint research collaboration between Wonik Robotics and the RCI Lab at Kyung Hee University. Two models are available: **cobot_v2** (QD #2 type, without the arm) and **SD** (single-drive pallet-truck type, virtual model).

## Dependencies

- Ubuntu 22.04 / ROS 2 Humble
- Gazebo Classic 11
- Nav2, ros2_control, robot_localization and slam_toolbox (installed through rosdep or the Docker image)

## Packages

Clone this repository into the `src/` folder of a colcon workspace.

| Package | Contents |
| --- | --- |
| `qd_kinematics` | Steer-drive kinematics for N-module swerve and single-drive tricycle bases. `/cmd_vel` to steering angles and wheel speeds through a stop / steer-align / creep / drive state machine; stamped joint feedback to wheel odometry. Command and feedback timeouts, measured startup checks and latched faults. ROS-independent core with unit tests. |
| `wonik_cobot_description` | URDF/xacro and geometry in `urdf/<robot>.urdf.xacro` and `config/<robot>/geometry.yaml`. cobot_v2 uses meshes converted from Wonik's STEP model. SD uses primitives with dimensions from the drawing, pending the actual CAD. Gazebo-specific tags are in separate `.gazebo.xacro` files. |
| `wonik_cobot_bringup` | Per-robot launch files (`cobot_v2.launch.py`, `sd.launch.py`) through `base.launch.py robot:=<robot>`. Loads the model, ros2_control configuration and kinematics, with wheel odometry on `/wheel/odom`. The EKF fuses wheel velocities and IMU yaw rate into `/odom` and `odom -> base_footprint` TF (`ekf:=false`: wheel odometry only). |
| `wonik_gazebo` | Per-robot simulation launch files through `sim.launch.py`: world, robot, joint controllers and bringup. Includes the `qd_kinematic_base` plugin, an empty world and `wonik_hall`, a 16 m x 10 m test hall with a central island. |
| `wonik_navigation` | Robot-specific configuration under `config/<robot>/`, selected with `robot:=<robot>` (default `cobot_v2`). LiDAR body-point removal, laser-scan projection, slam_toolbox mapping, AMCL localization and Nav2 route following. Includes the route supervisor, stop/avoid zones, obstacle detector and final command gate. |
| `stop_go_local_planner` | Nav2 Controller plugin for straight route segments and in-place turns at corners, up to 1.0 m/s. Stops and waits when blocked. Small endpoint errors are corrected sideways on QD (`holonomic: true`) or along the heading on SD (`false`). |
| `pure_pursuit_local_planner` | Nav2 Controller plugin for curved detours. Slows in curves, smooths curvature changes and stops when the swept footprint or path ahead is blocked. Reports blockage to the route supervisor for replanning. |
| `local_planner_checks` | Shared swept-footprint collision checks for translations and turns, including nonconvex polygons and off-centre rotation. Unknown and off-map cells block motion. Includes stopping-distance checks and Nav2 goal/progress checkers. |

The robot-side packages use ros2_control position interfaces for steering, velocity interfaces for drive wheels and stamped `/joint_states` feedback. A hardware driver supplies these interfaces and the actual sensor topics. Timeouts, faults, collision checks and driver requirements are described in [docs/safety.md](docs/safety.md).

In Gazebo, `sim_drive` selects how the wheels move the base:

- `kinematic` (default): frictionless wheel contacts; the Gazebo plugin moves the base using the twist implied by measured steering angles and wheel speeds. Used for SLAM and navigation.
- `physics`: wheel contacts drive the base through friction, including slip. Used for inspecting kinematics and wheel odometry; the EKF also uses simulated IMU yaw rate.

## Results

The following GIFs were recorded from **ROS 2 Humble**, using Gazebo and RViz2. Playback is accelerated; the captures use the default kinematic drive mode.

### cobot_v2 (QD #2 type)

Basic motion in Gazebo, commanded through `/cmd_vel` (2.5x playback):

| Mode 1: straight, sideways, diagonal | Mode 2: arc turn | Mode 3: spin in place |
| :---: | :---: | :---: |
| <img src="docs/gifs/cobot_v2_mode1.gif" width="280"> | <img src="docs/gifs/cobot_v2_mode2.gif" width="280"> | <img src="docs/gifs/cobot_v2_mode3.gif" width="280"> |

2D SLAM and AMCL localization in the test hall (`world:=wonik_hall`). The RViz2 captures show the map and laser scan; mapping builds the map while driving around the central island, and AMCL uses that saved map (8x playback).

| Test hall (Gazebo) | Mapping (slam_toolbox) | Localization (AMCL) |
| :---: | :---: | :---: |
| <img src="docs/images/wonik_hall_gazebo.png" width="280"> | <img src="docs/gifs/cobot_v2_mapping.gif" width="280"> | <img src="docs/gifs/cobot_v2_localization.gif" width="280"> |

Stop-and-go along the U route `(0,0) -> (12,0) -> (12,6) -> (0,6)`: straight legs at up to 1.0 m/s, a stop and an in-place turn at each corner (5x playback, the same run in RViz2 and Gazebo).

| U route (RViz2) | Gazebo |
| :---: | :---: |
| <img src="docs/gifs/cobot_v2_stop_go_route.gif" width="420"> | <img src="docs/gifs/cobot_v2_stop_go_route_gazebo.gif" width="420"> |

Zones on the same route (`config/zones/wonik_hall.yaml`): an obstacle in the avoid zone triggers a detour and an early return to the route. In the stop zone, the robot waits in front of the obstacle and continues after it is removed (5x playback).

| Zones (RViz2) | Gazebo |
| :---: | :---: |
| <img src="docs/gifs/cobot_v2_zones.gif" width="420"> | <img src="docs/gifs/cobot_v2_zones_gazebo.gif" width="420"> |

### SD (virtual model)

SD has one steered drive unit in the body and fixed rollers at the fork tip. It cannot move sideways and turns about the middle of the roller axle (`base_footprint`), with the body swinging around it. The drawing-based model uses the same packages and planners with `config/sd/`.

Basic motion in Gazebo (2.5x playback):

| Mode 1: straight | Mode 2: arc turn | Mode 3: turn in place |
| :---: | :---: | :---: |
| <img src="docs/gifs/sd_mode1.gif" width="280"> | <img src="docs/gifs/sd_mode2.gif" width="280"> | <img src="docs/gifs/sd_mode3.gif" width="280"> |

Stop-and-go along the U route (`stop_go`, AMCL with the differential odometry model; 5x playback):

| U route (RViz2) | Gazebo |
| :---: | :---: |
| <img src="docs/gifs/sd_stop_go_route.gif" width="420"> | <img src="docs/gifs/sd_stop_go_route_gazebo.gif" width="420"> |

Obstacle avoidance and stop zones on the same route (5x playback):

| Zones (RViz2) | Gazebo |
| :---: | :---: |
| <img src="docs/gifs/sd_zones.gif" width="420"> | <img src="docs/gifs/sd_zones_gazebo.gif" width="420"> |

## Quick start

With ROS 2 Humble installed:

```bash
source /opt/ros/humble/setup.bash
sudo apt install python3-colcon-common-extensions python3-rosdep ros-humble-teleop-twist-keyboard
# First ROS workspace only: sudo rosdep init
rosdep update
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --branch humble-devel --single-branch https://github.com/RCILab/RCI_wonik_robotics.git RCI_wonik_robotics2
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
colcon build --symlink-install --parallel-workers 2
source install/setup.bash

ros2 launch wonik_gazebo cobot_v2.launch.py
# Or SD, in place of the launch above:
ros2 launch wonik_gazebo sd.launch.py

# Another terminal; source install/setup.bash in every terminal
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.3 -p turn:=0.5
```

The base stops when `/cmd_vel` is older than 0.5 s. Hold a movement key to keep publishing commands with keyboard repeat; the Humble keyboard teleop has no `repeat_rate` parameter. Teleop keys: `i` / `,` forward/back, `Shift+J` / `Shift+L` sideways (QD only), `j` / `l` turn in place, `u` / `o` arcs, `k` stop, `z` slower. A large steering change causes a stop, wheel alignment and creep before driving.

Mapping and localization in the test hall:

```bash
ros2 launch wonik_gazebo cobot_v2.launch.py world:=wonik_hall rviz:=false
# Another terminal: drive with teleop while mapping
ros2 launch wonik_navigation mapping.launch.py
# Save the completed map
ros2 run nav2_map_server map_saver_cli -f /absolute/output/wonik_hall --ros-args -p use_sim_time:=true

# Stop mapping and restart the simulator, then localize on the saved map
ros2 launch wonik_navigation localization.launch.py map_file:=/absolute/output/wonik_hall.yaml
```

Omit `map_file` to use the included hall map. For SD, use `sd.launch.py` and add `robot:=sd` to mapping, localization or navigation. The route launch is shared.

Route following with zones, in three separate terminals:

```bash
ros2 launch wonik_gazebo cobot_v2.launch.py world:=wonik_hall rviz:=false
ros2 launch wonik_navigation navigation.launch.py
ros2 launch wonik_navigation route.launch.py
```

Mode and controller selection during a mission:

```bash
ros2 topic pub --once /route_supervisor/mode std_msgs/msg/String "{data: avoid}"          # auto | stop_go | avoid
ros2 topic pub --once /route_supervisor/controller std_msgs/msg/String "{data: stop_go_slow}"
```

Data flow: Nav2 controller -> `/cmd_vel_raw` -> command gate -> `/cmd_vel` -> `qd_kinematics`. Clear avoid zones remain on the route; a blocked route triggers a detour. Running detours replan only when blocked and hand back to StopGo after aligning with the route. Outside a confirmed detour, the gate checks directional stopping distances and footprint-based turn clearances. Required sensor loss stops motion in every state.

Simulation options include `sim_drive:=physics`, `ekf:=false`, `gui:=false rviz:=false`, `world_file:=/absolute/path.world` and `x:=0.0 y:=0.0 yaw:=0.0`. Give navigation the same initial pose if the spawn pose changes. Custom maps and routes use `map_file:=/absolute/path.yaml` and `route_file:=/absolute/path.yaml`.

A depth camera needs an X display even without the Gazebo GUI. Use Xvfb on headless Linux, or pass `depth_camera:=false` to **both** simulation and navigation for LiDAR-only operation. The Docker helpers supply a display. Teleop during navigation goes to `/cmd_vel_raw`; stop autonomous route execution before manual driving. Mapping-only teleop goes directly to `/cmd_vel`.

Tests and fault injection:

```bash
colcon test --executor sequential
colcon test-result --verbose

# Separate empty-world bench, with the normal simulator and mission stopped
ros2 launch wonik_gazebo fault_injection.launch.py
# Another terminal
ros2 run wonik_gazebo fault_injection.py all
```

The test suites cover kinematics, collision checks, detector/gate policy and Nav2 action integration. The Gazebo bench covers feedback loss, command loss, an unresponsive drive, invalid feedback, side obstacles during rotation and depth-camera loss. See [safety behaviour](docs/safety.md) for details.

## Docker

The image includes ROS 2 Humble, Gazebo 11 and the project dependencies. Build artifacts stay in the `rci_wonik_humble_ws` volume; the source directory is mounted into `/ros2_ws/src/RCI_wonik_robotics2`.

```bash
git clone --branch humble-devel --single-branch https://github.com/RCILab/RCI_wonik_robotics.git RCI_wonik_robotics2
cd RCI_wonik_robotics2
docker build -t rci_wonik_humble:latest docker
bash docker/run.sh colcon build --symlink-install --parallel-workers 2
bash docker/run.sh ros2 launch wonik_gazebo cobot_v2.launch.py world:=wonik_hall rviz:=false
# Separate terminals
bash docker/run.sh ros2 launch wonik_navigation navigation.launch.py
bash docker/run.sh ros2 launch wonik_navigation route.launch.py
```

On Windows with Docker Desktop, use PowerShell:

```powershell
.\docker\run.ps1 colcon build --symlink-install --parallel-workers 2
.\docker\run.ps1 ros2 launch wonik_gazebo sd.launch.py world:=wonik_hall gui:=false rviz:=false
# Separate terminals
.\docker\run.ps1 ros2 launch wonik_navigation navigation.launch.py robot:=sd rviz:=false
.\docker\run.ps1 ros2 launch wonik_navigation route.launch.py
```

- The helpers share the `rci_wonik_humble` container and ROS domain 42. Running a helper without arguments opens a shell.
- The Bash helper forwards an X11/WSLg display. The PowerShell helper uses Xvfb for headless rendering.
- If PowerShell blocks scripts, use `powershell -NoProfile -ExecutionPolicy Bypass -File docker/run.ps1` followed by the same arguments.
- `docker stop rci_wonik_humble` stops the session and retains the build volume.

## Contact

Maintainer: yoonhyuck Jung (wjddbsgur501@khu.ac.kr)  
Lab: [RCI Lab @ Kyung Hee University](https://rcilab.khu.ac.kr)
