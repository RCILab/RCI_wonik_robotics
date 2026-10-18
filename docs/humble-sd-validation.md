# Humble QD/SD validation (2026-10-18)

ROS 1 source: ae1bb3a on main. The [source manifest](ros1-source-snapshot.sha256) records
all 135 files from that commit. The ROS 1 working tree was left unchanged. This update is on
humble-devel in RCI_wonik_robotics2; [the earlier validation](humble-validation.md) is historical.

Environment: Ubuntu 22.04, ROS 2 Humble, Gazebo Classic 11, Docker Desktop on Windows;
CPU LiDAR and depth camera with Xvfb rendering, default kinematic drive for navigation.

## Changes carried over

- Unified wonik_cobot_description / wonik_cobot_bringup, `config/<robot>` and common launch files.
- cobot_v2 QD plus the drawing-based virtual SD model, single-drive tricycle kinematics,
  fork-axle turning centre, nonholonomic final adjustment and differential AMCL odometry.
- Curvature filtering/rate limits, extended pursuit lookahead and stable path headings.
- Nonconvex and off-centre footprint sweeps, signed turn clearances and stopping-angle gate.
- LiDAR body-point filtering before scan projection and obstacle detection.
- Costmap-triggered detours, blocked-only replanning and heading-aligned early rejoining.

ROS 2 adaptations include lifecycle-aware controller blocked publishers, Nav2 full/incremental
costmap subscriptions and asynchronous action clients. A single-line JSON string transports the
footprint parameter: multiline YAML flow strings are not accepted by Humble's footprint parser.
The global costmap publishes a full grid at 1 Hz so a supervisor started or restarted later receives
current obstacles. Humble's [costmap publisher](https://github.com/ros-navigation/navigation2/blob/humble/nav2_costmap_2d/src/costmap_2d_publisher.cpp)
retains only the last full grid and does not refresh it for each new subscriber when using incremental
publication alone. Incremental-update support is retained in the supervisor.
The point-cloud filter handles row padding, byte order and arbitrary extra fields while preserving
measurement time. Docker startup also clears a stale container-owned Xvfb socket/lock on restart;
host-mounted X11 sockets are excluded. Camera output was verified after restarting the container.

## Build and tests

All 8 packages passed a clean colcon build after removing the old generated build/install overlay.
79 individual tests passed; colcon test-result reports 86 including 7 CTest wrappers,
with 0 errors, 0 failures and 0 skipped.

| Suite | Cases |
| --- | ---: |
| Kinematics and fault state machine | 26 |
| Footprint collision and goal checks | 10 |
| Detector/gate policy | 26 |
| Route/costmap helpers | 5 |
| ROS 2 callbacks and cloud-format regressions | 6 |
| DDS gate timeouts | 1 |
| Nav2 action / supervisor integration | 5 |

The additional integration scenario starts in a clear avoid zone, inserts a lethal cell through
an incremental costmap update, checks early rejoining, retains an unblocked detour, then triggers
replanning through the blocked topic. Proximity without heading alignment must not hand control
back to StopGo. Test nodes use localhost ROS_DOMAIN_ID=174.

## Live obstacle missions

Each robot runs the U route (0,0) -> (12,0) -> (12,6) -> (0,6), in a fresh wonik_hall simulation.
Two static boxes of size 0.6 x 0.6 x 1.0 m are placed at (6,0) and (6.5,6).
The first is inside the avoid zone; the second is removed only after the robot has remained
stopped in FOLLOW for 1.5 simulated seconds with zero cmd_vel and measured speed below 0.02 m/s.
Avoidance starts only inside an avoid zone: an obstacle too close to its entrance can correctly
stop the robot before it enters the zone, especially with the longer SD footprint.

Both missions must reach DONE, use stop_go -> avoid -> stop_go, show lateral deviation around
the first box, stop and resume for the second box, have no latched kinematics fault, and finish
with zero cmd_vel, current required sensors, XY error below 0.03 m and yaw error below 0.02 rad.
The SD raw lateral command must remain exactly zero throughout.

| Result | cobot_v2 (QD) | SD |
| --- | ---: | ---: |
| Final XY error (m) | 0.010685 | 0.007865 |
| Final yaw error (rad) | 0.008972 | 0.004516 |
| Maximum detour lateral deviation (m) | 1.016 | 1.059 |
| Stopped before second box at X (m) | 8.471 | 9.067 |
| Estimated final speed (m/s) | 0.001530 | 0.002097 |
| Final cmd_vel | [0, 0, 0] | [0, 0, 0] |
| Mission assertions | PASS | PASS |

Both runs observed FOLLOW -> AVOID_PREP -> AVOID -> FOLLOW -> DONE. SD's raw vy stayed 0.0.
The two final errors are measured in the localized map frame. Brief depth-camera delays
under rendering load in the QD run were handled by the unchanged sensor timeout, which closed
the gate, and fresh data reopened it. No safety limit was relaxed.

## Fault-injection bench

All six cases passed in a single fresh cobot_v2 bench run of fault_injection.py all:

| Case | Measurement |
| --- | --- |
| b1 feedback loss | Stopped after 0.27 simulated seconds; restarted through WAIT_STOP / STEER_ALIGN / CREEP / DRIVE to 0.50 m/s |
| b2 command loss | Stopped after 1.33 s and 0.45 m; matched timeout plus braking allowance |
| c unresponsive drive | FAULT latched; new commands moved 0.000 m; explicit clear restored startup and 1.21 m travel |
| h bad feedback | 27 injections per category; wheel/ground-truth travel difference 0.1 cm; finite outputs, max wheel command 6.58 rad/s |
| d side obstacle | 21 degree turn clearance; 0.0 degrees blocked rotation, 1.65 m travel alongside, 136.7 degrees rotation after removal |
| g low obstacle / camera loss | Camera initially detected the box; loss set sensors_ok false; robot moved 0.000 m |

For case d, gate logs explicitly reported 21 degrees of counterclockwise clearance below the
25-degree stopping requirement. The diagnostic reason topic checks forward motion separately;
its incidental sensor-delay message is not the evidence for the rotation assertion.
The runner restored controllers/camera relay and removed the injected obstacles on completion.

## SD mapping, physics startup and late costmap subscription

- sim_drive:=physics spawned SD and activated joint_state_broadcaster, steer_controller and
  drive_controller. This was a startup check, not a characterization of physical tracking/slip.
- With navigation running, a box at (3,0) was added after an initial clear global grid was received.
  A subsequent full grid contained lethal cells on the box surface. A second subscriber created
  afterward received that updated grid with a newer stamp (5.45 s -> 6.57 s simulation time).
- After stopping navigation, mapping.launch.py robot:=sd ran the body-point filter, scan projection
  and slam_toolbox. map_saver_cli saved a 321 x 201 PGM and YAML at 0.05 m/pixel.

## Reproduce

Use the build/test and three-terminal QD/SD commands in [README](../README.md). To run the bench:

```bash
ros2 launch wonik_gazebo fault_injection.launch.py
# another terminal
ros2 run wonik_gazebo fault_injection.py all
```

For SD mapping and physics startup, with navigation and the route stopped:

```bash
ros2 launch wonik_gazebo sd.launch.py world:=wonik_hall sim_drive:=physics gui:=false rviz:=false
# another terminal
ros2 launch wonik_navigation mapping.launch.py robot:=sd rviz:=false
# another terminal
ros2 run nav2_map_server map_saver_cli -f /tmp/sd_map --ros-args -p use_sim_time:=true
```

The obstacle missions used ROS domains 81/82, the fault bench 83, and the SD auxiliary checks 84.
Simulation launches were stopped after validation. Full autonomous physics missions and interactive
RViz rendering were not part of this update; the tested autonomous missions used kinematic mode.
