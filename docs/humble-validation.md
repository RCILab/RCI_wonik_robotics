# Humble port validation

Validated on 2026-10-17 with Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11,
running in Docker Desktop on Windows. C++17 and Python 3.10.

## Source baseline

The initial migration (commit 8c815a7, based on c86c2cd) used the ROS 1 working tree at
approximately 12:45 KST. At the user's request, the subsequent ROS 1 changes were also ported:
8-direction obstacle distances and speed-dependent gate fields, PurePursuit yaw-rate ramp prediction,
StopGo corner handover, updated safety tests/docs and all six fault-injection cases.

[ros1-source-snapshot.sha256](ros1-source-snapshot.sha256) records the 108 source files in the
updated snapshot captured at 13:40 KST on 2026-10-17. The older 105-file snapshot remains in commit 8c815a7.
The original ROS 1 files were not edited by the porting work.

## Build and tests

- All 8 updated packages built successfully. The initial port also passed a fresh build-volume
  check using docker/run.ps1; this follow-up used that existing build volume.
- 62 individual test cases passed: 26 kinematics, 8 footprint/collision/goal,
  19 sensor/gate policy, 4 ROS 2 callback regressions, 1 DDS gate-timeout scenario and
  4 Nav2 action integration cases.
- colcon test-result reported 68 tests, 0 errors, 0 failures, 0 skipped;
  that total includes the 6 CTest suite wrappers.
- Action tests exercise planning timeout, planning failure, rejected detour and successful
  detour feedback. Failed requests keep the gate closed.
- ROS integration tests use localhost domain 174. Do not run operational nodes in this domain.
- Remaining test warnings are dependency deprecations from transforms3d/NumPy.

## Initial migration: live simulation

The default depth-camera + CPU LiDAR configuration was used in wonik_hall.
The following were observed with actual ROS 2 nodes and Gazebo, not mock action servers:

- Robot spawning; joint_state_broadcaster, steer_controller and drive_controller activation.
- LiDAR, depth point clouds, IMU, wheel odometry, EKF and localization TF.
- All three Nav2 controller plugins, both goal-checker instances and global planner activation.
- Full U route: (0,0) -> (12,0) -> (12,6) -> (0,6).
- Supervisor sequence: FOLLOW -> AVOID_PREP -> AVOID -> FOLLOW -> DONE.
  The avoid-zone section exercised Navfn planning, PurePursuit execution, periodic replanning
  and handover back to StopGo. No extra obstacle was injected into this mission.
- Final map-frame position (0.003694, 6.002006) m; XY error 0.004204 m.
  Final yaw 3.128587 rad; absolute yaw error 0.013006 rad (about 0.75 degrees).
- At completion the outgoing velocity command was exactly zero; estimated speed
  0.001053 m/s, yaw rate 0.003071 rad/s, required sensors current.
- Under rendering/build load, a temporary depth-camera delay closed the gate and fresh
  sensor data reopened it. Safety timeouts were not relaxed to accommodate the load.
- Mapping was started after stopping AMCL/navigation. slam_toolbox published a
  321 x 201 map at 0.05 m resolution; map_saver_cli successfully saved PGM and YAML.
- The physics mode also spawned and activated all joint controllers. Its physical
  tracking/slip performance was not characterized.
- RViz initialized with the ROS 2 configuration and received the map. The Xvfb software
  renderer logged an indexed-map shader warning; interactive desktop rendering was not
  visually verified.

## Updated fault-injection bench

All six cases passed in one run of fault_injection.py all on a freshly launched bench:

| Case | Observed result |
| --- | --- |
| b1: joint feedback loss | Stopped after 0.23 simulated seconds; resumed through WAIT_STOP/STEER_ALIGN/CREEP to 0.50 m/s |
| b2: command loss | Stopped after 1.28 s and 0.44 m, within the configured timeout plus braking allowance |
| c: unresponsive drive | FAULT latched, no movement after a new command; explicit clear_fault restored measured startup and 1.21 m travel |
| h: invalid feedback | 27 injections per category; wheel/ground-truth travel difference 0.1 cm; finite outputs; maximum wheel command 6.58 rad/s |
| d: side obstacle | Blocked rotation 0.0 degrees; 1.68 m forward travel alongside; 133.0 degrees rotation after removal |
| g: low obstacle / camera loss | Camera detected the obstacle; loss made sensors_ok false; total movement 0.000 m |

The NaN injector was adapted to invalidate required drive feedback regardless of JointState ordering.
The camera relay uses a ROS 2 SetBool service. Cleanup restores controllers/relay and removes obstacles.
The source acceptance thresholds were retained. Some camera frames exceeded the freshness threshold
under software rendering; the gate stopped for those intervals without relaxed timeouts.

## Updated mission and controller handover

The full U route was rerun after the latest changes with the default LiDAR and depth camera:

- FOLLOW -> AVOID_PREP -> AVOID -> FOLLOW -> DONE; the avoid zone exercised Navfn, the updated
  PurePursuit controller and route rejoining.
- At the first corner, switched stop_go -> stop_go_slow while turning (yaw 0.201 rad).
  Switched back to stop_go after advancing 0.50 m along the second leg. Both goals produced feedback,
  and the route continued through the second corner to the final goal.
- Final map-frame pose (0.007766, 5.996265, 3.126937) m/m/rad.
  Position error 0.008617 m; yaw error 0.014656 rad
  (0.84 degrees), within the configured 0.03 m / 0.02 rad limits.
- Final /cmd_vel exactly zero; estimated speed 0.000666 m/s and yaw rate
  0.004270 rad/s; required sensors current.
- No additional obstacle was injected into this mission. The side/low-obstacle checks were performed
  in the dedicated fault bench above.

## Reproduce

From the project directory in PowerShell:

```powershell
docker build -t rci_wonik_humble:latest docker
powershell -NoProfile -ExecutionPolicy Bypass -File docker/run.ps1 colcon build --symlink-install --parallel-workers 2
powershell -NoProfile -ExecutionPolicy Bypass -File docker/run.ps1 colcon test --executor sequential
powershell -NoProfile -ExecutionPolicy Bypass -File docker/run.ps1 colcon test-result --verbose
```

Run the three mission launch commands in [README](../README.md) in separate terminals.
Monitor /route_supervisor/state, /cmd_vel_gate/reason and /obstacle_detector/status.
Simulation results are not measurements of hardware stopping distance or steering response.
Hardware commissioning remains separate from this migration.
