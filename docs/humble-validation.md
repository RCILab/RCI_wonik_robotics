# Humble port validation

Validated on 2026-10-17 with Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11,
running in Docker Desktop on Windows. C++17 and Python 3.10.

## Source baseline

The port uses the ROS 1 working-tree snapshot copied at approximately 12:45 KST,
including the reviewed, uncommitted fixes present at that time. Its Git base is c86c2cd.
The original ROS 1 files were not edited by the porting work.

The original working tree was independently updated during this task (including directional
obstacle fields, PurePursuit adjustments and a fault-injection tool). Those later changes are
**not included** in this port. The original 105 file hashes are recorded in
[ros1-source-snapshot.sha256](ros1-source-snapshot.sha256) to make this boundary explicit.

## Build and tests

- All 8 packages built successfully, both in the development container and from a fresh
  build volume using docker/run.ps1.
- 56 individual test cases passed: 26 kinematics, 8 footprint/collision/goal,
  14 sensor/gate policy, 4 ROS 2 callback regressions and 4 Nav2 action integration cases.
- colcon test-result reported 61 tests, 0 errors, 0 failures, 0 skipped;
  that total includes the 5 CTest suite wrappers.
- Action tests exercise planning timeout, planning failure, rejected detour and successful
  detour feedback. Failed requests keep the gate closed.
- ROS integration tests use localhost domain 174. Do not run operational nodes in this domain.
- Remaining test warnings are dependency deprecations from transforms3d/NumPy.

## Live simulation

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
