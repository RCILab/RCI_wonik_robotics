# Safety behaviour: timeouts, faults and collision checks

What each layer does when an input goes missing, stale or invalid, how collisions are checked, and what
the drive unit driver (Wonik) has to provide. Values marked *placeholder* wait for the drive unit and
sensor specifications; nothing here is a confirmed hardware limit.

## Who stops the robot for what

| Layer | Checks | On failure |
|---|---|---|
| Drive unit driver (hardware interface, by Wonik) | its own watchdog on the steer/drive commands | must stop the wheels by itself (see [Driver requirements](#driver-requirements)) |
| `qd_kinematics` `kinematics_node` | `cmd_vel` age, `joint_states` age and content, start-up of every motion, drive response while driving | zero wheel speed / FAULT |
| `robot_localization` `ekf_odom` | wheel odometry + IMU yaw rate | (see [Odometry](#odometry)) |
| `stop_go_local_planner`, `pure_pursuit_local_planner` | odometry age; swept footprint of the commanded motion on the local costmap | controller fails / stop and wait |
| Nav2 local costmap | every observation source within its expected update interval | controller waits for a current costmap; the independent gate stops stale raw commands |
| `obstacle_detector` + `cmd_vel_gate` | sensor validity; protective field in the direction of travel; turn radius | zero command |
| `route_supervisor` | detour planning and execution | holds the robot or keeps it on the route behind the gate |

The higher layers only stop what they command. None of them can stop the wheels if the drive unit driver
keeps executing an old command, so the driver's watchdog is required, not optional.

Nav2 Humble waits while the costmap is not current; it does not publish a fresh zero on every wait
iteration. Here the gate rechecks raw-command age every 50 ms and stops after raw_timeout (0.5 s),
and it independently stops for stale required sensor data. See the
[Humble controller source](https://github.com/ros-navigation/navigation2/blob/humble/nav2_controller/src/controller_server.cpp).

## qd_kinematics

Parameters in `wonik_cobot_v2_bringup/config/kinematics.yaml`.

- `cmd_vel` older than `cmd_timeout_sec` (0.5 s): zero command, the base brakes at `linear_dec`. A
  non-finite `cmd_vel` is taken as a zero command. Teleop must repeat its command
  (`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=10.0`).
- `joint_states` is used for control only while an accepted sample is younger than
  `joint_state_timeout_sec` (0.2 s), measured on arrival and by its stamp. Otherwise the node commands
  zero wheel speed, holds the steering and restarts the state machine; it does not drive on old feedback.
  When feedback returns, the next motion goes through the measured start sequence below.
- A `joint_states` message is rejected when a configured steer/drive joint is missing, the position or
  velocity array does not match the name list, a required steer position or drive velocity is not finite, it has no stamp or a stamp more
  than `stamp_future_tolerance_sec` ahead of the node's clock.
- Stamps (`StampMonitor`): duplicates and anything older than the last accepted sample (however old)
  are dropped, never re-stamped; a gap over `odom_max_dt` is not integrated. Odometry carries the
  measurement stamp, so `/wheel/odom` for the EKF is consistent with the measurements. Only the node's
  own ROS clock going back (Gazebo world reset, bag restart) resets odometry and the state machine; a
  paused clock (dt = 0) skips the control step. Counters are in `/kinematics_node/debug_state`.
- Start-up supervision (stop → steer-align → creep → drive is unchanged): CREEP is left for DRIVE only
  when the *measured* wheel speeds follow the creep reference for `sync_hold_sec`; a converged command
  proves nothing. Time limits (*placeholder*):
  - WAIT_STOP: wheels stopped within speed / `linear_dec` + `wait_stop_timeout` (1 s), else FAULT (a
    drive that does not stop is not retried)
  - STEER_ALIGN within `align_timeout` (5 s), CREEP within `creep_timeout` (3 s); a miss restarts the
    sequence, `max_start_attempts` (2) misses without reaching DRIVE → FAULT
  - DRIVE: a wheel off its command by more than max(`drive_track_abs_tol`, `drive_track_rel_tol` ×
    command) for `drive_fault_time` (1 s) → FAULT (one module not driving included)
  - after a feedback dropout, a clock reset or a cleared fault the next motion always goes through
    WAIT_STOP → STEER_ALIGN → CREEP, even if the wheels already point the right way
- FAULT: zero wheel speed, steering held, reason on `/kinematics_node/fault` (transient local) and `/kinematics_node/debug_state`. It is latched:
  new `cmd_vel` messages and state machine restarts do not clear it. `ros2 service call
  /kinematics_node/clear_fault std_srvs/srv/Trigger "{}"` clears it while `joint_states` is current.

## Odometry

`ekf_odom` (robot_localization EKF, `config/ekf.yaml`) fuses the wheel body velocities with the IMU yaw
rate and is the only publisher of `/odom` and TF `odom → base_footprint`. If `/wheel/odom` stops, the
EKF keeps predicting with its last velocity; the kinematics node has stopped the base by then
(feedback timeout), but `/odom` drifts until the wheel odometry is back. robot_localization does not
estimate a gyro bias: the IMU driver has to remove it (e.g. calibrated at standstill).

## Local planners

Shared code in `local_planner_checks` (the same functions in both planners and in the tests):

- Every command is checked on the local costmap with the full footprint polygon at the start pose, at
  the end pose and in between, spaced so no footprint point moves more than half a cell between two
  checked poses (the corners of a turning rectangle included). A lethal cell, an unobserved cell
  (`NO_INFORMATION`, kept because the local costmap has `track_unknown_space: true`) or a pose outside
  the costmap blocks. Inflation does not.
- Translation: the motion the controller drives from the robot's actual pose (so being off the path or
  cutting a corner counts), over the stopping distance from the faster of the measured, commanded and
  planned speed: speed × `collision_latency` + speed² / (2 `dec_lim_x`) + margin. For `stop_go` this is
  the commanded twist held constant; for `avoid` the yaw rate ramps from the command toward the pursuit
  arc at `trans_acc_lim_theta`, as the next control steps command it.
- Turning in place: the whole remaining turn plus `rotation_margin`.
- Responsibilities: `stop_go` (the route, no avoidance) stops and waits while blocked; `avoid` (detours)
  also checks the path ahead over the same distance and stops; the route supervisor replans the detour.
- Odometry older than `odom_timeout` (0.5 s): the controller fails (Nav2 stops the robot).
- A segment end or the goal counts only by the actual distance to it (`xy_goal_tolerance`), with the
  heading (`yaw_goal_tolerance`) and the robot stopped; along-track distance is not enough. A small miss
  (sideways, overshoot, a localization correction after reaching it) is driven out sideways
  (`adjust_max_dist`, holonomic base), a larger one is approached again as a new segment.

## Obstacle detector and cmd_vel gate

`wonik_navigation/src/wonik_navigation/safety.py` holds the policy for both nodes.

- Sensor validity: every sensor in `required_topics` must deliver a cloud whose *measurement* time is
  within `timeout` (0.5 s) and that has at least `min_valid_points` finite points (an all-NaN depth
  image is no measurement; a real depth camera returns that for an object closer than its minimum
  range). A required sensor that is missing, stale or invalid sets `sensors_ok` false and all obstacle
  distances to 0; the gate then stops everything, in every supervisor state. One sensor working does not
  make up for another failing. A sensor in `optional_topics` keeps its last obstacles while stale.
- Switched off on purpose: `navigation.launch.py depth_camera:=false` removes the depth camera from the
  required sensors and the costmaps. With `depth_camera:=true` a silent camera stops the robot.
- Protective field: the detector reports how far the body (grown by `side_margin`) can move in 8
  directions before touching an obstacle. The gate stops a translation when the distance in the
  direction(s) it moves is below the stopping distance using the faster of the current and last output commands (gate `dec`, `latency`,
  `margin`). Turning in place stops for obstacle points within `turn_radius` of the centre.
- The supervisor state counts only while current (published at 10 Hz, `state_timeout` 0.5 s). Only a
  confirmed detour (AVOID) delegates the obstacle stops to the avoid controller; sensor validity is
  always checked.
- Every 50 ms the gate judges the command in force again (the last `cmd_vel_raw`, none once it is older
  than `raw_timeout`) and sends a stop at once when the result is more restrictive than what it sent last.
- Sensor coverage: the LiDAR sees 360° but not low objects close to the robot; the depth camera only
  looks forward. Turning-radius and side checks depend on the LiDAR (the simulated HDL-32E has a 0.3 m
  minimum range, the Mid-360 0.1 m).

The detector publishes /obstacle_distances as eight Float32MultiArray entries, in metres from the
expanded body: front, front-left, left, rear-left, rear, rear-right, right, front-right. Adjacent
directions bracket a command between bins. Infinity means no qualifying obstacle within max_range;
a failed required sensor yields zeros and sensors_ok=false. /obstacle_detected is retained as a
front-field display indicator; the gate uses the distance array.

The gate defaults are dec=0.5 m/s^2, latency=0.3 s and margin=0.3 m. Its speed input is derived from
commands, not measured odometry; the controller collision checks separately include measured speed.
The detector's side_margin defaults to 0.1 m and max_range to 3.0 m.

## Route supervisor

States on `/route_supervisor/state`: FOLLOW, AVOID_PREP, AVOID, AVOID_HOLD, DONE (see
`scripts/route_supervisor.py`).

- Entering avoidance (zone or mode): AVOID_PREP while the detour is planned. The route goal keeps
  running behind the closed gate, so the robot stops in front of an obstacle. Planning failures and
  timeouts (`plan_timeout`) are retried every `replan_period`; a detour goal that Nav2 rejects
  or aborts before it runs restarts the route goal.
- AVOID only once Nav2 reports feedback from the detour goal. A failed replan keeps the
  current detour. A detour that ends without success → AVOID_HOLD: the robot stands, the gate is closed,
  the detour is replanned until one runs.
- Results and feedback of replaced goals are ignored. Zone data older than `zone_timeout` counts as no
  zone.

## Driver requirements

For the drive unit driver that replaces Gazebo (Wonik):

- Watchdog: stop the drive wheels when no drive command arrives for a short time (e.g. 0.1–0.2 s at the
  100 Hz command rate of `kinematics_node`), independent of the ROS nodes above.
- `joint_states` with the measurement time in `header.stamp` (same clock as the ROS nodes), every
  steer joint's position and every drive joint's velocity, at a steady rate (100 Hz in simulation).
- Steer position and drive velocity interfaces as in `config/controllers.yaml`.
- IMU on `/imu/data` (frame `imu_link`, mount in `geometry.yaml`) with its gyro bias removed.

## Verification

Tests (`colcon test --executor sequential`):

| Package | Covers |
|---|---|
| `qd_kinematics` (26) | kinematics and odometry (the 18 original tests); CREEP needs measured wheel speed, one module not driving, wheel stall in DRIVE, FAULT latched until cleared, restart after reset/clear checks measured speed, non-finite input, stamp classification |
| `local_planner_checks` (8) | turning next to a wall, commanded arc vs. path, unobserved and off-map cells, start and end poses, inflation, stopping distance, goal by actual position (goal (5, 0), robot (5, 1): not reached) |
| `wonik_navigation` (19 policy + 9 ROS 2 cases) | corridors and protective field, required/optional sensors, invalid frames, measurement time, gate policy; the real `route_supervisor` and `cmd_vel_gate` against mock Nav2 action servers (detour planning times out, fails, is rejected, works); gate with a dead supervisor or detector, a turn stopped without a new command, a silent Nav2 |

Fault injection in Gazebo (`ros2 launch wonik_gazebo fault_injection.launch.py`, then
`ros2 run wonik_gazebo fault_injection.py <case>`): b1 joint_states lost while driving, b2 cmd_vel lost,
c drive controller not responding, h bad joint_states (NaN, future, late, duplicate, missing joints),
d box beside the robot while turning in place, g obstacle only the depth camera sees and the camera
stream lost.

Not verified here: the drive unit driver's watchdog (no driver in this repository), real sensors (the
Mid-360 and D435 behaviour, e.g. their invalid-measurement patterns), an exhaustive planner-level rotation/arc obstacle matrix in Gazebo (the shared collision checker
has unit tests, and the full route exercises the controllers).

The ROS 2 fault bench uses /camera_relay/enabled (std_srvs/SetBool) to simulate camera loss.
The all argument runs all six cases, restores the relay/controllers and removes spawned obstacles.
NaN injection invalidates all velocity entries so that the test includes a drive joint regardless
of ros2_control JointState ordering. The acceptance thresholds are unchanged from the ROS 1 tool.
See [Humble validation](humble-validation.md) for measured results.
