#!/bin/bash
# Run a command (default: interactive shell) in the Noetic container.
# This repository is mounted at /catkin_ws/src/RCI_wonik_robotics, so edits on the host apply
# immediately. build/ and devel/ stay in the Docker volume "rci_wonik_ws" between runs.
#   docker/run.sh catkin_make                                         # build (again after C++ changes)
#   docker/run.sh roslaunch wonik_cobot_v2_bringup gazebo.launch
#   docker/run.sh rosrun teleop_twist_keyboard teleop_twist_keyboard.py
REPO="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${IMAGE:-rci_wonik_noetic:latest}"

# --net=host: every docker/run.sh terminal talks to the same roscore.
ARGS=(--rm --net=host --ipc=host
      -v rci_wonik_ws:/catkin_ws
      -v "$REPO:/catkin_ws/src/RCI_wonik_robotics"
      -e ROS_LOG_DIR=/tmp/ros_log)
[ -t 0 ] && ARGS+=(-it)

# Gazebo/RViz windows: forward the X11 display when there is one (Linux desktop, WSLg).
if [ -n "$DISPLAY" ] && [ -d /tmp/.X11-unix ]; then
  xhost +local:root >/dev/null 2>&1 || true
  ARGS+=(-e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
  [ -e /dev/dri ] && ARGS+=(--device /dev/dri)
fi

exec docker run "${ARGS[@]}" "$IMAGE" bash -c \
  "source /opt/ros/noetic/setup.bash; [ -f devel/setup.bash ] && source devel/setup.bash; ${*:-bash}"
