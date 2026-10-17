#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
cd /ros2_ws
[ ! -f install/setup.bash ] || source install/setup.bash
exec "$@"
