#!/usr/bin/env bash
set -e
REPO="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${IMAGE:-rci_wonik_humble:latest}"
CONTAINER="${CONTAINER:-rci_wonik_humble}"
if ! docker container inspect "$CONTAINER" >/dev/null 2>&1; then
  ARGS=(--name "$CONTAINER" --shm-size=1g
        --mount "type=volume,source=rci_wonik_humble_ws,target=/ros2_ws"
        --mount "type=bind,source=$REPO,target=/ros2_ws/src/RCI_wonik_robotics2"
        -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}")
  # All terminals use this one container, including on Docker Desktop.
  if [ -n "${DISPLAY:-}" ] && [ -d /tmp/.X11-unix ]; then
    xhost +local:root >/dev/null 2>&1 || true
    ARGS+=(-e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1
           -v /tmp/.X11-unix:/tmp/.X11-unix:rw)
    [ ! -e /dev/dri ] || ARGS+=(--device /dev/dri)
  fi
  docker run -d "${ARGS[@]}" "$IMAGE" >/dev/null
else
  docker start "$CONTAINER" >/dev/null
fi
TTY=()
[ ! -t 0 ] || TTY=(-it)
[ "$#" -gt 0 ] || set -- bash
exec docker exec "${TTY[@]}" "$CONTAINER" /usr/local/bin/wonik-command "$@"
