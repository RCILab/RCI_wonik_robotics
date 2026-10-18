#!/usr/bin/env bash
set -e
if [ "${DISPLAY:-:99}" = ":99" ] && ! mountpoint -q /tmp/.X11-unix; then
  # Depth sensors require rendering even with gui:=false.
  # A stopped container can leave its own Xvfb lock/socket behind. Never remove
  # a host-mounted X11 socket (the Linux/WSLg helper forwards that display).
  rm -f /tmp/.X99-lock /tmp/.X11-unix/X99
  Xvfb :99 -screen 0 1280x720x24 >/tmp/xvfb.log 2>&1 &
fi
exec sleep infinity
