#!/usr/bin/env bash
set -e
if [ "${DISPLAY:-:99}" = ":99" ]; then
  # Depth sensors require rendering even with gui:=false.
  Xvfb :99 -screen 0 1280x720x24 >/tmp/xvfb.log 2>&1 &
fi
exec sleep infinity
