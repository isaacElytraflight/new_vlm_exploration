#!/usr/bin/env bash
# (Re)start Xvfb + x11vnc + noVNC if missing. Safe to call after container
# restart or before feh in start_sim.sh.
set -euo pipefail

DISPLAY="${DISPLAY:-:1}"
export DISPLAY
NUM="${DISPLAY#:}"

if ! pgrep -x Xvfb >/dev/null 2>&1; then
  rm -f "/tmp/.X${NUM}-lock" "/tmp/.X11-unix/X${NUM}" 2>/dev/null || true
  mkdir -p /tmp/.X11-unix
  Xvfb "$DISPLAY" -screen 0 1280x720x24 &
fi

for _ in $(seq 1 40); do
  if [ -S "/tmp/.X11-unix/X${NUM}" ]; then
    break
  fi
  sleep 0.25
done

if [ ! -S "/tmp/.X11-unix/X${NUM}" ]; then
  echo "ensure_display: X socket /tmp/.X11-unix/X${NUM} not ready" >&2
  exit 1
fi

if ! pgrep -f "x11vnc.*rfbport 5900" >/dev/null 2>&1; then
  x11vnc -display "$DISPLAY" -forever -shared -nopw -quiet -bg -rfbport 5900 \
    -wait 10 -defer 10 -threads
  sleep 0.5
fi

if ! pgrep -f "websockify.*6080" >/dev/null 2>&1; then
  websockify --web=/usr/share/novnc 6080 localhost:5900 &
fi
