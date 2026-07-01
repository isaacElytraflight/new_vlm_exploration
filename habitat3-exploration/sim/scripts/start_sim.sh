#!/usr/bin/env bash
# Elytra-bridge startScriptPath target: Habitat engine + ROS bridge + noVNC viewer.
# Note: no `set -u` — ROS/conda setup scripts reference optional unbound vars.
set -eo pipefail

export DISPLAY="${DISPLAY:-:1}"
bash /workspace/scripts/ensure_display.sh

LIVE_DIR=/tmp/habitat_live
FRAME="$LIVE_DIR/frame.jpg"
mkdir -p "$LIVE_DIR"

# Seed placeholder frame for noVNC (conda env has numpy/imageio).
if [ ! -f "$FRAME" ]; then
    (
        source /opt/conda/etc/profile.d/conda.sh
        conda activate habitat
        python - <<'EOF'
import numpy as np, imageio.v2 as imageio
imageio.imwrite("/tmp/habitat_live/frame.jpg",
                np.zeros((480, 640, 3), dtype=np.uint8), quality=85)
EOF
    )
fi

pkill -f "feh.*habitat_live" 2>/dev/null || true
pkill -f "live_viewer.py" 2>/dev/null || true
pkill -f "habitat_engine.py" 2>/dev/null || true
rm -f /tmp/habitat_engine.sock 2>/dev/null || true
sleep 0.2

# Run conda-based processes in isolated subshells so conda env (PATH, LD_LIBRARY_PATH,
# PYTHONPATH) never leaks into the ROS process below.
(
    source /opt/conda/etc/profile.d/conda.sh
    conda activate habitat
    exec python /workspace/scripts/live_viewer.py
) &
VIEWER_PID=$!

(
    source /opt/conda/etc/profile.d/conda.sh
    conda activate habitat
    exec python /workspace/scripts/habitat_engine.py
) &
ENGINE_PID=$!

cleanup() {
    kill "$ENGINE_PID" "$VIEWER_PID" "${VIEW_SERVER_PID:-}" 2>/dev/null || true
    pkill -f "habitat_engine.py" 2>/dev/null || true
    pkill -f "live_viewer.py" 2>/dev/null || true
    pkill -f "elytra_view_server.py" 2>/dev/null || true
    rm -f /tmp/habitat_engine.sock 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Wait for IPC socket
for _ in $(seq 1 60); do
    if [ -S /tmp/habitat_engine.sock ]; then
        break
    fi
    sleep 0.5
done
if [ ! -S /tmp/habitat_engine.sock ]; then
    echo "habitat_engine failed to start (no socket)" >&2
    exit 1
fi

# ROS bridge runs on system Python 3.12 — keep conda out of PATH.
export PATH=/usr/local/sbin:/usr/local/bin:/usr/bin:/bin:/opt/ros/jazzy/bin
source /opt/ros/jazzy/setup.bash
source /opt/explorer_workspace/ros_workspace/install/setup.bash

pkill -f "elytra_view_server.py" 2>/dev/null || true
(
  exec python3 /opt/elytra/elytra_view_server.py
) &
VIEW_SERVER_PID=$!

exec ros2 launch explorer_mission exploration.launch.py driver_backend:=habitat
