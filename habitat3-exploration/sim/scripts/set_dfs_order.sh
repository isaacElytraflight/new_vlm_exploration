#!/usr/bin/env bash
# Toggle DFS child selection: highest-openness-first vs lowest-first.
# Usage: set_dfs_order.sh highest|lowest
# Requires a running exploration episode (explore node alive).
set -eo pipefail

export PATH=/usr/local/sbin:/usr/local/bin:/usr/bin:/bin:/opt/ros/jazzy/bin
source /opt/ros/jazzy/setup.bash
source /opt/explorer_workspace/ros_workspace/install/setup.bash

ORDER="${1:-highest}"
case "$ORDER" in
  highest)
    VALUE=true
    LABEL="highest openness first"
    ;;
  lowest)
    VALUE=false
    LABEL="lowest openness first"
    ;;
  *)
    echo "Usage: $0 highest|lowest" >&2
    exit 1
    ;;
esac

NODE="/explore"
PARAM="dfs_prefer_highest_openness"

echo "Waiting for ${NODE}..."
for _ in $(seq 1 30); do
  if ros2 param describe "$NODE" "$PARAM" >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done
if ! ros2 param describe "$NODE" "$PARAM" >/dev/null 2>&1; then
  echo "ERROR: ${NODE} / ${PARAM} not available. Start the exploration episode first." >&2
  exit 1
fi

echo "Setting ${PARAM}=${VALUE} (${LABEL})"
ros2 param set "$NODE" "$PARAM" "$VALUE"
echo "DFS child order: ${LABEL}"
