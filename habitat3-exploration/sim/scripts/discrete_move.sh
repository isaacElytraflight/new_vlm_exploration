#!/usr/bin/env bash
# One-shot DiscreteMove: discrete_move.sh <forward|backward|turn_left|turn_right> [steps]
# Requires a running exploration episode (ROS bridge + habitat engine).
set -eo pipefail

export PATH=/usr/local/sbin:/usr/local/bin:/usr/bin:/bin:/opt/ros/jazzy/bin
source /opt/ros/jazzy/setup.bash
source /opt/explorer_workspace/ros_workspace/install/setup.bash

ACTION="/movement/discrete_move"
ACTION_TYPE="explorer_msgs/action/DiscreteMove"

DIRECTION_NAME="${1:-}"
STEPS="${2:-1}"

case "${DIRECTION_NAME}" in
  forward|FORWARD) DIRECTION=0 ;;
  backward|BACKWARD) DIRECTION=1 ;;
  turn_left|left|TURN_LEFT) DIRECTION=2 ;;
  turn_right|right|TURN_RIGHT) DIRECTION=3 ;;
  *)
    echo "Usage: $0 <forward|backward|turn_left|turn_right> [steps]" >&2
    exit 2
    ;;
esac

if ! [[ "${STEPS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "ERROR: steps must be a positive integer, got: ${STEPS}" >&2
  exit 2
fi

echo "Waiting for ${ACTION} server..."
for _ in $(seq 1 60); do
  if ros2 action info "$ACTION" 2>/dev/null | grep -q "Action servers: 1"; then
    break
  fi
  sleep 0.5
done
if ! ros2 action info "$ACTION" 2>/dev/null | grep -q "Action servers: 1"; then
  echo "ERROR: ${ACTION} is not available. Start the exploration episode first." >&2
  exit 1
fi

echo ">>> ${DIRECTION_NAME} x${STEPS} (direction=${DIRECTION})"
ros2 action send_goal "$ACTION" "$ACTION_TYPE" \
  "{direction: ${DIRECTION}, steps: ${STEPS}}" --feedback

echo "Discrete move complete."
