#!/usr/bin/env bash
# Send a fixed movement demo sequence via /movement/discrete_move.
# Requires the exploration episode (ROS bridge + habitat engine) to be running.
set -eo pipefail

export PATH=/usr/local/sbin:/usr/local/bin:/usr/bin:/bin:/opt/ros/jazzy/bin
source /opt/ros/jazzy/setup.bash
source /opt/explorer_workspace/ros_workspace/install/setup.bash

ACTION="/movement/discrete_move"
ACTION_TYPE="explorer_msgs/action/DiscreteMove"
# habitat_engine ActuationSpec uses 10 deg per turn step → 9 steps ≈ 90°
TURN_STEPS_90="${HABITAT_TURN_STEPS_90:-9}"

FORWARD=0
BACKWARD=1
TURN_LEFT=2
TURN_RIGHT=3

send_move() {
    local direction="$1"
    local steps="$2"
    local label="$3"
    echo ">>> ${label} (direction=${direction}, steps=${steps})"
    ros2 action send_goal "$ACTION" "$ACTION_TYPE" \
        "{direction: ${direction}, steps: ${steps}}" --feedback
}

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

send_move "$FORWARD" 1 "forward 1 step"
send_move "$TURN_LEFT" "$TURN_STEPS_90" "rotate left 90°"
send_move "$FORWARD" 2 "forward 2 steps"
send_move "$TURN_RIGHT" "$TURN_STEPS_90" "rotate right 90°"
send_move "$BACKWARD" 2 "backward 2 steps"

echo "Movement demo sequence complete."
