#!/usr/bin/env bash
# Apply a named Goal B exploration profile to the running explore node.
# Usage: apply_exploration_profile.sh exploration_policy_vlm_default
set -eo pipefail

export PATH=/usr/local/sbin:/usr/local/bin:/usr/bin:/bin:/opt/ros/jazzy/bin
source /opt/ros/jazzy/setup.bash
source /opt/explorer_workspace/ros_workspace/install/setup.bash

PROFILE="${1:-exploration_policy_vlm_default}"

case "$PROFILE" in
  exploration_policy_vlm_default)
    DFS_ORDER=highest
    PARENT_NEAREST=true
    ;;
  exploration_policy_greedy)
    DFS_ORDER=lowest
    PARENT_NEAREST=false
    ;;
  *)
    echo "Unknown profile: $PROFILE" >&2
    exit 1
    ;;
esac

bash /workspace/scripts/set_dfs_order.sh "$DFS_ORDER"
ros2 param set /explore parent_to_nearest_node "$PARENT_NEAREST"
echo "Applied profile $PROFILE (dfs=$DFS_ORDER parent_to_nearest=$PARENT_NEAREST)"
