#!/usr/bin/env bash
# Apply a named Goal B/C exploration profile to the running explore node.
# Sets brain_id + DFS knobs. Prefer writing /data/selected_brain.id before
# start_sim so the node constructs the correct brain at launch.
# Usage: apply_exploration_profile.sh exploration_policy_vlm_default
set -eo pipefail

export PATH=/usr/local/sbin:/usr/local/bin:/usr/bin:/bin:/opt/ros/jazzy/bin
source /opt/ros/jazzy/setup.bash
source /opt/explorer_workspace/ros_workspace/install/setup.bash

PROFILE="${1:-exploration_policy_vlm_default}"

case "$PROFILE" in
  exploration_policy_vlm_default|brain:vlm_tree_dfs)
    BRAIN=vlm_tree_dfs
    DFS_ORDER=highest
    PARENT_NEAREST=true
    ;;
  exploration_policy_greedy|brain:greedy_nearest)
    BRAIN=greedy_nearest
    DFS_ORDER=highest
    PARENT_NEAREST=true
    ;;
  brain:vlm_frontier_graph)
    BRAIN=vlm_frontier_graph
    DFS_ORDER=highest
    PARENT_NEAREST=true
    ;;
  brain:vlm_choice_dijkstra)
    BRAIN=vlm_choice_dijkstra
    DFS_ORDER=highest
    PARENT_NEAREST=true
    ;;
  *)
    echo "Unknown profile: $PROFILE" >&2
    exit 1
    ;;
esac

ros2 param set /explore brain_id "$BRAIN"
bash /workspace/scripts/set_dfs_order.sh "$DFS_ORDER"
ros2 param set /explore parent_to_nearest_node "$PARENT_NEAREST"
echo "Applied profile $PROFILE (brain=$BRAIN dfs=$DFS_ORDER parent_to_nearest=$PARENT_NEAREST)"
