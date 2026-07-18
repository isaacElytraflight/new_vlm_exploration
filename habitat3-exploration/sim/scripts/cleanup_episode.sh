#!/usr/bin/env bash
# Kill ALL Habitat exploration episode processes — including orphans left outside
# the tmux session (e.g. docker exec ros2 run hot-swaps). Safe to run repeatedly.
#
# Used by stop_sim.sh (Stop Episode) and start_sim.sh (before a fresh launch).

set -uo pipefail

_kill_patterns=(
  "habitat_engine.py"
  "live_viewer.py"
  "elytra_view_server.py"
  "feh.*habitat_live"
  "ros2 launch explorer_mission"
  "ros2 launch explorer_bridge"
  "explorer_bridge_node"
  "depth_to_laserscan"
  "depth_camera_info"
  "known_pose_mapper"
  "cmd_vel_to_discrete"
  "habitat_map_node"
  "async_slam_toolbox"
  "slam_toolbox_node"
  "maprender_node"
  "map_renderer"
  "explore_node"
  "actions_node"
  "frontier_vlm_client"
  "vlm_node"
  "vlm_server"
  "explore_episode.py"
  "static_transform_publisher .* map odom"
  "static_transform_publisher .* base_link depth_frame"
  "ros2 run explorer_bridge"
  "ros2 run explorer_mission"
  "ros2 launch nav2"
)

_soft_kill() {
  local pat
  for pat in "${_kill_patterns[@]}"; do
    pkill -INT -f "$pat" 2>/dev/null || true
  done
}

_hard_kill() {
  local pat
  for pat in "${_kill_patterns[@]}"; do
    pkill -KILL -f "$pat" 2>/dev/null || true
  done
}

_soft_kill
sleep 1
_hard_kill

# Drop IPC socket so a new engine cannot attach to a dead peer.
rm -f /tmp/habitat_engine.sock 2>/dev/null || true
rm -f /tmp/elytra_teleop.sock 2>/dev/null || true

# Optional: kill leftover tmux habitat session (Stop Episode also does this).
if [ "${CLEANUP_KILL_TMUX:-0}" = "1" ]; then
  tmux kill-session -t habitat 2>/dev/null || true
fi

echo "Episode cleanup complete."
exit 0
