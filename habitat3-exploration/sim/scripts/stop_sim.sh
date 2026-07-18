#!/usr/bin/env bash
# Elytra "Stop Episode": tear down tmux-owned processes AND any orphaned ROS/Habitat
# nodes left outside the session (hot-swaps, failed launches, agent debug runs).
set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Prefer graceful interrupt of the launch tree first when still in tmux.
tmux send-keys -t habitat C-c 2>/dev/null || true
sleep 2
tmux kill-session -t habitat 2>/dev/null || true

CLEANUP_KILL_TMUX=1 bash "${SCRIPT_DIR}/cleanup_episode.sh"
