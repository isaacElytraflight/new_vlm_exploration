#!/usr/bin/env bash
set -eu

echo "[entrypoint] bringing up virtual display + noVNC"
/workspace/scripts/ensure_display.sh

echo "[entrypoint] creating tmux session 'habitat'"
tmux new-session -d -s habitat 2>/dev/null || true

echo "[entrypoint] ready — noVNC at http://localhost:6080/vnc.html"
exec tail -f /dev/null
