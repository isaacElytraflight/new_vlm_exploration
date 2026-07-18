#!/usr/bin/env bash
# Fast teleop: one step via explorer_bridge unix socket (no ros2 CLI / sourcing).
# Usage: teleop_step.sh <forward|backward|turn_left|turn_right>
set -euo pipefail

DIR="${1:-}"
case "${DIR}" in
  forward|backward|turn_left|turn_right) ;;
  *)
    echo "Usage: $0 <forward|backward|turn_left|turn_right>" >&2
    exit 2
    ;;
esac

SOCK="${ELYTRA_TELEOP_SOCKET:-/tmp/elytra_teleop.sock}"

/usr/bin/python3 - "$SOCK" "$DIR" <<'PY'
import socket
import sys

sock_path, direction = sys.argv[1], sys.argv[2]
s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
s.settimeout(5.0)
try:
    s.connect(sock_path)
except OSError as exc:
    print(f"ERROR: teleop socket {sock_path} unavailable ({exc}). Is the episode running?", file=sys.stderr)
    sys.exit(1)
s.sendall((direction + "\n").encode("utf-8"))
reply = s.recv(256).decode("utf-8", errors="replace").strip()
s.close()
print(reply)
if not reply.startswith("ok:"):
    sys.exit(1)
PY
