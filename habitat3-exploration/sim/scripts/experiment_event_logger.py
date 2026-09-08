#!/usr/bin/env python3
"""Text-only ROS event logger for ablation run packages.

Writes JSONL (optionally gzip) for exploration/status, frontier_tree, and
vlm/scores — never image payloads.
"""

from __future__ import annotations

import argparse
import gzip
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, TextIO

import rclpy
from explorer_msgs.msg import (
    ExplorationStatus,
    FrontierOpennessScores,
    FrontierTree,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


class EventLogger(Node):
    def __init__(
        self,
        *,
        output: Path,
        gzip_out: bool,
        max_bytes: int,
        stop_flag_path: Path | None,
    ) -> None:
        super().__init__("experiment_event_logger")
        self._output = Path(output)
        self._output.parent.mkdir(parents=True, exist_ok=True)
        self._gzip = bool(gzip_out)
        self._max_bytes = int(max_bytes)
        self._stop_flag_path = Path(stop_flag_path) if stop_flag_path else None
        self._bytes_written = 0
        self._closed = False

        if self._gzip and not str(self._output).endswith(".gz"):
            self._output = Path(str(self._output) + ".gz")

        self._fh: TextIO
        if self._gzip:
            self._fh = gzip.open(self._output, "wt", encoding="utf-8")  # type: ignore[assignment]
        else:
            self._fh = self._output.open("w", encoding="utf-8")

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            ExplorationStatus, "exploration/status", self._status_cb, status_qos
        )
        self.create_subscription(
            FrontierTree, "exploration/frontier_tree", self._tree_cb, status_qos
        )
        self.create_subscription(
            FrontierOpennessScores, "exploration/vlm/scores", self._scores_cb, 10
        )
        self.create_timer(0.5, self._check_stop)

    def _check_stop(self) -> None:
        if self._stop_flag_path and self._stop_flag_path.is_file():
            self.close()
            raise SystemExit(0)

    def _emit(self, topic: str, fields: Dict[str, Any]) -> None:
        if self._closed:
            return
        if self._bytes_written >= self._max_bytes:
            return
        line = json.dumps(
            {"t": utc_now_iso(), "topic": topic, **fields},
            separators=(",", ":"),
            ensure_ascii=True,
        )
        # Soft truncate oversized reasoning fields already handled by callers.
        payload = line + "\n"
        if self._bytes_written + len(payload.encode("utf-8")) > self._max_bytes:
            self._closed = True
            return
        self._fh.write(payload)
        self._fh.flush()
        self._bytes_written += len(payload.encode("utf-8"))

    def _status_cb(self, msg: ExplorationStatus) -> None:
        self._emit(
            "exploration/status",
            {
                "phase": str(msg.phase),
                "current_node_id": int(msg.current_node_id),
                "target_node_id": int(msg.target_node_id),
                "exploration_complete": bool(msg.exploration_complete),
                "detail": str(msg.detail)[:240],
            },
        )

    def _tree_cb(self, msg: FrontierTree) -> None:
        nodes = []
        for n in msg.nodes:
            nodes.append(
                {
                    "id": int(n.id),
                    "parent_id": int(n.parent_id),
                    "openness_score": int(n.openness_score),
                    "fully_explored": bool(n.fully_explored),
                    "x": float(n.position.x),
                    "y": float(n.position.y),
                    "children_ids": [int(c) for c in n.children_ids],
                }
            )
        self._emit(
            "exploration/frontier_tree",
            {
                "current_node_id": int(msg.current_node_id),
                "nodes": nodes,
            },
        )

    def _scores_cb(self, msg: FrontierOpennessScores) -> None:
        reasonings = [str(r)[:160] for r in msg.reasonings]
        self._emit(
            "exploration/vlm/scores",
            {
                "frontier_ids": [int(i) for i in msg.frontier_ids],
                "scores": [int(s) for s in msg.scores],
                "reasonings": reasonings,
            },
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        try:
            self._fh.close()
        except Exception:  # noqa: BLE001
            pass


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Log compact ROS exploration events.")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gzip", action="store_true", default=True)
    parser.add_argument("--no-gzip", action="store_true")
    parser.add_argument(
        "--max-bytes",
        type=int,
        default=2_000_000,
        help="Hard size budget for the events file (default 2 MB)",
    )
    parser.add_argument("--stop-flag", type=Path, default=None)
    parser.add_argument("--max-runtime-s", type=float, default=0.0)
    args = parser.parse_args(argv)

    gzip_out = bool(args.gzip) and not bool(args.no_gzip)

    rclpy.init()
    node = EventLogger(
        output=args.output,
        gzip_out=gzip_out,
        max_bytes=args.max_bytes,
        stop_flag_path=args.stop_flag,
    )
    started = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.stop_flag and Path(args.stop_flag).is_file():
                break
            if args.max_runtime_s > 0 and (time.monotonic() - started) >= args.max_runtime_s:
                break
    except SystemExit:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
