#!/usr/bin/env python3
"""Collect per-run metrics inside habitat3-sim while an episode runs.

Waits on exploration/status (exploration_complete) with timeout, logs /odom
trajectory and coverage_snapshot time series, then fetches privileged
get_coverage_stats via Habitat IPC.

Usage (in container):
  python3 /workspace/scripts/experiment_collect.py --timeout-s 7200 \\
      --output /data/experiments/run_metrics.json
"""

from __future__ import annotations

import argparse
import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from explorer_msgs.msg import ExplorationStatus
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray

from explorer_bridge.habitat_ipc import DEFAULT_SOCKET_PATH, HabitatIpcClient, HabitatIpcError


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


def write_episode_progress(path: Path | None, *, phase: str, detail: str) -> None:
    if path is None:
        return
    payload = {
        "phase": phase,
        "detail": detail,
        "updated_at": utc_now_iso(),
    }
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    except OSError:
        pass


class ExperimentCollector(Node):
    def __init__(
        self,
        *,
        timeout_s: float,
        sample_hz: float,
        progress_file: Path | None = None,
    ) -> None:
        super().__init__("experiment_collect")
        self._timeout_s = float(timeout_s)
        self._sample_period = 1.0 / max(0.1, float(sample_hz))
        self._started_mono = time.monotonic()
        self._started_at = utc_now_iso()
        self._progress_file = progress_file

        self._complete = False
        self._phase = "waiting"
        self._detail = "waiting for exploration/status"
        self._distance_m = 0.0
        self._prev_xy: Optional[Tuple[float, float]] = None
        self._trajectory: List[List[float]] = []
        self._coverage_samples: List[List[float]] = []
        self._last_sample_mono = 0.0
        self._last_progress_mono = 0.0
        self._latest_cov = 0.0

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            ExplorationStatus, "exploration/status", self._status_cb, status_qos
        )
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(
            Float32MultiArray, "exploration/debug/coverage_snapshot", self._snap_cb, 10
        )
        write_episode_progress(
            self._progress_file,
            phase=self._phase,
            detail=self._detail,
        )

    def _status_cb(self, msg: ExplorationStatus) -> None:
        self._phase = str(msg.phase)
        self._detail = str(msg.detail)
        if bool(msg.exploration_complete):
            self._complete = True
        self._maybe_write_progress()

    def _odom_cb(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        t_s = time.monotonic() - self._started_mono
        if self._prev_xy is not None:
            dx = x - self._prev_xy[0]
            dy = y - self._prev_xy[1]
            self._distance_m += math.hypot(dx, dy)
        self._prev_xy = (x, y)
        self._trajectory.append([x, y, t_s])

    def _snap_cb(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < 4:
            return
        now = time.monotonic()
        if now - self._last_sample_mono < self._sample_period:
            return
        self._last_sample_mono = now
        meters = float(msg.data[0])
        cov = float(msg.data[3])
        self._latest_cov = cov
        t_s = now - self._started_mono
        self._coverage_samples.append([t_s, meters, cov])

    def _maybe_write_progress(self) -> None:
        now = time.monotonic()
        if now - self._last_progress_mono < 1.0:
            return
        self._last_progress_mono = now
        write_episode_progress(
            self._progress_file,
            phase=self._phase,
            detail=self._detail,
        )

    def wait_until_done(self) -> Dict[str, Any]:
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            self._maybe_write_progress()
            elapsed = time.monotonic() - self._started_mono
            if self._complete:
                return self._build_payload(status="completed", elapsed=elapsed)
            if elapsed >= self._timeout_s:
                return self._build_payload(
                    status="timeout",
                    elapsed=elapsed,
                    error_message=f"timeout after {self._timeout_s:.0f}s (phase={self._phase})",
                )
        return self._build_payload(status="error", elapsed=0.0, error_message="rclpy shutdown")

    def _build_payload(
        self,
        *,
        status: str,
        elapsed: float,
        error_message: Optional[str] = None,
    ) -> Dict[str, Any]:
        mapped_m2 = 0.0
        gt_m2 = 0.0
        try:
            explored, gt, _mpp = HabitatIpcClient(DEFAULT_SOCKET_PATH).get_coverage_stats()
            mapped_m2 = float(explored)
            gt_m2 = float(gt)
        except HabitatIpcError as exc:
            if status == "completed":
                status = "error"
            error_message = error_message or f"get_coverage_stats failed: {exc}"

        final_cov = self._latest_cov
        if gt_m2 > 0.0:
            final_cov = max(0.0, min(1.0, mapped_m2 / gt_m2))

        return {
            "status": status,
            "error_message": error_message,
            "started_at": self._started_at,
            "finished_at": utc_now_iso(),
            "duration_s": float(elapsed),
            "distance_m": float(self._distance_m),
            "mapped_m2": mapped_m2,
            "gt_m2": gt_m2,
            "final_coverage": float(final_cov),
            "phase": self._phase,
            "detail": self._detail,
            "coverage_samples": self._coverage_samples,
            "trajectory": self._trajectory,
        }


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Collect experiment run metrics.")
    parser.add_argument("--timeout-s", type=float, default=7200.0)
    parser.add_argument("--sample-hz", type=float, default=2.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--progress-file",
        type=Path,
        default=None,
        help="Episode phase JSON for Elytra ablation progress",
    )
    args = parser.parse_args(argv)

    args.output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = ExperimentCollector(
        timeout_s=args.timeout_s,
        sample_hz=args.sample_hz,
        progress_file=args.progress_file,
    )
    try:
        payload = node.wait_until_done()
    finally:
        node.destroy_node()
        rclpy.shutdown()

    args.output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(json.dumps({"status": payload["status"], "output": str(args.output)}))
    return 0 if payload["status"] == "completed" else 1


if __name__ == "__main__":
    raise SystemExit(main())
