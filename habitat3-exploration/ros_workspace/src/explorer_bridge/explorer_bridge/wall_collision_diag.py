"""TEMP: wall-collision diagnostics — delete after root-causing planner vs wall bumps.

Appends rows to /data/temp_wall_collision_diag.csv (host: sim/data/).
"""

from __future__ import annotations

import csv
import math
import threading
import time
from pathlib import Path
from typing import Any, Iterable, Optional, Sequence

# TEMP: wall collision diag
_CSV_PATH = Path("/data/temp_wall_collision_diag.csv")
_LOCK = threading.Lock()
_FIELDS = (
    "wall_time",
    "event",
    "lin_x",
    "ang_z",
    "direction",
    "action",
    "collided",
    "pose_x",
    "pose_y",
    "yaw_rad",
    "dx",
    "dy",
    "dist_m",
    "plan_n",
    "plan_nx",
    "plan_ny",
    "cross_track_m",
    "heading_to_wp_rad",
    "yaw_err_rad",
    "note",
)

_DIR_NAME = {
    0: "FORWARD",
    1: "BACKWARD",
    2: "TURN_LEFT",
    3: "TURN_RIGHT",
}


def direction_name(direction: int) -> str:
    return _DIR_NAME.get(int(direction), str(direction))


def _ensure_header() -> None:
    try:
        _CSV_PATH.parent.mkdir(parents=True, exist_ok=True)
        if not _CSV_PATH.exists() or _CSV_PATH.stat().st_size == 0:
            with _CSV_PATH.open("w", newline="", encoding="utf-8") as f:
                csv.DictWriter(f, fieldnames=_FIELDS).writeheader()
    except OSError:
        pass


def log_event(event: str, **fields: Any) -> None:
    """Append one diagnostic row. Unknown keys ignored; missing → empty."""
    row = {k: "" for k in _FIELDS}
    row["wall_time"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    row["event"] = event
    for k, v in fields.items():
        if k not in row:
            continue
        if v is None:
            continue
        if isinstance(v, float):
            row[k] = f"{v:.4f}"
        elif isinstance(v, bool):
            row[k] = "1" if v else "0"
        else:
            row[k] = str(v)
    with _LOCK:
        try:
            _ensure_header()
            with _CSV_PATH.open("a", newline="", encoding="utf-8") as f:
                csv.DictWriter(f, fieldnames=_FIELDS).writerow(row)
        except OSError:
            pass


def nearest_plan_metrics(
    pose_x: float,
    pose_y: float,
    yaw_rad: float,
    plan_xy: Sequence[tuple[float, float]],
) -> dict[str, Any]:
    """Cross-track / heading error to nearest plan waypoint (map frame)."""
    if not plan_xy:
        return {
            "plan_n": 0,
            "plan_nx": "",
            "plan_ny": "",
            "cross_track_m": "",
            "heading_to_wp_rad": "",
            "yaw_err_rad": "",
        }
    best_i = 0
    best_d2 = float("inf")
    for i, (px, py) in enumerate(plan_xy):
        d2 = (px - pose_x) ** 2 + (py - pose_y) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_i = i
    nx, ny = plan_xy[best_i]
    # Prefer a look-ahead waypoint when available (closer to RPP carrot).
    look = plan_xy[min(best_i + 3, len(plan_xy) - 1)]
    hx, hy = look[0] - pose_x, look[1] - pose_y
    heading = math.atan2(hy, hx)
    yaw_err = heading - yaw_rad
    while yaw_err > math.pi:
        yaw_err -= 2.0 * math.pi
    while yaw_err < -math.pi:
        yaw_err += 2.0 * math.pi
    return {
        "plan_n": len(plan_xy),
        "plan_nx": nx,
        "plan_ny": ny,
        "cross_track_m": math.sqrt(best_d2),
        "heading_to_wp_rad": heading,
        "yaw_err_rad": yaw_err,
    }


def path_to_xy(poses: Iterable) -> list[tuple[float, float]]:
    out: list[tuple[float, float]] = []
    for p in poses:
        out.append((float(p.pose.position.x), float(p.pose.position.y)))
    return out
