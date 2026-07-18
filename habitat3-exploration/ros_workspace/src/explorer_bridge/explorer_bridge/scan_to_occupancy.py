"""Known-pose occupancy mapping: integrate LaserScan rays into a grid (no SLAM)."""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

UNKNOWN = -1
FREE = 0
OCCUPIED = 100


def scan_content_signature(ranges: list[float] | np.ndarray, *, decimals: int = 1) -> tuple:
    """Fingerprint a FOV wedge; similar views should compare equal across noise."""
    arr = np.asarray(ranges, dtype=float)
    finite = arr[np.isfinite(arr)]
    if finite.size == 0:
        return (0, ())
    # Subsample quantized ranges so near-identical wedges match after a turn.
    step = max(1, finite.size // 32)
    sample = tuple(round(float(v), decimals) for v in finite[::step])
    return (
        int(finite.size),
        round(float(finite.min()), decimals),
        round(float(finite.max()), decimals),
        round(float(finite.mean()), decimals),
        sample,
    )


def signatures_similar(a: tuple, b: tuple, *, mean_tol: float = 0.15) -> bool:
    """True when two scan fingerprints are the same FOV (allow tiny numeric drift)."""
    if a is None or b is None:
        return False
    if a[0] != b[0]:
        return False
    if len(a) < 4 or len(b) < 4:
        return a == b
    if a[1] is None or b[1] is None:
        return a == b
    if abs(float(a[1]) - float(b[1])) > mean_tol:
        return False
    if abs(float(a[2]) - float(b[2])) > mean_tol:
        return False
    if abs(float(a[3]) - float(b[3])) > mean_tol:
        return False
    if len(a) >= 5 and len(b) >= 5 and a[4] and b[4]:
        # Fraction of subsample bins that differ by more than mean_tol.
        sa, sb = a[4], b[4]
        n = min(len(sa), len(sb))
        if n == 0:
            return True
        diffs = sum(1 for i in range(n) if abs(sa[i] - sb[i]) > mean_tol)
        return (diffs / n) <= 0.15
    return True


def should_integrate_with_tf(*, stamp_lookup_ok: bool) -> bool:
    """Only integrate when pose was resolved for the scan stamp.

    Falling back to an unmatched / arbitrarily old pose paints the current FOV at
    a stale yaw while the robot is turning.
    """
    return bool(stamp_lookup_ok)


def stamp_msg_to_ns(stamp) -> int:
    """Convert builtin_interfaces Time / ROS stamp to integer nanoseconds."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def find_pose_for_stamp(
    cache: list[tuple[int, float, float, float]],
    stamp_ns: int,
    *,
    max_skew_ns: int = 0,
) -> tuple[float, float, float] | None:
    """Return (x, y, yaw) for stamp_ns from an odom cache, or None if no match.

    cache entries are (stamp_ns, x, y, yaw), newest last.

    By default ``max_skew_ns=0`` requires an **exact** stamp match so a delayed
    scan cannot latch onto a nearby turn's pose (yaw smear). A positive skew is
    only for callers that explicitly allow tolerance.
    """
    if not cache:
        return None
    if max_skew_ns <= 0:
        for ts, x, y, yaw in reversed(cache):
            if ts == stamp_ns:
                return (x, y, yaw)
        return None
    best: tuple[float, float, float] | None = None
    best_skew = max_skew_ns + 1
    for ts, x, y, yaw in cache:
        skew = abs(ts - stamp_ns)
        if skew == 0:
            return (x, y, yaw)
        if skew < best_skew:
            best_skew = skew
            best = (x, y, yaw)
    if best is None or best_skew > max_skew_ns:
        return None
    return best


def should_integrate_scan(
    *,
    signature: tuple,
    yaw: float,
    last_signature: tuple | None,
    last_yaw: float | None,
    min_yaw_change_rad: float = 0.02,
) -> bool:
    """Skip yaw-only updates that reuse an identical / near-identical depth wedge."""
    if last_signature is None or last_yaw is None:
        return True
    dyaw = abs(yaw - last_yaw)
    while dyaw > math.pi:
        dyaw = abs(dyaw - 2.0 * math.pi)
    if signatures_similar(signature, last_signature):
        # Same FOV at a new heading is the spiral/smear artifact — never integrate.
        return dyaw < min_yaw_change_rad
    return True


@dataclass
class OccupancyMap:
    resolution: float
    initial_size_m: float = 10.0
    data: np.ndarray = field(init=False)
    origin_x: float = field(init=False)
    origin_y: float = field(init=False)

    def __post_init__(self) -> None:
        cells = max(2, int(math.ceil(self.initial_size_m / self.resolution)))
        self.data = np.full((cells, cells), UNKNOWN, dtype=np.int8)
        half = (cells * self.resolution) / 2.0
        self.origin_x = -half
        self.origin_y = -half

    @property
    def height(self) -> int:
        return int(self.data.shape[0])

    @property
    def width(self) -> int:
        return int(self.data.shape[1])

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return row, col

    def ensure_contains(self, x: float, y: float, margin_m: float = 1.0) -> None:
        """Grow the grid so (x, y) plus margin fits inside."""
        min_x = x - margin_m
        max_x = x + margin_m
        min_y = y - margin_m
        max_y = y + margin_m
        while True:
            r0, c0 = self.world_to_cell(min_x, min_y)
            r1, c1 = self.world_to_cell(max_x, max_y)
            if 0 <= r0 < self.height and 0 <= c0 < self.width and 0 <= r1 < self.height and 0 <= c1 < self.width:
                return
            self._expand()

    def _expand(self) -> None:
        """Double extent, keeping world content aligned."""
        old = self.data
        old_h, old_w = old.shape
        new_h, new_w = old_h * 2, old_w * 2
        new = np.full((new_h, new_w), UNKNOWN, dtype=np.int8)
        # Place old block in center of new grid.
        r0 = (new_h - old_h) // 2
        c0 = (new_w - old_w) // 2
        new[r0 : r0 + old_h, c0 : c0 + old_w] = old
        self.origin_x -= c0 * self.resolution
        self.origin_y -= r0 * self.resolution
        self.data = new


def _bresenham(r0: int, c0: int, r1: int, c1: int) -> list[tuple[int, int]]:
    cells: list[tuple[int, int]] = []
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dr - dc
    r, c = r0, c0
    while True:
        cells.append((r, c))
        if r == r1 and c == c1:
            break
        e2 = 2 * err
        if e2 > -dc:
            err -= dc
            r += sr
        if e2 < dr:
            err += dr
            c += sc
    return cells


def integrate_scan(
    grid: OccupancyMap,
    *,
    robot_x: float,
    robot_y: float,
    ranges: list[float] | np.ndarray,
    angles: list[float] | np.ndarray,
    range_min: float,
    range_max: float,
    free_near_max_eps: float = 2.5,
) -> None:
    """Raycast scan into grid. Pose is trusted (T265 / Habitat GT); no scan matching.

    Ranges within ``free_near_max_eps`` of ``range_max`` clear free space only —
    they are treated as sensor clip / no-return, not obstacle hits.

    Free cells along a ray overwrite prior occupied (so corrected clip rays erase
    fake max-range walls from earlier poses).

    The grid is expanded for all ray endpoints *before* raycasting so
    ``world_to_cell`` stays consistent (mid-scan expand used to leave a stale
    robot cell and paint a huge free triangle).
    """
    if len(ranges) != len(angles):
        raise ValueError("ranges and angles length mismatch")

    free_eps = max(float(free_near_max_eps), grid.resolution * 0.5, 1e-3)
    grid.ensure_contains(robot_x, robot_y, margin_m=max(grid.resolution * 4.0, 0.2))

    prepared: list[tuple[float, float, float, float, bool]] = []
    for raw, angle in zip(ranges, angles):
        if not math.isfinite(raw):
            continue
        if raw < range_min:
            continue
        is_hit = raw < (range_max - free_eps)
        use_range = min(float(raw), range_max)
        end_x = robot_x + use_range * math.cos(angle)
        end_y = robot_y + use_range * math.sin(angle)
        grid.ensure_contains(end_x, end_y, margin_m=grid.resolution * 2)
        prepared.append((end_x, end_y, float(angle), use_range, is_hit))

    # Origin is now stable for this scan.
    rr, rc = grid.world_to_cell(robot_x, robot_y)
    for end_x, end_y, _angle, _use_range, is_hit in prepared:
        er, ec = grid.world_to_cell(end_x, end_y)
        cells = _bresenham(rr, rc, er, ec)
        if not cells:
            continue
        last = cells[-1] if is_hit else None
        for cell in cells:
            r, c = cell
            if not (0 <= r < grid.height and 0 <= c < grid.width):
                continue
            if last is not None and (r, c) == last:
                grid.data[r, c] = OCCUPIED
            else:
                grid.data[r, c] = FREE
