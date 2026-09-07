"""Pure metric helpers for experiment runs (no ROS / Habitat deps)."""

from __future__ import annotations

import math
from collections import Counter
from typing import Dict, Iterable, List, Sequence, Tuple


def integrate_trajectory_meters(
    trajectory: Sequence[Tuple[float, float]],
) -> float:
    """Total planar path length from (x, y) samples."""
    if len(trajectory) < 2:
        return 0.0
    total = 0.0
    prev = trajectory[0]
    for point in trajectory[1:]:
        total += math.hypot(point[0] - prev[0], point[1] - prev[1])
        prev = point
    return float(total)


def compute_revisit_bins(
    trajectory: Sequence[Tuple[float, float]],
    *,
    cell_size_m: float = 0.25,
) -> Dict[int, int]:
    """Histogram: revisit_count -> number of grid cells visited that many times."""
    if cell_size_m <= 0.0:
        raise ValueError("cell_size_m must be positive")
    if not trajectory:
        return {}

    visit_counts: Counter[Tuple[int, int]] = Counter()
    inv = 1.0 / float(cell_size_m)
    for x, y in trajectory:
        cx = int(round(float(x) * inv))
        cy = int(round(float(y) * inv))
        visit_counts[(cx, cy)] += 1

    bins: Counter[int] = Counter()
    for count in visit_counts.values():
        bins[int(count)] += 1
    return dict(sorted(bins.items()))


def coverage_ratio(mapped_m2: float, gt_m2: float) -> float:
    if float(gt_m2) <= 0.0:
        return 0.0
    return max(0.0, min(1.0, float(mapped_m2) / float(gt_m2)))


def summarize_coverage_samples(
    samples: Iterable[Sequence[float]],
) -> Tuple[float, float]:
    """Return (final_distance_m, final_coverage) from (t_s, distance_m, coverage) rows."""
    last: Tuple[float, float] | None = None
    for row in samples:
        if len(row) < 3:
            continue
        last = (float(row[1]), float(row[2]))
    if last is None:
        return 0.0, 0.0
    return last
