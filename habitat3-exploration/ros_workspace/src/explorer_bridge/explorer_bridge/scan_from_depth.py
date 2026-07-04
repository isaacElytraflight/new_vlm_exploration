"""Pure depth-image → LaserScan range conversion (testable without ROS)."""

from __future__ import annotations

import math
from typing import Literal, Sequence

import numpy as np

BandAnchor = Literal["bottom", "center"]


def row_band_slice(height: int, scan_height: int, *, anchor: BandAnchor = "bottom") -> slice:
    """Select depth rows for 2-D SLAM (floor band for low-mounted cameras)."""
    band = max(1, scan_height)
    if anchor == "bottom":
        return slice(height - band, height)
    half = max(1, band // 2)
    cy = height // 2
    return slice(cy - half, cy + half)


def center_band_row_slice(height: int, scan_height: int) -> slice:
    return row_band_slice(height, scan_height, anchor="center")


def _pixel_to_base_link_xy(
    col: int,
    row: int,
    depth: float,
    *,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> tuple[float, float] | None:
    """Project a depth pixel to base_link (x forward, y left) on the ground plane."""
    if not math.isfinite(depth) or depth <= 0.0:
        return None
    # Habitat depth = distance along the optical +Z axis (meters).
    z_c = depth
    x_c = (col - cx) / fx * z_c
    y_c = (row - cy) / fy * z_c
    # Optical frame: +Z forward, +X right, +Y down; base_link: +X forward, +Y left.
    x_bl = z_c
    y_bl = -x_c
    if x_bl <= 0.01:
        return None
    return x_bl, y_bl


def normalize_range(
    raw: float,
    *,
    range_min: float,
    clear_range: float,
) -> float:
    """Clamp invalid / long hits to clear_range so SLAM marks free space, not unknown."""
    if not math.isfinite(raw) or raw < range_min:
        return clear_range
    if raw > clear_range:
        return clear_range
    return raw


def depth_to_laserscan_bins(
    depth: np.ndarray,
    *,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    range_min: float,
    clear_range: float,
    scan_height: int = 24,
    band_anchor: BandAnchor = "bottom",
    full_360: bool = True,
    num_bins: int = 360,
) -> tuple[list[float], float, float, float]:
    """Build a LaserScan range array in base_link (0 rad = forward, + = left)."""
    if depth.ndim != 2:
        raise ValueError("depth must be 2-D")
    height, width = depth.shape
    band_slice = row_band_slice(height, scan_height, anchor=band_anchor)
    band = depth[band_slice, :]
    band_rows = np.arange(band_slice.start or 0, band_slice.stop or height)

    if full_360:
        angle_min = -math.pi
        angle_max = math.pi
        angle_increment = (angle_max - angle_min) / float(num_bins)
        bins: list[list[float]] = [[] for _ in range(num_bins)]
    else:
        angle_min = -math.atan2(cx, fx)
        angle_max = math.atan2(width - cx, fx)
        num_bins = max(1, width)
        angle_increment = (angle_max - angle_min) / float(max(1, num_bins - 1))
        bins = [[] for _ in range(num_bins)]

    for col in range(width):
        column = band[:, col]
        valid_mask = np.isfinite(column) & (column > 0.0)
        if not np.any(valid_mask):
            continue
        row_idx = int(np.argmin(np.where(valid_mask, column, np.inf)))
        raw_depth = float(column[row_idx])
        row = int(band_rows[row_idx])
        xy = _pixel_to_base_link_xy(
            col, row, raw_depth, fx=fx, fy=fy, cx=cx, cy=cy
        )
        if xy is None:
            continue
        x_bl, y_bl = xy
        bearing = math.atan2(y_bl, x_bl)
        horiz_range = math.hypot(x_bl, y_bl)
        normalized = normalize_range(
            horiz_range, range_min=range_min, clear_range=clear_range
        )
        if full_360:
            idx = int(round((bearing - angle_min) / angle_increment)) % num_bins
        else:
            idx = int(round((bearing - angle_min) / angle_increment))
            idx = max(0, min(num_bins - 1, idx))
        bins[idx].append(normalized)

    ranges: list[float] = []
    for cell in bins:
        if cell:
            ranges.append(min(cell))
        else:
            # No sensor coverage for this bearing — assume open floor to clear_range.
            ranges.append(clear_range)

    return ranges, angle_min, angle_max, angle_increment


def column_ranges_from_depth(
    depth: np.ndarray,
    *,
    fx: float,
    cx: float,
    range_min: float,
    range_max: float,
    scan_height: int = 50,
    **kwargs,
) -> tuple[list[float], float, float, float]:
    """Backward-compatible wrapper (uses clear_range = range_max)."""
    return depth_to_laserscan_bins(
        depth,
        fx=fx,
        fy=fx,
        cx=cx,
        cy=depth.shape[0] / 2.0,
        range_min=range_min,
        clear_range=range_max,
        scan_height=scan_height,
        full_360=kwargs.get("full_360", False),
        band_anchor=kwargs.get("band_anchor", "center"),
    )


def finite_fraction(ranges: Sequence[float]) -> float:
    total = len(ranges)
    if total == 0:
        return 0.0
    finite = sum(1 for r in ranges if math.isfinite(r) and r > 0.0)
    return finite / float(total)
