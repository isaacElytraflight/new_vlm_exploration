"""Pure depth-image → LaserScan range conversion (testable without ROS)."""

from __future__ import annotations

import math
from typing import Literal, Sequence

import numpy as np

BandAnchor = Literal["bottom", "center", "upper_third"]

# Habitat depth.far — keep well above room-scale walls so voids saturate near
# ``far``, not near the mapping horizon (``clear_range`` / range_max).
DEFAULT_SENSOR_FAR_M = 50.0
DEFAULT_SAT_EPS_M = 0.5


def row_band_slice(height: int, scan_height: int, *, anchor: BandAnchor = "upper_third") -> slice:
    """Select depth rows for 2-D occupancy rays.

    Production default is ``upper_third``: a low camera (~0.1 m) sees the floor
    in the center/bottom of the image; sampling there paints phantom walls.
    A band centered near row ``H/3`` looks at wall geometry above the floor.
    """
    band = max(1, scan_height)
    half = max(1, band // 2)
    if anchor == "bottom":
        return slice(height - band, height)
    if anchor == "upper_third":
        mid = max(half, min(height - half, height // 3))
        return slice(mid - half, mid + half)
    mid = height // 2
    return slice(mid - half, mid + half)


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
    """Project a depth pixel to base_link (x forward, y left).

    Habitat depth is camera-frame **Z** (optical-axis depth), not Euclidean ray
    length. Pinhole::

        P_cam = ((u-cx)/fx * Z, (v-cy)/fy * Z, Z)
        x_base, y_base = Z, -P_cam.x

    ``row`` must be the real pixel row (upper-third band looks upward).
    """
    if not math.isfinite(depth) or depth <= 0.0:
        return None
    z_c = depth
    x_c = (col - cx) / fx * z_c
    x_bl = z_c
    y_bl = -x_c
    if x_bl <= 0.01:
        return None
    return x_bl, y_bl


def pixel_elevation_rad(row: float, *, fy: float, cy: float) -> float:
    """Elevation above the optical axis (radians, + = look up)."""
    return math.atan(-(row - cy) / fy)


def normalize_range(
    raw: float,
    *,
    range_min: float,
    clear_range: float,
    sensor_far: float = DEFAULT_SENSOR_FAR_M,
    sat_eps: float = DEFAULT_SAT_EPS_M,
) -> float:
    """Classify a horizontal range for LaserScan publishing.

    - finite hit — obstacle range
    - ``clear_range`` — past mapping horizon but below sensor far (free ray)
    - NaN — invalid or near ``sensor_far`` saturation (mapper leaves UNKNOWN)
    """
    if not math.isfinite(raw):
        return float("nan")
    if raw < range_min:
        return clear_range
    far = max(float(sensor_far), float(clear_range))
    if raw >= far - max(0.0, float(sat_eps)):
        return float("nan")
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
    band_anchor: BandAnchor = "upper_third",
    full_360: bool = False,
    num_bins: int = 360,
    sensor_far: float = DEFAULT_SENSOR_FAR_M,
    sat_eps: float = DEFAULT_SAT_EPS_M,
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
            horiz_range,
            range_min=range_min,
            clear_range=clear_range,
            sensor_far=sensor_far,
            sat_eps=sat_eps,
        )
        if not math.isfinite(normalized):
            continue
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
            # No coverage / only saturated — UNKNOWN for the mapper (not free).
            ranges.append(float("nan"))

    return ranges, angle_min, angle_max, angle_increment


def finite_fraction(ranges: Sequence[float]) -> float:
    total = len(ranges)
    if total == 0:
        return 0.0
    finite = sum(1 for r in ranges if math.isfinite(r) and r > 0.0)
    return finite / float(total)
