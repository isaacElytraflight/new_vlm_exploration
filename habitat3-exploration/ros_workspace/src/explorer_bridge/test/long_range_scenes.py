"""Controlled synthetic depth scenes for mapping tests."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from explorer_bridge.scan_from_depth import depth_to_laserscan_bins, row_band_slice
from explorer_bridge.scan_to_occupancy import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    OccupancyMap,
    integrate_scan,
)

DEFAULT_H = 480
DEFAULT_W = 640
DEFAULT_FX = 320.0
DEFAULT_FY = 320.0
DEFAULT_CX = 320.0
DEFAULT_CY = 240.0
DEFAULT_RANGE_MIN = 0.1
DEFAULT_RANGE_MAX = 10.0
DEFAULT_SENSOR_FAR = 50.0
DEFAULT_SAT_EPS = 0.5
DEFAULT_SCAN_HEIGHT = 24
DEFAULT_SAT_M = 49.7
DEFAULT_BAND = "upper_third"


@dataclass(frozen=True)
class Intrinsics:
    height: int = DEFAULT_H
    width: int = DEFAULT_W
    fx: float = DEFAULT_FX
    fy: float = DEFAULT_FY
    cx: float = DEFAULT_CX
    cy: float = DEFAULT_CY


def blank_depth(*, fill: float = float("nan"), intrinsics: Intrinsics | None = None) -> np.ndarray:
    K = intrinsics or Intrinsics()
    return np.full((K.height, K.width), fill, dtype=np.float32)


def paint_scan_band(
    depth: np.ndarray,
    value: float,
    *,
    scan_height: int = DEFAULT_SCAN_HEIGHT,
    col_start: int = 0,
    col_end: int | None = None,
    anchor: str = DEFAULT_BAND,
) -> None:
    h, w = depth.shape
    band = row_band_slice(h, scan_height, anchor=anchor)  # type: ignore[arg-type]
    c2 = w if col_end is None else min(w, col_end)
    depth[band, max(0, col_start):c2] = np.float32(value)


def paint_floor_band(depth: np.ndarray, value: float = 0.27, *, rows: int = 24) -> None:
    h, _ = depth.shape
    depth[h - rows : h, :] = np.float32(value)


def scene_open_saturated(*, sat_m: float = DEFAULT_SAT_M, intrinsics: Intrinsics | None = None) -> np.ndarray:
    depth = blank_depth(fill=sat_m, intrinsics=intrinsics)
    paint_floor_band(depth)
    return depth


def scene_planar_wall(
    wall_m: float,
    *,
    sat_m: float = DEFAULT_SAT_M,
    col_start: int = 80,
    col_end: int = 560,
    intrinsics: Intrinsics | None = None,
) -> np.ndarray:
    depth = scene_open_saturated(sat_m=sat_m, intrinsics=intrinsics)
    paint_scan_band(depth, wall_m, col_start=col_start, col_end=col_end)
    return depth


def scene_constant_plane(plane_m: float, *, intrinsics: Intrinsics | None = None) -> np.ndarray:
    depth = blank_depth(fill=float("nan"), intrinsics=intrinsics)
    paint_scan_band(depth, plane_m)
    return depth


def depth_to_grid(
    depth: np.ndarray,
    *,
    sensor_far: float = DEFAULT_SENSOR_FAR,
    sat_eps: float = DEFAULT_SAT_EPS,
    range_min: float = DEFAULT_RANGE_MIN,
    range_max: float = DEFAULT_RANGE_MAX,
    resolution: float = 0.05,
    initial_size_m: float = 24.0,
    robot_x: float = 0.0,
    robot_y: float = 0.0,
    yaw: float = 0.0,
    band_anchor: str = DEFAULT_BAND,
    intrinsics: Intrinsics | None = None,
) -> tuple[OccupancyMap, list[float], float, float, float]:
    K = intrinsics or Intrinsics()
    ranges, angle_min, angle_max, angle_increment = depth_to_laserscan_bins(
        depth,
        fx=K.fx,
        fy=K.fy,
        cx=K.cx,
        cy=K.cy,
        range_min=range_min,
        clear_range=range_max,
        scan_height=DEFAULT_SCAN_HEIGHT,
        band_anchor=band_anchor,  # type: ignore[arg-type]
        full_360=False,
        sensor_far=sensor_far,
        sat_eps=sat_eps,
    )
    angles = [yaw + angle_min + i * angle_increment for i in range(len(ranges))]
    grid = OccupancyMap(resolution=resolution, initial_size_m=initial_size_m)
    integrate_scan(
        grid,
        robot_x=robot_x,
        robot_y=robot_y,
        ranges=ranges,
        angles=angles,
        range_min=range_min,
        range_max=range_max,
    )
    return grid, ranges, angle_min, angle_max, angle_increment


def cell_at(grid: OccupancyMap, x: float, y: float) -> int:
    r, c = grid.world_to_cell(x, y)
    if not (0 <= r < grid.height and 0 <= c < grid.width):
        return UNKNOWN
    return int(grid.data[r, c])


def occupied_in_annulus(
    grid: OccupancyMap,
    *,
    robot_x: float = 0.0,
    robot_y: float = 0.0,
    r_min: float,
    r_max: float,
) -> int:
    count = 0
    for r in range(grid.height):
        for c in range(grid.width):
            if int(grid.data[r, c]) != OCCUPIED:
                continue
            x = grid.origin_x + (c + 0.5) * grid.resolution
            y = grid.origin_y + (r + 0.5) * grid.resolution
            dist = math.hypot(x - robot_x, y - robot_y)
            if r_min <= dist <= r_max:
                count += 1
    return count
