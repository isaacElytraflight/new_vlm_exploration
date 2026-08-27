"""Known-pose occupancy mapping from depth frames (point-cloud style, no SLAM)."""

from __future__ import annotations

import math

import numpy as np

from explorer_bridge.depth_projection import (
    DEFAULT_CAMERA_Z_M,
    base_link_xy_to_map,
    pixel_to_base_link_xyz,
)
from explorer_bridge.scan_from_depth import DEFAULT_SAT_EPS_M, DEFAULT_SENSOR_FAR_M, normalize_range
from explorer_bridge.scan_to_occupancy import OccupancyMap, _bresenham, OCCUPIED, FREE

# Wall band: above floor, up to robot body height (~1 m).
DEFAULT_WALL_HEIGHT_MIN_M = 0.05
DEFAULT_WALL_HEIGHT_MAX_M = 1.0


def is_wall_height(
    z_m: float,
    *,
    min_z: float = DEFAULT_WALL_HEIGHT_MIN_M,
    max_z: float = DEFAULT_WALL_HEIGHT_MAX_M,
) -> bool:
    """True when a map/base z is in the navigable obstacle band (not floor/ceiling/tall)."""
    return min_z <= z_m <= max_z


def depth_content_signature(depth: np.ndarray, *, stride: int = 16, decimals: int = 2) -> tuple:
    """Fingerprint a depth frame for duplicate-view detection."""
    if depth.ndim != 2:
        return (0, ())
    step = max(1, int(stride))
    sample = depth[::step, ::step].astype(float).ravel()
    finite = sample[np.isfinite(sample) & (sample > 0.0)]
    if finite.size == 0:
        return (0, ())
    q = tuple(round(float(v), decimals) for v in finite[:: max(1, finite.size // 32)])
    return (
        int(finite.size),
        round(float(finite.min()), decimals),
        round(float(finite.max()), decimals),
        round(float(finite.mean()), decimals),
        q,
    )


def integrate_depth_frame(
    grid: OccupancyMap,
    depth: np.ndarray,
    *,
    robot_x: float,
    robot_y: float,
    yaw: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    range_min: float,
    range_max: float,
    camera_z: float = DEFAULT_CAMERA_Z_M,
    sensor_far: float = DEFAULT_SENSOR_FAR_M,
    sat_eps: float = DEFAULT_SAT_EPS_M,
    wall_height_min: float = DEFAULT_WALL_HEIGHT_MIN_M,
    wall_height_max: float = DEFAULT_WALL_HEIGHT_MAX_M,
    subsample: int = 4,
) -> None:
    """Ray-carve free space from depth; mark occupied only in the wall height band.

    All valid depth returns carve free space along the horizontal ray. Endpoints
    become OCCUPIED only when the hit z is above the floor and at/below robot
    height — floor, ceiling, and tall clutter above ~1 m do not paint walls.

    FREE never overwrites OCCUPIED. Two wall-band hits on the same 2D bearing
    (near table + far wall) must both stay occupied even if the farther ray is
    integrated later.
    """
    if depth.ndim != 2:
        raise ValueError("depth must be 2-D")

    free_eps = max(grid.resolution * 0.5, 1e-3)
    step = max(1, int(subsample))
    height, width = depth.shape

    grid.ensure_contains(robot_x, robot_y, margin_m=max(grid.resolution * 4.0, 0.2))
    rr, rc = grid.world_to_cell(robot_x, robot_y)

    prepared: list[tuple[float, float, bool]] = []
    for row in range(0, height, step):
        for col in range(0, width, step):
            raw_depth = float(depth[row, col])
            xyz = pixel_to_base_link_xyz(
                col,
                row,
                raw_depth,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                camera_z=camera_z,
            )
            if xyz is None:
                continue
            x_bl, y_bl, z_bl = xyz
            horiz = math.hypot(x_bl, y_bl)
            classified = normalize_range(
                horiz,
                range_min=range_min,
                clear_range=range_max,
                sensor_far=sensor_far,
                sat_eps=sat_eps,
            )
            if not math.isfinite(classified):
                continue
            is_hit = classified < (range_max - free_eps)
            use_range = min(classified, range_max)
            scale = use_range / horiz if horiz > 1e-6 else 0.0
            end_x_bl = x_bl * scale
            end_y_bl = y_bl * scale
            end_z_bl = z_bl * scale
            end_x, end_y = base_link_xy_to_map(
                end_x_bl,
                end_y_bl,
                robot_x=robot_x,
                robot_y=robot_y,
                yaw=yaw,
            )
            grid.ensure_contains(end_x, end_y, margin_m=grid.resolution * 2)
            mark_occ = is_hit and is_wall_height(
                end_z_bl,
                min_z=wall_height_min,
                max_z=wall_height_max,
            )
            prepared.append((end_x, end_y, mark_occ))

    # Occupied endpoints first, then free carve, so a farther ray cannot paint
    # FREE through a nearer wall-band hit (same frame or a later frame).
    for end_x, end_y, mark_occ in prepared:
        if not mark_occ:
            continue
        er, ec = grid.world_to_cell(end_x, end_y)
        if 0 <= er < grid.height and 0 <= ec < grid.width:
            grid.data[er, ec] = OCCUPIED

    for end_x, end_y, mark_occ in prepared:
        er, ec = grid.world_to_cell(end_x, end_y)
        cells = _bresenham(rr, rc, er, ec)
        if not cells:
            continue
        last = cells[-1] if mark_occ else None
        for cell in cells:
            r, c = cell
            if not (0 <= r < grid.height and 0 <= c < grid.width):
                continue
            if last is not None and (r, c) == last:
                grid.data[r, c] = OCCUPIED
            elif grid.data[r, c] != OCCUPIED:
                grid.data[r, c] = FREE
