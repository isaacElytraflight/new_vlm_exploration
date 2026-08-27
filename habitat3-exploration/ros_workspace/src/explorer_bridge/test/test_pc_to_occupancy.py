"""Unit tests for depth → height-filtered occupancy integration (Goal A)."""

from __future__ import annotations

import pytest

from explorer_bridge.depth_projection import pixel_to_base_link_xyz
from explorer_bridge.pc_to_occupancy import (
    DEFAULT_WALL_HEIGHT_MAX_M,
    DEFAULT_WALL_HEIGHT_MIN_M,
    integrate_depth_frame,
    is_wall_height,
)
from explorer_bridge.scan_to_occupancy import FREE, OCCUPIED, OccupancyMap, UNKNOWN

from long_range_scenes import (
    DEFAULT_CY,
    DEFAULT_FX,
    DEFAULT_FY,
    DEFAULT_CX,
    DEFAULT_RANGE_MAX,
    DEFAULT_RANGE_MIN,
    DEFAULT_SAT_EPS,
    DEFAULT_SENSOR_FAR,
    Intrinsics,
    blank_depth,
    cell_at,
    depth_to_grid,
    occupied_in_annulus,
    paint_floor_band,
    paint_scan_band,
    scene_open_saturated,
    scene_planar_wall,
)


def _integrate_into(grid, depth, *, intrinsics=None, subsample=2):
    K = intrinsics or Intrinsics()
    integrate_depth_frame(
        grid,
        depth,
        robot_x=0.0,
        robot_y=0.0,
        yaw=0.0,
        fx=K.fx,
        fy=K.fy,
        cx=K.cx,
        cy=K.cy,
        range_min=DEFAULT_RANGE_MIN,
        range_max=DEFAULT_RANGE_MAX,
        sensor_far=DEFAULT_SENSOR_FAR,
        sat_eps=DEFAULT_SAT_EPS,
        subsample=subsample,
    )
    return grid


def _integrate(depth, *, intrinsics=None, subsample=2):
    grid = OccupancyMap(resolution=0.05, initial_size_m=24.0)
    return _integrate_into(grid, depth, intrinsics=intrinsics, subsample=subsample)


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_is_wall_height_band_positive():
    assert is_wall_height(0.5)
    assert is_wall_height(DEFAULT_WALL_HEIGHT_MIN_M)
    assert is_wall_height(DEFAULT_WALL_HEIGHT_MAX_M)


def test_is_wall_height_floor_and_tall_negative():
    assert not is_wall_height(0.0)
    assert not is_wall_height(-0.1)
    assert not is_wall_height(1.5)
    assert not is_wall_height(2.5)


def test_wall_at_3m_marks_occupied_and_ray_free_positive():
    grid = _integrate(scene_planar_wall(3.0), subsample=2)
    assert cell_at(grid, 3.0, 0.0) == OCCUPIED
    assert cell_at(grid, 1.5, 0.0) == FREE


def test_floor_only_does_not_paint_phantom_walls_negative():
    depth = scene_open_saturated()
    grid = _integrate(depth, subsample=2)
    assert occupied_in_annulus(grid, r_min=0.5, r_max=8.0) == 0


def test_upper_image_hits_above_robot_height_not_wall_negative():
    """Ceiling / tall clutter (>1 m) must not become occupied walls."""
    depth = blank_depth(fill=float("nan"))
    depth[40:80, 200:440] = 2.5
    grid = _integrate(depth, subsample=2)
    assert occupied_in_annulus(grid, r_min=1.0, r_max=3.0) == 0


def test_mid_height_obstacle_in_wall_band_positive():
    """Synthetic wall row band at ~0.6 m height should mark occupied."""
    depth = blank_depth(fill=float("nan"))
    depth[196:204, 280:360] = 2.0
    xyz = pixel_to_base_link_xyz(
        320, 200, 2.0, fx=DEFAULT_FX, fy=DEFAULT_FY, cx=DEFAULT_CX, cy=DEFAULT_CY
    )
    assert xyz is not None
    assert is_wall_height(xyz[2])
    grid = _integrate(depth, subsample=1)
    assert cell_at(grid, 2.0, 0.0) == OCCUPIED


def test_pc_mapper_beats_laser_on_floor_phantoms_negative():
    """Looking-down floor pixels must not paint walls; laser center-band still does."""
    laser_depth = blank_depth(fill=float("nan"))
    paint_floor_band(laser_depth, value=0.35)
    paint_scan_band(laser_depth, 0.35, anchor="center")
    laser_grid, *_ = depth_to_grid(laser_depth, band_anchor="center")
    assert occupied_in_annulus(laser_grid, r_min=0.2, r_max=0.5) > 0

    floor = blank_depth(fill=float("nan"))
    paint_floor_band(floor, value=0.35)
    pc_grid = _integrate(floor, subsample=2)
    assert occupied_in_annulus(pc_grid, r_min=0.2, r_max=0.5) == 0


def test_beyond_horizon_clears_free_positive():
    depth = blank_depth(fill=15.0)
    paint_scan_band(depth, 15.0)
    grid = _integrate(depth, subsample=2)
    assert cell_at(grid, 5.0, 0.0) == FREE
    assert occupied_in_annulus(grid, r_min=0.5, r_max=10.0) == 0


def test_saturated_open_space_stays_unknown_negative():
    grid = _integrate(scene_open_saturated(), subsample=2)
    assert cell_at(grid, 5.0, 0.0) == UNKNOWN


def test_closer_occupied_survives_farther_ray_positive():
    """Two wall-band hits on the same 2D bearing must both stay OCCUPIED.

    Closer-then-farther is the bad order: the far ray used to paint FREE through
    the nearer wall cell (table underside / near wall + far wall).
    """
    near = blank_depth(fill=float("nan"))
    # Row 180 @ 2 m, col center → ~0.48 m height on +x.
    near[180, 320] = 2.0
    assert is_wall_height(
        pixel_to_base_link_xyz(
            320, 180, 2.0, fx=DEFAULT_FX, fy=DEFAULT_FY, cx=DEFAULT_CX, cy=DEFAULT_CY
        )[2]
    )

    far = blank_depth(fill=float("nan"))
    # Row 220 @ 4 m, same bearing → ~0.35 m height on +x.
    far[220, 320] = 4.0
    assert is_wall_height(
        pixel_to_base_link_xyz(
            320, 220, 4.0, fx=DEFAULT_FX, fy=DEFAULT_FY, cx=DEFAULT_CX, cy=DEFAULT_CY
        )[2]
    )

    grid = OccupancyMap(resolution=0.05, initial_size_m=24.0)
    _integrate_into(grid, near, subsample=1)
    assert cell_at(grid, 2.0, 0.0) == OCCUPIED
    _integrate_into(grid, far, subsample=1)
    assert cell_at(grid, 2.0, 0.0) == OCCUPIED
    assert cell_at(grid, 4.0, 0.0) == OCCUPIED
    assert cell_at(grid, 1.0, 0.0) == FREE

    both = blank_depth(fill=float("nan"))
    both[180, 320] = 2.0
    both[220, 320] = 4.0
    same_frame = _integrate(both, subsample=1)
    assert cell_at(same_frame, 2.0, 0.0) == OCCUPIED
    assert cell_at(same_frame, 4.0, 0.0) == OCCUPIED


def test_far_free_ray_does_not_erase_occupied_negative():
    """A later clear/horizon ray must not wipe a real occupied wall cell."""
    wall = blank_depth(fill=float("nan"))
    wall[200, 320] = 2.0
    grid = _integrate(wall, subsample=1)
    assert cell_at(grid, 2.0, 0.0) == OCCUPIED

    clear = blank_depth(fill=float("nan"))
    clear[200, 320] = 15.0  # beyond horizon → free carve only
    _integrate_into(grid, clear, subsample=1)
    assert cell_at(grid, 2.0, 0.0) == OCCUPIED
