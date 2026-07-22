"""Depth → scan → grid: upper-third band + range classification.

The production fix for phantom mid-range walls is ``band_anchor=upper_third``
(floor hits from a low camera). Keep sensor_far >> range_max so saturation
does not look like a room-scale wall.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from explorer_bridge.scan_from_depth import normalize_range
from explorer_bridge.scan_to_occupancy import FREE, OCCUPIED, UNKNOWN

from long_range_scenes import (
    DEFAULT_RANGE_MAX,
    DEFAULT_SAT_M,
    DEFAULT_SENSOR_FAR,
    blank_depth,
    cell_at,
    depth_to_grid,
    occupied_in_annulus,
    paint_scan_band,
    scene_constant_plane,
    scene_open_saturated,
    scene_planar_wall,
)


def test_harness_positive():
    assert 1 + 1 == 2


def test_harness_negative():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_real_wall_at_3m_occupied_positive():
    grid, ranges, *_ = depth_to_grid(scene_planar_wall(3.0))
    hits = [r for r in ranges if math.isfinite(r) and r < 7.0]
    assert hits and min(hits) == pytest.approx(3.0, abs=0.15)
    assert cell_at(grid, 3.0, 0.0) == OCCUPIED
    assert cell_at(grid, 1.5, 0.0) == FREE


def test_open_saturated_no_midrange_wall_negative():
    grid, ranges, *_ = depth_to_grid(scene_open_saturated())
    assert occupied_in_annulus(grid, r_min=2.5, r_max=3.5) == 0
    assert all(not math.isfinite(r) for r in ranges)
    assert cell_at(grid, 5.0, 0.0) == UNKNOWN


def test_wall_at_8m_occupied_positive():
    grid, ranges, *_ = depth_to_grid(scene_planar_wall(8.0))
    assert any(math.isfinite(r) and 7.5 <= r <= 8.5 for r in ranges)
    assert cell_at(grid, 8.0, 0.0) == OCCUPIED


def test_beyond_horizon_clears_free_positive():
    depth = blank_depth(fill=15.0)
    paint_scan_band(depth, 15.0)
    grid, ranges, *_ = depth_to_grid(depth)
    assert all(r == pytest.approx(10.0) for r in ranges if math.isfinite(r))
    assert cell_at(grid, 5.0, 0.0) == FREE
    assert occupied_in_annulus(grid, r_min=0.5, r_max=10.0) == 0


def test_normalize_sat_nan_and_wall_kept_positive():
    assert math.isnan(
        normalize_range(DEFAULT_SAT_M, range_min=0.1, clear_range=10.0, sensor_far=DEFAULT_SENSOR_FAR)
    )
    assert normalize_range(8.0, range_min=0.1, clear_range=10.0, sensor_far=DEFAULT_SENSOR_FAR) == 8.0
    assert (
        normalize_range(15.0, range_min=0.1, clear_range=10.0, sensor_far=DEFAULT_SENSOR_FAR)
        == DEFAULT_RANGE_MAX
    )


def test_constant_plane_maps_to_linear_front_positive():
    grid, ranges, *_ = depth_to_grid(scene_constant_plane(3.0))
    finite = [r for r in ranges if math.isfinite(r)]
    assert len(finite) > 100
    assert min(finite) == pytest.approx(3.0, abs=0.05)
    occ_line = sum(1 for y in (-1.0, -0.5, 0.0, 0.5, 1.0) if cell_at(grid, 3.0, y) == OCCUPIED)
    assert occ_line >= 3


def test_depth_shape_contract_positive():
    depth = scene_planar_wall(3.0)
    assert depth.shape == (480, 640) and depth.dtype == np.float32
