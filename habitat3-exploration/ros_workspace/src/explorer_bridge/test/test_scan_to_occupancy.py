"""Unit tests for known-pose LaserScan → OccupancyGrid integration."""

from __future__ import annotations

import math

import numpy as np
import pytest

from explorer_bridge.scan_to_occupancy import (
    OccupancyMap,
    integrate_scan,
    scan_content_signature,
    should_integrate_scan,
)


def test_hit_marks_occupied_and_ray_free_positive():
    grid = OccupancyMap(resolution=0.05, initial_size_m=4.0)
    # Robot at origin facing +x; single hit at 1.0 m forward.
    ranges = [1.0]
    angles = [0.0]
    integrate_scan(
        grid,
        robot_x=0.0,
        robot_y=0.0,
        ranges=ranges,
        angles=angles,
        range_min=0.1,
        range_max=5.0,
    )
    # Endpoint ~ (1, 0) should be occupied.
    r, c = grid.world_to_cell(1.0, 0.0)
    assert grid.data[r, c] == 100
    # Mid-ray free.
    r2, c2 = grid.world_to_cell(0.5, 0.0)
    assert grid.data[r2, c2] == 0


def test_max_range_clears_without_occupied_positive():
    grid = OccupancyMap(resolution=0.05, initial_size_m=12.0)
    integrate_scan(
        grid,
        robot_x=0.0,
        robot_y=0.0,
        ranges=[5.0],
        angles=[0.0],
        range_min=0.1,
        range_max=5.0,
    )
    r, c = grid.world_to_cell(1.0, 0.0)
    assert grid.data[r, c] == 0
    r_end, c_end = grid.world_to_cell(5.0, 0.0)
    assert grid.data[r_end, c_end] == 0
    # Off-axis stays unknown.
    r2, c2 = grid.world_to_cell(1.0, 1.0)
    assert grid.data[r2, c2] == -1


def test_near_clip_clears_without_occupied_positive():
    """Saturated depth just below range_max must not paint a fake wall."""
    grid = OccupancyMap(resolution=0.05, initial_size_m=24.0)
    integrate_scan(
        grid,
        robot_x=0.0,
        robot_y=0.0,
        ranges=[8.0],
        angles=[0.0],
        range_min=0.1,
        range_max=10.0,
        free_near_max_eps=2.5,
    )
    r_end, c_end = grid.world_to_cell(8.0, 0.0)
    assert grid.data[r_end, c_end] == 0
    r_mid, c_mid = grid.world_to_cell(5.0, 0.0)
    assert grid.data[r_mid, c_mid] == 0


def test_real_hit_inside_clip_still_occupied_negative():
    """Negative control: a real obstacle well inside range_max stays occupied."""
    grid = OccupancyMap(resolution=0.05, initial_size_m=12.0)
    integrate_scan(
        grid,
        robot_x=0.0,
        robot_y=0.0,
        ranges=[4.0],
        angles=[0.0],
        range_min=0.1,
        range_max=10.0,
        free_near_max_eps=2.5,
    )
    r, c = grid.world_to_cell(4.0, 0.0)
    assert grid.data[r, c] == 100


def test_free_ray_clears_stale_occupied_positive():
    """A later free (clip) ray must erase a prior fake max-range wall."""
    grid = OccupancyMap(resolution=0.05, initial_size_m=24.0)
    integrate_scan(
        grid,
        robot_x=0.0,
        robot_y=0.0,
        ranges=[4.0],
        angles=[0.0],
        range_min=0.1,
        range_max=10.0,
        free_near_max_eps=2.5,
    )
    r, c = grid.world_to_cell(4.0, 0.0)
    assert grid.data[r, c] == 100
    integrate_scan(
        grid,
        robot_x=0.0,
        robot_y=0.0,
        ranges=[10.0],
        angles=[0.0],
        range_min=0.1,
        range_max=10.0,
        free_near_max_eps=2.5,
    )
    assert grid.data[r, c] == 0


def test_expand_mid_scan_does_not_paint_corner_triangle_positive():
    """Regression: expanding the grid mid-scan must not free a map-wide diagonal."""
    grid = OccupancyMap(resolution=0.05, initial_size_m=2.0)
    # Short hit keeps grid small, then a long free ray forces expand.
    integrate_scan(
        grid,
        robot_x=0.0,
        robot_y=0.0,
        ranges=[0.5, 10.0],
        angles=[0.0, math.pi / 4],
        range_min=0.1,
        range_max=10.0,
        free_near_max_eps=2.5,
    )
    # Far corner of the expanded map (away from the FOV wedge) must stay unknown.
    r_far, c_far = grid.world_to_cell(grid.origin_x + grid.resolution, grid.origin_y + grid.resolution)
    assert grid.data[r_far, c_far] == -1


def test_nan_ranges_ignored_negative():
    grid = OccupancyMap(resolution=0.05, initial_size_m=2.0)
    before = grid.data.copy()
    integrate_scan(
        grid,
        robot_x=0.0,
        robot_y=0.0,
        ranges=[float("nan"), float("inf")],
        angles=[0.0, math.pi / 2],
        range_min=0.1,
        range_max=5.0,
    )
    assert np.array_equal(grid.data, before)


def test_rotate_in_place_fills_new_bearings_positive():
    """Privileged pose + FOV scans must accumulate during pure rotation."""
    grid = OccupancyMap(resolution=0.05, initial_size_m=6.0)
    # First heading: wall at +x
    integrate_scan(
        grid,
        robot_x=0.0,
        robot_y=0.0,
        ranges=[1.5],
        angles=[0.0],
        range_min=0.1,
        range_max=5.0,
    )
    # Second heading: wall at +y (90° turn)
    integrate_scan(
        grid,
        robot_x=0.0,
        robot_y=0.0,
        ranges=[1.5],
        angles=[math.pi / 2],
        range_min=0.1,
        range_max=5.0,
    )
    assert grid.data[grid.world_to_cell(1.5, 0.0)] == 100
    assert grid.data[grid.world_to_cell(0.0, 1.5)] == 100


def test_skip_identical_scan_when_yaw_changes_negative():
    """Negative control: same wedge + rotating yaw must not integrate (spiral)."""
    sig = scan_content_signature([1.0, 1.1, 1.2])
    assert not should_integrate_scan(
        signature=sig,
        yaw=1.0,
        last_signature=sig,
        last_yaw=0.0,
    )


def test_skip_near_identical_scan_when_yaw_changes_negative():
    """Near-duplicate FOV after a turn must not smear across headings."""
    from explorer_bridge.scan_to_occupancy import signatures_similar

    a = scan_content_signature([1.00, 1.10, 1.20, 2.00, 2.10])
    b = scan_content_signature([1.01, 1.11, 1.19, 2.01, 2.09])
    assert signatures_similar(a, b)
    assert not should_integrate_scan(
        signature=b,
        yaw=1.0,
        last_signature=a,
        last_yaw=0.0,
    )


def test_integrate_when_scan_content_changes_positive():
    assert should_integrate_scan(
        signature=scan_content_signature([2.0, 2.1, 5.0, 5.1]),
        yaw=1.0,
        last_signature=scan_content_signature([1.0, 1.1, 1.2, 1.3]),
        last_yaw=0.0,
    )


def test_stamp_tf_required_for_integrate_positive():
    from explorer_bridge.scan_to_occupancy import should_integrate_with_tf

    assert should_integrate_with_tf(stamp_lookup_ok=True) is True


def test_stale_latest_tf_fallback_rejected_negative():
    """Negative control: stamp miss must not integrate (would paint at old yaw)."""
    from explorer_bridge.scan_to_occupancy import should_integrate_with_tf

    assert should_integrate_with_tf(stamp_lookup_ok=False) is False


def test_find_pose_exact_stamp_positive():
    from explorer_bridge.scan_to_occupancy import find_pose_for_stamp

    cache = [(100, 0.0, 0.0, 0.0), (200, 1.0, 2.0, 0.5)]
    assert find_pose_for_stamp(cache, 200) == (1.0, 2.0, 0.5)


def test_find_pose_default_rejects_near_miss_negative():
    """Default is exact match only — nearby odom must not claim a delayed scan."""
    from explorer_bridge.scan_to_occupancy import find_pose_for_stamp

    cache = [(1_000_000_000, 3.0, 4.0, 1.2)]
    # 20 ms later: would have matched under 50 ms skew; must fail with exact-only.
    assert find_pose_for_stamp(cache, 1_020_000_000) is None


def test_find_pose_explicit_skew_still_allowed_positive():
    from explorer_bridge.scan_to_occupancy import find_pose_for_stamp

    cache = [(1_000_000_000, 3.0, 4.0, 1.2)]
    got = find_pose_for_stamp(cache, 1_020_000_000, max_skew_ns=50_000_000)
    assert got == (3.0, 4.0, 1.2)


def test_find_pose_skew_too_large_negative():
    from explorer_bridge.scan_to_occupancy import find_pose_for_stamp

    cache = [(1_000_000_000, 0.0, 0.0, 0.0)]
    assert find_pose_for_stamp(cache, 1_200_000_000, max_skew_ns=50_000_000) is None


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
