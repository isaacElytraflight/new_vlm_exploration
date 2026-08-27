"""Unit tests for coverage-vs-distance pure metrics helpers."""

from __future__ import annotations

import numpy as np
import pytest

from explorer_mission.coverage_metrics import (
    CoverageSampleRing,
    adjacent_wall_mask,
    coverage_ratio,
    floor_area_m2_from_navigable,
    free_area_m2,
    gt_mappable_area_m2,
    integrate_path_meters,
    mapped_area_m2,
    render_coverage_chart_jpeg,
)


def test_harness_positive_control():
    assert 1 + 1 == 2


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_path_integrate_hypotenuse_positive():
    meters, prev = integrate_path_meters(0.0, None, 0.0, 0.0)
    assert meters == pytest.approx(0.0)
    meters, prev = integrate_path_meters(meters, prev, 3.0, 4.0)
    assert meters == pytest.approx(5.0)
    assert prev == (3.0, 4.0)
    meters2, prev2 = integrate_path_meters(meters, prev, 3.0, 4.0)
    assert meters2 == pytest.approx(5.0)
    meters3, _ = integrate_path_meters(meters2, prev2, 6.0, 8.0)
    assert meters3 == pytest.approx(10.0)


def test_path_integrate_zero_motion_negative():
    """Yaw-only / stationary odom must not inflate meters traveled."""
    meters, prev = integrate_path_meters(0.0, None, 1.0, 2.0)
    meters2, _ = integrate_path_meters(meters, prev, 1.0, 2.0)
    assert meters2 == pytest.approx(0.0)


def test_free_area_counts_free_cells_positive():
    grid = np.array([[0, 0, 100], [-1, 0, -1]], dtype=np.int8)
    area = free_area_m2(grid, resolution=0.5)
    assert area == pytest.approx(0.75)


def test_mapped_area_includes_walls_positive():
    """Free + occupied count as mapped; unknown does not."""
    grid = np.array([[0, 0, 100], [-1, 0, -1]], dtype=np.int8)
    # 3 free + 1 occupied = 4 cells * 0.25
    assert mapped_area_m2(grid, resolution=0.5) == pytest.approx(1.0)


def test_mapped_area_stable_when_free_becomes_wall_positive():
    """Reclassifying free→occupied must not shrink mapped area (coverage num)."""
    before = np.array([[0, 0, -1], [-1, 0, -1]], dtype=np.int8)
    after = np.array([[0, 100, -1], [-1, 0, -1]], dtype=np.int8)
    res = 0.1
    assert mapped_area_m2(after, res) == pytest.approx(mapped_area_m2(before, res))


def test_mapped_area_unknown_only_negative():
    empty = np.zeros((0, 0), dtype=np.int8)
    assert mapped_area_m2(empty, resolution=0.05) == pytest.approx(0.0)
    unknown = np.full((4, 4), -1, dtype=np.int8)
    assert mapped_area_m2(unknown, resolution=0.05) == pytest.approx(0.0)


def test_free_area_empty_or_unknown_only_negative():
    empty = np.zeros((0, 0), dtype=np.int8)
    assert free_area_m2(empty, resolution=0.05) == pytest.approx(0.0)
    unknown = np.full((4, 4), -1, dtype=np.int8)
    assert free_area_m2(unknown, resolution=0.05) == pytest.approx(0.0)
    occupied = np.full((3, 3), 100, dtype=np.int8)
    assert free_area_m2(occupied, resolution=0.1) == pytest.approx(0.0)


def test_coverage_ratio_clamps_above_one_negative():
    """Clamp hides laser overcount; callers must keep mapped ≤ GT."""
    assert coverage_ratio(150.0, 84.0) == pytest.approx(1.0)


def test_habitat_aligned_mapped_never_exceeds_gt_positive():
    """Revealed free+walls on a navmesh slice cannot exceed GT mappable."""
    navigable = np.zeros((8, 8), dtype=bool)
    navigable[2:6, 2:6] = True
    mpp = 0.5
    gt = gt_mappable_area_m2(navigable, mpp)
    # Partial reveal: some free + adjacent walls.
    grid = np.full((8, 8), -1, dtype=np.int8)
    grid[3:5, 3:5] = 0
    grid[2, 3] = 100
    grid[3, 2] = 100
    mapped = mapped_area_m2(grid, mpp)
    assert mapped < gt
    # Full reveal of floor+walls matches GT.
    full = np.full((8, 8), -1, dtype=np.int8)
    full[navigable] = 0
    walls = adjacent_wall_mask(navigable)
    full[walls] = 100
    assert mapped_area_m2(full, mpp) == pytest.approx(gt)



def test_floor_area_from_navigable_positive():
    navigable = np.array([[True, True], [False, True]], dtype=bool)
    assert floor_area_m2_from_navigable(navigable, mpp=0.5) == pytest.approx(0.75)


def test_gt_mappable_includes_adjacent_walls_positive():
    # One navigable cell with surrounding blocked → walls counted in GT.
    navigable = np.zeros((3, 3), dtype=bool)
    navigable[1, 1] = True
    mpp = 1.0
    floor_only = floor_area_m2_from_navigable(navigable, mpp)
    with_walls = gt_mappable_area_m2(navigable, mpp)
    assert floor_only == pytest.approx(1.0)
    # Center + 8 neighbors = 9
    assert with_walls == pytest.approx(9.0)
    assert with_walls > floor_only


def test_gt_mappable_all_blocked_negative():
    navigable = np.zeros((5, 5), dtype=bool)
    assert gt_mappable_area_m2(navigable, mpp=0.05) == pytest.approx(0.0)


def test_floor_area_all_blocked_negative():
    navigable = np.zeros((5, 5), dtype=bool)
    assert floor_area_m2_from_navigable(navigable, mpp=0.05) == pytest.approx(0.0)


def test_ring_buffer_keeps_newest_positive():
    ring = CoverageSampleRing(maxlen=3)
    ring.append(0.0, 0.1)
    ring.append(1.0, 0.2)
    ring.append(2.0, 0.3)
    ring.append(3.0, 0.4)
    xs, ys = ring.as_series()
    assert xs == [1.0, 2.0, 3.0]
    assert ys == [0.2, 0.3, 0.4]


def test_ring_buffer_empty_negative():
    ring = CoverageSampleRing(maxlen=10)
    xs, ys = ring.as_series()
    assert xs == []
    assert ys == []


def test_render_coverage_chart_jpeg_positive():
    jpeg = render_coverage_chart_jpeg([0.0, 1.0, 2.0], [0.0, 0.25, 0.5])
    assert isinstance(jpeg, (bytes, bytearray))
    assert len(jpeg) > 100
    assert jpeg[:2] == b"\xff\xd8"  # JPEG SOI


def test_render_coverage_chart_has_axis_labels_positive():
    """Axis label glyphs must paint non-background pixels (not empty axes)."""
    from PIL import Image

    jpeg = render_coverage_chart_jpeg([0.0, 2.0, 4.0], [0.1, 0.4, 0.7])
    img = Image.open(__import__("io").BytesIO(jpeg)).convert("RGB")
    arr = np.asarray(img)
    # Bottom band should contain X-axis label pixels lighter than background.
    bottom = arr[-30:, :, :]
    assert bottom.max() > 40
    # Left band should contain rotated Y-axis label pixels.
    left = arr[:, :40, :]
    assert left.max() > 40


def test_render_coverage_chart_empty_series_negative():
    jpeg = render_coverage_chart_jpeg([], [])
    assert isinstance(jpeg, (bytes, bytearray))
    assert jpeg[:2] == b"\xff\xd8"
