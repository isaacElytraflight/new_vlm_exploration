"""Unit tests for depth → LaserScan conversion."""

from __future__ import annotations

import math

import numpy as np
import pytest

from explorer_bridge.scan_from_depth import (
    depth_to_laserscan_bins,
    finite_fraction,
    normalize_range,
    pixel_elevation_rad,
    row_band_slice,
    _pixel_to_base_link_xy,
)


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_normalize_range_basic_positive():
    assert normalize_range(12.0, range_min=0.1, clear_range=5.0) == 5.0
    assert math.isnan(normalize_range(float("inf"), range_min=0.1, clear_range=5.0))
    assert math.isnan(normalize_range(49.7, range_min=0.1, clear_range=10.0, sensor_far=50.0))
    assert normalize_range(8.0, range_min=0.1, clear_range=10.0, sensor_far=50.0) == 8.0
    assert normalize_range(0.0, range_min=0.1, clear_range=5.0) == 5.0


def test_upper_third_band_placement_positive():
    s = row_band_slice(480, 24, anchor="upper_third")
    assert s == slice(148, 172)
    assert (s.start + s.stop) // 2 == 160


def test_upper_third_not_center_negative():
    assert row_band_slice(480, 24, anchor="upper_third") != row_band_slice(
        480, 24, anchor="center"
    )


def test_floor_band_is_bottom_rows_positive():
    assert row_band_slice(480, 24, anchor="bottom") == slice(456, 480)


def test_fov_only_leaves_outside_nan_positive():
    depth = np.full((480, 640), 3.0, dtype=np.float32)
    ranges, angle_min, angle_max, inc = depth_to_laserscan_bins(
        depth,
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        range_min=0.1,
        clear_range=5.0,
        full_360=True,
        num_bins=360,
        band_anchor="upper_third",
    )
    assert angle_min == pytest.approx(-math.pi)
    assert len(ranges) == 360
    assert 0.2 < finite_fraction(ranges) < 0.4
    assert sum(1 for r in ranges if math.isnan(r)) > 200


def test_uncovered_bearings_are_nan_positive():
    depth = np.zeros((480, 640), dtype=np.float32)
    depth[148:172, 300:340] = 3.0
    ranges, _, _, _ = depth_to_laserscan_bins(
        depth,
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        range_min=0.1,
        clear_range=5.0,
        band_anchor="upper_third",
        full_360=True,
        num_bins=360,
    )
    assert sum(1 for r in ranges if math.isnan(r)) > 200
    finite = [r for r in ranges if math.isfinite(r)]
    assert finite and all(abs(r - 5.0) > 1e-3 for r in finite)


def test_saturated_near_sensor_far_is_nan_positive():
    depth = np.full((480, 640), 49.7, dtype=np.float32)
    ranges, _, _, _ = depth_to_laserscan_bins(
        depth,
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        range_min=0.1,
        clear_range=10.0,
        band_anchor="upper_third",
        full_360=False,
        sensor_far=50.0,
    )
    assert all(math.isnan(r) for r in ranges)


def _scene_floor_and_walls() -> np.ndarray:
    """Low camera: floor at bottom, mid-height phantom, high wall in upper third."""
    depth = np.full((480, 640), 8.0, dtype=np.float32)
    depth[456:480, :] = 0.27
    depth[228:252, 200:440] = 3.0  # mid-frame phantom (old center-band failure)
    depth[148:172, 200:440] = 5.0  # real wall in upper third
    return depth


def test_upper_third_sees_high_wall_not_floor_or_phantom_positive():
    ranges, _, _, _ = depth_to_laserscan_bins(
        _scene_floor_and_walls(),
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        range_min=0.1,
        clear_range=10.0,
        scan_height=24,
        band_anchor="upper_third",
        full_360=False,
        sensor_far=50.0,
    )
    hits = [r for r in ranges if math.isfinite(r) and r < 9.0]
    assert hits and min(hits) == pytest.approx(5.0, abs=0.2)
    assert not any(r < 0.5 for r in hits)
    assert not any(2.5 <= r <= 3.5 for r in hits)


def test_center_band_grabs_mid_phantom_negative():
    """Negative: center band still picks the mid-height 3 m strip."""
    ranges, _, _, _ = depth_to_laserscan_bins(
        _scene_floor_and_walls(),
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        range_min=0.1,
        clear_range=10.0,
        band_anchor="center",
        full_360=False,
        sensor_far=50.0,
    )
    hits = [r for r in ranges if math.isfinite(r) and r < 9.0]
    assert hits and min(hits) == pytest.approx(3.0, abs=0.2)


def test_bottom_band_sees_floor_negative():
    ranges, _, _, _ = depth_to_laserscan_bins(
        _scene_floor_and_walls(),
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        range_min=0.1,
        clear_range=5.0,
        band_anchor="bottom",
        full_360=False,
    )
    hits = [r for r in ranges if math.isfinite(r) and r < 4.999]
    assert hits and min(hits) < 0.5


def test_upper_third_elevation_trig_positive():
    band = row_band_slice(480, 24, anchor="upper_third")
    row = (band.start + band.stop) // 2
    assert pixel_elevation_rad(row, fy=320.0, cy=240.0) > 0.2
    xy = _pixel_to_base_link_xy(320, row, 5.0, fx=320.0, fy=320.0, cx=320.0, cy=240.0)
    assert xy == pytest.approx((5.0, 0.0))
