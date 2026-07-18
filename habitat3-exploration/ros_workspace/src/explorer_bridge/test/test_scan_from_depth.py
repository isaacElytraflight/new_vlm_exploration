"""Unit tests for depth → LaserScan range conversion."""

from __future__ import annotations

import math

import numpy as np
import pytest

from explorer_bridge.scan_from_depth import (
    depth_to_laserscan_bins,
    finite_fraction,
    normalize_range,
    row_band_slice,
)


def test_normalize_range_clamps_long_hits_positive():
    assert normalize_range(12.0, range_min=0.1, clear_range=5.0) == 5.0
    assert normalize_range(float("inf"), range_min=0.1, clear_range=5.0) == 5.0


def test_normalize_range_near_clip_is_clear_positive():
    assert normalize_range(4.6, range_min=0.1, clear_range=5.0, free_near_eps=0.5) == 5.0
    assert normalize_range(8.0, range_min=0.1, clear_range=10.0, free_near_eps=2.5) == 10.0


def test_normalize_range_inside_clip_kept_negative():
    """Negative control: real mid-range hits must not be forced to clear_range."""
    assert normalize_range(3.0, range_min=0.1, clear_range=5.0, free_near_eps=0.5) == 3.0
    assert normalize_range(6.0, range_min=0.1, clear_range=10.0, free_near_eps=2.5) == 6.0


def test_normalize_range_rejects_zero_negative():
    assert normalize_range(0.0, range_min=0.1, clear_range=5.0) == 5.0
    assert normalize_range(-1.0, range_min=0.1, clear_range=5.0) == 5.0


def test_floor_band_is_bottom_rows_positive():
    assert row_band_slice(480, 24, anchor="bottom") == slice(456, 480)


def test_full_360_scan_covers_camera_fov_only_positive():
    """360-bin scans still only contain rays inside the depth camera FOV."""
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
    )
    assert angle_min == pytest.approx(-math.pi)
    assert angle_max == pytest.approx(math.pi)
    assert len(ranges) == 360
    assert inc == pytest.approx(2.0 * math.pi / 360.0)
    frac = finite_fraction(ranges)
    assert 0.2 < frac < 0.4
    assert sum(1 for r in ranges if math.isnan(r)) > 200
    assert all(math.isfinite(r) and 0.0 < r <= 5.0 for r in ranges if math.isfinite(r))


def test_zero_depth_becomes_clear_range_not_zero_positive():
    depth = np.zeros((480, 640), dtype=np.float32)
    depth[470:480, 300:340] = 2.0
    ranges, _, _, _ = depth_to_laserscan_bins(
        depth,
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        range_min=0.1,
        clear_range=5.0,
        full_360=True,
        band_anchor="bottom",
    )
    finite = [r for r in ranges if math.isfinite(r)]
    assert finite
    assert 0.0 not in finite
    assert all(r > 0.0 for r in finite)


def test_uncovered_bearings_are_nan_not_clear_positive():
    """Outside the camera FOV must be NaN — fake clear_range fills break SLAM matching."""
    depth = np.zeros((480, 640), dtype=np.float32)
    depth[228:252, 300:340] = 3.0
    ranges, _, _, _ = depth_to_laserscan_bins(
        depth,
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        range_min=0.1,
        clear_range=5.0,
        scan_height=24,
        band_anchor="center",
        full_360=True,
        num_bins=360,
        free_near_eps=0.5,
    )
    nans = sum(1 for r in ranges if math.isnan(r))
    finite = [r for r in ranges if math.isfinite(r)]
    assert nans > 200
    assert finite
    assert all(abs(r - 5.0) > 1e-3 for r in finite)


def test_uncovered_bearings_must_not_be_clear_range_negative():
    depth = np.zeros((480, 640), dtype=np.float32)
    depth[228:252, 300:340] = 3.0
    ranges, _, _, _ = depth_to_laserscan_bins(
        depth,
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        range_min=0.1,
        clear_range=5.0,
        band_anchor="center",
        full_360=True,
        num_bins=360,
        free_near_eps=0.5,
    )
    fake_clear = sum(1 for r in ranges if math.isfinite(r) and abs(r - 5.0) < 1e-6)
    assert fake_clear == 0


def test_horizon_depth_clamped_at_clear_range_positive():
    depth = np.full((480, 640), 8.0, dtype=np.float32)
    ranges, _, _, _ = depth_to_laserscan_bins(
        depth,
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        range_min=0.1,
        clear_range=5.0,
        band_anchor="bottom",
        full_360=False,
    )
    finite = [r for r in ranges if math.isfinite(r)]
    assert finite
    assert all(r == pytest.approx(5.0) for r in finite)


def _habitat_like_depth() -> np.ndarray:
    """Level camera at ~0.1 m: floor in bottom rows, walls near image center."""
    depth = np.full((480, 640), 8.0, dtype=np.float32)
    depth[456:480, :] = 0.27
    # Wall well inside clear_range so free_near_eps does not treat it as clip.
    depth[228:252, 200:440] = 3.5
    return depth


def test_center_band_sees_walls_not_floor_positive():
    """Horizon band must report wall ranges, not the near floor plane."""
    ranges, _, _, _ = depth_to_laserscan_bins(
        _habitat_like_depth(),
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        range_min=0.1,
        clear_range=5.0,
        scan_height=24,
        band_anchor="center",
        full_360=True,
        num_bins=360,
        free_near_eps=0.5,
    )
    hits = [r for r in ranges if math.isfinite(r) and r < 4.4]
    assert hits, "expected wall hits in center band"
    assert min(hits) > 1.0
    assert any(3.0 <= r <= 4.0 for r in hits)


def test_bottom_band_sees_floor_as_near_hit_negative():
    """Negative control: bottom rows are the floor at ~0.27 m for a low camera."""
    ranges, _, _, _ = depth_to_laserscan_bins(
        _habitat_like_depth(),
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        range_min=0.1,
        clear_range=5.0,
        scan_height=24,
        band_anchor="bottom",
        full_360=True,
        num_bins=360,
    )
    hits = [r for r in ranges if math.isfinite(r) and r < 4.999]
    assert hits, "bottom band should produce floor hits"
    assert min(hits) < 0.5


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
