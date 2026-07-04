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


def test_normalize_range_rejects_zero_negative():
    assert normalize_range(0.0, range_min=0.1, clear_range=5.0) == 5.0
    assert normalize_range(-1.0, range_min=0.1, clear_range=5.0) == 5.0


def test_floor_band_is_bottom_rows_positive():
    assert row_band_slice(480, 24, anchor="bottom") == slice(456, 480)


def test_full_360_scan_has_full_circle_positive():
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
    assert finite_fraction(ranges) == pytest.approx(1.0)
    assert max(ranges) <= 5.0


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
    )
    assert 0.0 not in ranges
    assert all(r > 0.0 for r in ranges)


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
    assert finite_fraction(ranges) == pytest.approx(1.0)
    assert max(r for r in ranges) == pytest.approx(5.0)


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
