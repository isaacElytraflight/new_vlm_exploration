"""Regression: horizon (center) band for low-mounted depth camera SLAM."""

from __future__ import annotations

import numpy as np
import pytest


def _row_band_slice(height: int, scan_height: int, *, anchor: str) -> slice:
    band = max(1, scan_height)
    if anchor == "bottom":
        return slice(height - band, height)
    half = max(1, band // 2)
    cy = height // 2
    return slice(cy - half, cy + half)


def band_valid_fraction(
    depth: np.ndarray,
    clear_range: float,
    scan_height: int = 24,
    *,
    anchor: str = "center",
) -> float:
    band = depth[_row_band_slice(depth.shape[0], scan_height, anchor=anchor), :]
    valid = band[np.isfinite(band) & (band > 0.1) & (band <= clear_range)]
    total = band[np.isfinite(band) & (band > 0.1)]
    if total.size == 0:
        return 0.0
    return float(valid.size) / float(total.size)


def test_center_band_empty_at_3_5m_negative():
    rng = np.random.default_rng(0)
    depth = rng.uniform(4.0, 12.0, size=(480, 640)).astype(np.float32)
    assert band_valid_fraction(depth, clear_range=3.5, anchor="center") == 0.0


def test_center_band_nonempty_at_5m_positive():
    rng = np.random.default_rng(0)
    depth = rng.uniform(1.5, 4.5, size=(480, 640)).astype(np.float32)
    frac = band_valid_fraction(depth, clear_range=5.0, anchor="center")
    assert frac > 0.25


def test_bottom_band_floor_only_is_not_useful_for_walls_negative():
    """Floor plane in bottom rows must not be treated as a wall-valid band."""
    depth = np.full((480, 640), 8.0, dtype=np.float32)
    depth[456:480, :] = 0.27
    depth[228:252, :] = 4.5
    assert band_valid_fraction(depth, clear_range=5.0, anchor="bottom") == 1.0
    # But those "valid" bottom values are all near-floor — wrong for SLAM walls.
    band = depth[456:480, :]
    assert float(np.median(band)) < 0.5


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
