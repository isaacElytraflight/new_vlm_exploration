"""Regression: floor band + clear_range for low-mounted depth camera."""

from __future__ import annotations

import numpy as np


def _floor_band_slice(height: int, scan_height: int) -> slice:
    return slice(height - scan_height, height)


def floor_band_valid_fraction(depth: np.ndarray, clear_range: float, scan_height: int = 24) -> float:
    band = depth[_floor_band_slice(depth.shape[0], scan_height), :]
    valid = band[np.isfinite(band) & (band > 0.1) & (band <= clear_range)]
    total = band[np.isfinite(band) & (band > 0.1)]
    if total.size == 0:
        return 0.0
    return float(valid.size) / float(total.size)


def test_floor_band_empty_at_3_5m_negative():
    rng = np.random.default_rng(0)
    depth = rng.uniform(4.0, 12.0, size=(480, 640)).astype(np.float32)
    assert floor_band_valid_fraction(depth, clear_range=3.5) == 0.0


def test_floor_band_nonempty_at_5m_positive():
    rng = np.random.default_rng(0)
    depth = rng.uniform(1.5, 4.5, size=(480, 640)).astype(np.float32)
    frac = floor_band_valid_fraction(depth, clear_range=5.0)
    assert frac > 0.25


def test_harness_negative_control():
    with __import__("pytest").raises(AssertionError):
        assert 1 == 2
