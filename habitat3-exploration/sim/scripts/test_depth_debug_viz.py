"""Unit tests for depth debug colormap (positive + negative + harness)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

# Allow `import depth_debug_viz` when run from ros_workspace or scripts dir.
sys.path.insert(0, str(Path(__file__).resolve().parent))

from depth_debug_viz import (
    COLOR_BEYOND_HORIZON,
    COLOR_NAN,
    COLOR_SAT,
    COLOR_ZERO,
    classify_depth_masks,
    depth_debug_counts,
    depth_to_debug_bgr,
)


def test_harness_positive():
    assert 1 + 1 == 2


def test_harness_negative():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_classify_flags_specials_positive():
    depth = np.array(
        [
            [float("nan"), 0.0, -1.0],
            [0.05, 3.0, 15.0],
            [49.7, 50.0, float("inf")],
        ],
        dtype=np.float32,
    )
    m = classify_depth_masks(depth, range_min=0.1, range_max=10.0, sensor_far=50.0, sat_eps=0.5)
    assert m["nan"][0, 0] and m["nan"][2, 2]
    assert m["zero"][0, 1]
    assert m["neg"][0, 2]
    assert m["near"][1, 0]
    assert m["valid"][1, 1]
    assert m["beyond"][1, 2]
    assert m["sat"][2, 0] and m["sat"][2, 1]


def test_valid_and_sat_mutually_exclusive_negative():
    """Negative: a saturated pixel must not also count as valid mid-range."""
    depth = np.full((4, 4), 49.8, dtype=np.float32)
    m = classify_depth_masks(depth, sensor_far=50.0, sat_eps=0.5)
    assert np.all(m["sat"])
    assert not np.any(m["valid"])
    assert not np.any(m["beyond"])


def test_colormap_paints_nan_magenta_positive():
    depth = np.full((8, 8), float("nan"), dtype=np.float32)
    bgr = depth_to_debug_bgr(depth, with_legend=False)
    assert bgr.shape == (8, 8, 3)
    assert np.all(bgr == COLOR_NAN)


def test_colormap_paints_zero_red_positive():
    depth = np.zeros((8, 8), dtype=np.float32)
    bgr = depth_to_debug_bgr(depth, with_legend=False)
    assert np.all(bgr == COLOR_ZERO)


def test_colormap_paints_sat_white_positive():
    depth = np.full((8, 8), 49.7, dtype=np.float32)
    bgr = depth_to_debug_bgr(depth, sensor_far=50.0, sat_eps=0.5, with_legend=False)
    assert np.all(bgr == COLOR_SAT)


def test_colormap_beyond_horizon_yellow_positive():
    depth = np.full((8, 8), 15.0, dtype=np.float32)
    bgr = depth_to_debug_bgr(depth, range_max=10.0, sensor_far=50.0, with_legend=False)
    assert np.all(bgr == COLOR_BEYOND_HORIZON)


def test_valid_depth_not_special_overlay_negative():
    """Negative: a normal 3 m wall must not be flagged as sat/nan/zero."""
    depth = np.full((16, 16), 3.0, dtype=np.float32)
    counts = depth_debug_counts(depth)
    assert counts["valid"] == 16 * 16
    assert counts["sat"] == 0
    assert counts["nan"] == 0
    assert counts["zero"] == 0
    bgr = depth_to_debug_bgr(depth, with_legend=False)
    assert not np.all(bgr == COLOR_SAT)
    assert not np.all(bgr == COLOR_NAN)
    assert not np.all(bgr == COLOR_ZERO)


def test_legend_appended_positive():
    depth = np.full((10, 20), 2.0, dtype=np.float32)
    bgr = depth_to_debug_bgr(depth, with_legend=True)
    assert bgr.shape[0] == 10 + 28
    assert bgr.shape[1] == 20


def test_rejects_non_2d_negative():
    with pytest.raises(ValueError, match="2-D"):
        depth_to_debug_bgr(np.zeros((2, 2, 2), dtype=np.float32))
