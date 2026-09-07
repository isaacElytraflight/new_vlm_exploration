from __future__ import annotations

import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from experiments.metrics import (
    compute_revisit_bins,
    coverage_ratio,
    integrate_trajectory_meters,
)


def test_integrate_trajectory_positive():
    traj = [(0.0, 0.0), (3.0, 4.0)]
    assert integrate_trajectory_meters(traj) == pytest.approx(5.0)


def test_integrate_trajectory_single_point_negative():
    assert integrate_trajectory_meters([(1.0, 2.0)]) == 0.0


def test_revisit_bins_positive():
    # Revisit (0,0) three times, (1,0) once at 0.25 m cells.
    traj = [(0.0, 0.0), (0.0, 0.0), (0.25, 0.0), (0.0, 0.0)]
    bins = compute_revisit_bins(traj, cell_size_m=0.25)
    assert bins[1] == 1
    assert bins[3] == 1


def test_revisit_bins_empty_negative():
    assert compute_revisit_bins([]) == {}


def test_coverage_ratio_clamped_positive():
    assert coverage_ratio(50.0, 100.0) == pytest.approx(0.5)
    assert coverage_ratio(150.0, 100.0) == pytest.approx(1.0)


def test_coverage_ratio_zero_gt_negative():
    assert coverage_ratio(10.0, 0.0) == 0.0


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
