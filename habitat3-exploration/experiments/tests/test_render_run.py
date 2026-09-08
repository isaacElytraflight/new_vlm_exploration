"""Tests for offline render_run figures (no ROS)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from experiments.package import write_run_package  # noqa: E402
from experiments.render_run import load_coverage_csv, render_figures  # noqa: E402


def test_harness_positive_control():
    assert 1 + 1 == 2


def test_render_figures_positive(tmp_path: Path):
    pytest.importorskip("matplotlib")
    run_dir = tmp_path / "run"
    write_run_package(
        run_dir,
        run_id="exp__algo__scene__seed0",
        experiment_id="exp",
        algorithm_id="vlm_dfs",
        scene_id="scene",
        seed=0,
        status="completed",
        eval_fov_deg=360.0,
        eval_reveal_radius_m=5.0,
        coverage_samples=[[0.0, 0.0, 0.0], [10.0, 5.0, 0.4], [20.0, 12.0, 0.7]],
        trajectory=[[0.0, 0.0, 0.0], [1.0, 0.0, 1.0]],
        summary={
            "final_coverage": 0.7,
            "distance_m": 12.0,
            "status": "completed",
        },
        revisit_bins={1: 20, 2: 3, 3: 1},
    )
    out = tmp_path / "figures"
    outputs = render_figures(run_dir, out)
    assert outputs["coverage_vs_distance"].is_file()
    assert outputs["revisit_histogram"].is_file()
    assert outputs["summary_panel"].is_file()
    dists, covs = load_coverage_csv(run_dir / "metrics" / "coverage_vs_distance.csv")
    assert len(dists) == 3
    assert covs[-1] == pytest.approx(70.0)


def test_render_missing_manifest_negative(tmp_path: Path):
    pytest.importorskip("matplotlib")
    with pytest.raises(FileNotFoundError):
        render_figures(tmp_path / "empty", tmp_path / "out")
