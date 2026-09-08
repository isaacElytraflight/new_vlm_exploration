"""Tests for per-run artifact package writers."""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from experiments.package import (  # noqa: E402
    atomic_write_text,
    package_byte_sizes,
    write_coverage_csv,
    write_run_package,
    write_trajectory_csv,
)


def test_harness_positive_control():
    assert 1 + 1 == 2


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_write_coverage_csv_positive(tmp_path: Path):
    path = tmp_path / "coverage_vs_distance.csv"
    write_coverage_csv(path, [[0.0, 0.0, 0.1], [5.0, 2.5, 0.4]])
    with path.open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == 2
    assert float(rows[1]["coverage"]) == pytest.approx(0.4)


def test_write_coverage_skips_short_rows_negative(tmp_path: Path):
    path = tmp_path / "coverage_vs_distance.csv"
    write_coverage_csv(path, [[1.0, 2.0], [3.0, 4.0, 0.5]])
    with path.open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    assert len(rows) == 1


def test_trajectory_csv_collector_order_positive(tmp_path: Path):
    path = tmp_path / "trajectory.csv"
    # Collector format: x, y, t_s
    write_trajectory_csv(path, [[1.0, 2.0, 0.5], [1.5, 2.5, 1.0]])
    with path.open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    assert float(rows[0]["t_s"]) == pytest.approx(0.5)
    assert float(rows[0]["x"]) == pytest.approx(1.0)


def test_atomic_write_replaces_positive(tmp_path: Path):
    path = tmp_path / "note.txt"
    atomic_write_text(path, "a\n")
    atomic_write_text(path, "b\n")
    assert path.read_text(encoding="utf-8") == "b\n"
    assert not path.with_suffix(".txt.tmp").exists()


def test_write_run_package_manifest_positive(tmp_path: Path):
    run_dir = tmp_path / "run0"
    manifest = write_run_package(
        run_dir,
        run_id="exp__algo__scene__seed0",
        experiment_id="exp",
        algorithm_id="algo",
        scene_id="scene",
        seed=0,
        status="completed",
        eval_fov_deg=360.0,
        eval_reveal_radius_m=5.0,
        coverage_samples=[[0.0, 0.0, 0.0], [10.0, 3.0, 0.2]],
        trajectory=[[0.0, 0.0, 0.0], [1.0, 0.0, 1.0]],
        summary={"final_coverage": 0.2, "distance_m": 3.0},
        revisit_bins={1: 10, 2: 1},
    )
    assert (run_dir / "manifest.json").is_file()
    assert (run_dir / "metrics" / "coverage_vs_distance.csv").is_file()
    assert (run_dir / "metrics" / "trajectory.csv").is_file()
    assert (run_dir / "metrics" / "summary.json").is_file()
    data = json.loads((run_dir / "manifest.json").read_text(encoding="utf-8"))
    assert data["schema_version"] == 1
    assert data["paths"]["coverage_vs_distance"].endswith(".csv")
    sizes = package_byte_sizes(run_dir, data["paths"])
    assert manifest["total_bytes"] == sum(sizes.values())
    assert sizes["coverage_vs_distance"] > 0
    assert sizes["summary"] > 0
