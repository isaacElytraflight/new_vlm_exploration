from __future__ import annotations

import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from experiments.aggregate import aggregate_experiment, format_aggregate_table  # noqa: E402
from experiments.db import ExperimentDB  # noqa: E402


def _seed_runs(db: ExperimentDB, experiment_id: str) -> None:
    db.initialize()
    for i, (algo, cov, dist) in enumerate(
        [
            ("vlm_dfs", 0.40, 10.0),
            ("vlm_dfs", 0.50, 12.0),
            ("greedy_nearest", 0.30, 8.0),
            ("greedy_nearest", 0.34, 9.0),
        ]
    ):
        rid = f"{experiment_id}__{algo}__scene__seed{i}"
        db.insert_run_start(
            run_id=rid,
            experiment_id=experiment_id,
            algorithm_id=algo,
            scene_id="scene",
            seed=i,
            config_json="{}",
            artifact_dir=f"runs/{rid}",
        )
        db.finalize_run(
            rid,
            status="completed",
            final_coverage=cov,
            distance_m=dist,
            duration_s=100.0 + i,
        )


def test_aggregate_mean_std_positive(tmp_path: Path):
    db_path = tmp_path / "results.sqlite"
    db = ExperimentDB(db_path)
    _seed_runs(db, "exp1")
    rows = aggregate_experiment(db_path, "exp1")
    assert len(rows) == 2
    vlm = next(r for r in rows if r.algorithm_id == "vlm_dfs")
    assert vlm.coverage_mean == pytest.approx(0.45)
    assert vlm.coverage_std == pytest.approx(0.0707, abs=1e-3)
    table = format_aggregate_table(rows)
    assert "vlm_dfs" in table
    assert "greedy_nearest" in table


def test_aggregate_empty_experiment_negative(tmp_path: Path):
    db = ExperimentDB(tmp_path / "results.sqlite")
    db.initialize()
    rows = aggregate_experiment(tmp_path / "results.sqlite", "missing")
    assert rows == []
    assert format_aggregate_table(rows) == "(no runs)"


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
