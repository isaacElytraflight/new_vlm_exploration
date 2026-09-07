from __future__ import annotations

import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from experiments.db import ExperimentDB  # noqa: E402


def test_db_roundtrip_positive(tmp_path: Path):
    db = ExperimentDB(tmp_path / "results.sqlite")
    db.initialize()
    db.insert_run_start(
        run_id="exp__algo__scene__seed0",
        experiment_id="exp",
        algorithm_id="algo",
        scene_id="scene",
        seed=0,
        config_json="{}",
        artifact_dir="sim/data/experiments/exp/run0",
        started_at="2026-08-31T00:00:00+00:00",
    )
    db.finalize_run(
        "exp__algo__scene__seed0",
        status="completed",
        finished_at="2026-08-31T01:00:00+00:00",
        final_coverage=0.42,
        distance_m=12.5,
        duration_s=3600.0,
        coverage_samples=[[0.0, 0.0, 0.1], [10.0, 5.0, 0.3]],
        revisit_bins={1: 40, 2: 3},
    )
    run = db.get_run("exp__algo__scene__seed0")
    assert run is not None
    assert run.status == "completed"
    assert run.final_coverage == pytest.approx(0.42)
    assert len(db.list_runs("exp")) == 1


def test_db_missing_run_negative(tmp_path: Path):
    db = ExperimentDB(tmp_path / "results.sqlite")
    db.initialize()
    assert db.get_run("missing") is None


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
