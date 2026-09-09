"""Tests for crash-safe resume skip + interrupted recovery."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from experiments.config import ExperimentConfig, expand_matrix, run_id_for  # noqa: E402
from experiments.db import ExperimentDB  # noqa: E402
from experiments.orchestrator import ExperimentOrchestrator  # noqa: E402


SMOKE_YAML = """
experiment_id: resume_test
n_runs_per_cell: 2
timeout_s: 60
eval:
  fov_deg: 360
  reveal_radius_m: 5.0
algorithms:
  - id: vlm_dfs
    brain: vlm_tree_dfs
scenes:
  - 17DRP5sb8fy
seeds:
  mode: sequential
  start: 0
artifact_root: sim/data/experiments
"""


def _write_config(tmp_path: Path) -> Path:
    path = tmp_path / "exp.yaml"
    path.write_text(SMOKE_YAML, encoding="utf-8")
    return path


def test_mark_running_interrupted_positive(tmp_path: Path):
    db = ExperimentDB(tmp_path / "results.sqlite")
    db.initialize()
    db.insert_run_start(
        run_id="r1",
        experiment_id="resume_test",
        algorithm_id="vlm_dfs",
        scene_id="17DRP5sb8fy",
        seed=0,
        config_json="{}",
        artifact_dir="x",
    )
    ids = db.mark_running_interrupted("resume_test")
    assert ids == ["r1"]
    assert db.get_run("r1").status == "interrupted"


def test_list_completed_excludes_interrupted_negative(tmp_path: Path):
    db = ExperimentDB(tmp_path / "results.sqlite")
    db.initialize()
    db.insert_run_start(
        run_id="r1",
        experiment_id="resume_test",
        algorithm_id="a",
        scene_id="s",
        seed=0,
        config_json="{}",
        artifact_dir="x",
    )
    db.finalize_run("r1", status="completed")
    db.insert_run_start(
        run_id="r2",
        experiment_id="resume_test",
        algorithm_id="a",
        scene_id="s",
        seed=1,
        config_json="{}",
        artifact_dir="x",
    )
    db.mark_running_interrupted("resume_test")
    assert db.list_completed_run_ids("resume_test") == ["r1"]


def test_resume_skips_completed_dry_run_positive(tmp_path: Path):
    cfg_path = _write_config(tmp_path)
    config = ExperimentConfig.from_yaml(cfg_path)
    # Point artifact root into tmp so state/db stay isolated.
    config.artifact_root = Path("artifacts")
    project = tmp_path / "proj"
    project.mkdir()
    (project / "artifacts").mkdir()

    db = ExperimentDB(project / "artifacts" / "results.sqlite")
    db.initialize()
    specs = list(expand_matrix(config))
    first = run_id_for(specs[0])
    db.insert_run_start(
        run_id=first,
        experiment_id=config.experiment_id,
        algorithm_id=specs[0].algorithm_id,
        scene_id=specs[0].scene_id,
        seed=specs[0].seed,
        config_json="{}",
        artifact_dir="x",
    )
    db.finalize_run(first, status="completed")

    orch = ExperimentOrchestrator(
        config,
        project_root=project,
        dry_run=True,
        resume=True,
        progress_file=project / "progress.json",
    )
    results = orch.run_all()
    # One prior completed + remaining dry_runs
    assert any(r.run_id == first and r.status == "completed" for r in results)
    dry = [r for r in results if r.status == "dry_run"]
    assert len(dry) == len(specs) - 1
    progress = json.loads((project / "progress.json").read_text(encoding="utf-8"))
    assert progress["resumed"] is True
    assert progress["completed_prior"] == 1


def test_fresh_ignores_completed_negative(tmp_path: Path):
    cfg_path = _write_config(tmp_path)
    config = ExperimentConfig.from_yaml(cfg_path)
    config.artifact_root = Path("artifacts")
    project = tmp_path / "proj"
    project.mkdir()
    (project / "artifacts").mkdir()

    db = ExperimentDB(project / "artifacts" / "results.sqlite")
    db.initialize()
    specs = list(expand_matrix(config))
    first = run_id_for(specs[0])
    db.insert_run_start(
        run_id=first,
        experiment_id=config.experiment_id,
        algorithm_id=specs[0].algorithm_id,
        scene_id=specs[0].scene_id,
        seed=specs[0].seed,
        config_json="{}",
        artifact_dir="x",
    )
    db.finalize_run(first, status="completed")

    orch = ExperimentOrchestrator(
        config,
        project_root=project,
        dry_run=True,
        resume=False,
        fresh=True,
        progress_file=project / "progress.json",
    )
    results = orch.run_all()
    assert all(r.status == "dry_run" for r in results)
    assert len(results) == len(specs)
    progress = json.loads((project / "progress.json").read_text(encoding="utf-8"))
    assert progress["resumed"] is False
    assert progress["completed_prior"] == 0
