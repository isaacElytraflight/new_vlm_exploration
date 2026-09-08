"""Tests for crash-safe experiment state."""

from __future__ import annotations

import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from experiments.state import ExperimentState, config_fingerprint  # noqa: E402


def test_fingerprint_stable_positive():
    assert config_fingerprint('{"a":1}') == config_fingerprint('{"a":1}')


def test_fingerprint_differs_negative():
    assert config_fingerprint('{"a":1}') != config_fingerprint('{"a":2}')


def test_state_mark_completed_positive(tmp_path: Path):
    state = ExperimentState(tmp_path / "experiment_state.json")
    state.load_or_create(experiment_id="exp", total_runs=4, fingerprint="abc")
    state.mark_completed("run_a")
    state.mark_completed("run_a")  # idempotent
    data = state.read()
    assert data is not None
    assert data["completed_run_ids"] == ["run_a"]


def test_state_fresh_clears_negative(tmp_path: Path):
    path = tmp_path / "experiment_state.json"
    state = ExperimentState(path)
    state.load_or_create(experiment_id="exp", total_runs=2, fingerprint="abc")
    state.mark_completed("run_a")
    state2 = ExperimentState(path)
    data = state2.load_or_create(
        experiment_id="exp", total_runs=2, fingerprint="abc", fresh=True
    )
    assert data["completed_run_ids"] == []
