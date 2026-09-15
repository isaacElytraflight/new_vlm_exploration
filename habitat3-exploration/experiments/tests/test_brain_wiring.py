"""Guards: ablation apply script sets brain_id (Goal C), not only DFS knobs."""

from __future__ import annotations

from pathlib import Path

import pytest

from experiments.apply_profile_policy import apply_profile_failure_is_fatal

SCRIPT = Path(__file__).resolve().parents[2] / "sim" / "scripts" / "apply_exploration_profile.sh"
START_SIM = Path(__file__).resolve().parents[2] / "sim" / "scripts" / "start_sim.sh"


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_apply_profile_sets_brain_id_positive():
    assert SCRIPT.is_file()
    text = SCRIPT.read_text(encoding="utf-8")
    assert "brain_id" in text
    assert "greedy_nearest" in text
    assert "vlm_tree_dfs" in text


def test_apply_profile_rejects_unknown_negative():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "Unknown profile" in text


def test_start_sim_passes_brain_id_positive():
    assert START_SIM.is_file()
    text = START_SIM.read_text(encoding="utf-8")
    assert "selected_brain.id" in text
    assert "brain_id:=" in text


def test_apply_profile_soft_fail_when_brain_at_launch_positive():
    """Ablation launches with EXPLORER_BRAIN_ID — apply failure must not kill the cell."""
    assert apply_profile_failure_is_fatal(brain_set_at_launch=True) is False


def test_apply_profile_fatal_when_brain_not_at_launch_negative():
    assert apply_profile_failure_is_fatal(brain_set_at_launch=False) is True
