"""Tests for resilient tmux episode start (orchestrator)."""

from __future__ import annotations

import pytest

from experiments.orchestrator import build_tmux_episode_start_cmd, tmux_start_looks_ok


def test_harness_positive_control():
    assert 1 + 1 == 2


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_build_tmux_cmd_starts_server_after_kill_positive():
    cmd = build_tmux_episode_start_cmd("/data/experiments/exp/run/.env")
    assert "tmux kill-session -t habitat" in cmd
    assert "tmux start-server" in cmd
    assert "tmux new-session -d -s habitat" in cmd
    # start-server must come after kill and before new-session
    assert cmd.index("kill-session") < cmd.index("start-server") < cmd.index("new-session")
    assert "source '/data/experiments/exp/run/.env'" in cmd or \
        "source /data/experiments/exp/run/.env" in cmd
    assert "start_sim.sh" in cmd


def test_build_tmux_cmd_includes_settle_sleep_positive():
    cmd = build_tmux_episode_start_cmd("/tmp/.env")
    assert "sleep" in cmd


def test_tmux_start_ok_when_session_exists_positive():
    assert tmux_start_looks_ok(new_session_rc=0, has_session_rc=0) is True


def test_tmux_start_fails_when_server_exited_negative():
    assert tmux_start_looks_ok(new_session_rc=1, has_session_rc=1) is False


def test_tmux_start_fails_when_new_ok_but_session_missing_negative():
    # Race: new-session returned 0 but session never stuck
    assert tmux_start_looks_ok(new_session_rc=0, has_session_rc=1) is False
