"""Guards: episode cleanup must kill orphaned ROS/Habitat processes outside tmux."""

from __future__ import annotations

from pathlib import Path

import pytest

CLEANUP = Path(__file__).resolve().parent / "cleanup_episode.sh"
STOP = Path(__file__).resolve().parent / "stop_sim.sh"
START = Path(__file__).resolve().parent / "start_sim.sh"

REQUIRED_PATTERNS = [
    "habitat_engine.py",
    "known_pose_mapper",
    "depth_to_laserscan",
    "explorer_bridge_node",
    "ros2 launch explorer_mission",
    "ros2 run explorer_bridge",
    "static_transform_publisher .* map odom",
]


def test_cleanup_script_lists_orphan_patterns_positive():
    text = CLEANUP.read_text(encoding="utf-8")
    for pat in REQUIRED_PATTERNS:
        assert pat in text, f"missing cleanup pattern: {pat}"


def test_stop_sim_invokes_cleanup_positive():
    text = STOP.read_text(encoding="utf-8")
    assert "cleanup_episode.sh" in text
    assert "tmux kill-session" in text


def test_start_sim_cleans_before_launch_positive():
    text = START.read_text(encoding="utf-8")
    assert "cleanup_episode.sh" in text
    assert text.index("cleanup_episode.sh") < text.index("habitat_engine.py")


def test_cleanup_missing_mapper_would_leave_dupes_negative():
    text = CLEANUP.read_text(encoding="utf-8")
    assert "known_pose_mapper" in text
    # Negative control: a stop that only kills tmux is insufficient.
    assert "tmux kill-session" not in text or "pkill" in text


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
