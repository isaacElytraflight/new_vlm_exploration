"""Guards for Habitat Nav2 bringup without collision_monitor."""

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
SIM_LAUNCH = ROOT / "launch" / "navigation_sim.launch.py"
PARENT_LAUNCH = ROOT / "launch" / "nav2_exploration.launch.py"


def test_harness_positive():
    assert 1 + 1 == 2


def test_harness_negative():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_sim_launch_omits_collision_monitor_positive():
    text = SIM_LAUNCH.read_text(encoding="utf-8")
    assert "nav2_collision_monitor" not in text
    assert "LIFECYCLE_NODES" in text
    assert '"collision_monitor"' not in text
    assert '"controller_server"' in text
    assert '"velocity_smoother"' in text
    assert '"bt_navigator"' in text


def test_sim_launch_must_not_manage_collision_monitor_negative():
    """Regression: lifecycle including collision_monitor aborts Nav2 bringup."""
    text = SIM_LAUNCH.read_text(encoding="utf-8")
    assert "nav2_collision_monitor" not in text
    assert '"collision_monitor"' not in text


def test_sim_launch_wires_cmd_vel_bypass_positive():
    text = SIM_LAUNCH.read_text(encoding="utf-8")
    assert '("cmd_vel_smoothed", "cmd_vel")' in text


def test_parent_uses_sim_navigation_launch_positive():
    text = PARENT_LAUNCH.read_text(encoding="utf-8")
    assert "navigation_sim.launch.py" in text
    assert "navigation_launch.py" not in text
