"""Guard nav2_params.yaml against configs that crash Nav2 bringup."""

from pathlib import Path

import pytest
import yaml


PARAMS = (
    Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
)


def _collision_monitor_params():
    with PARAMS.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data["collision_monitor"]["ros__parameters"]


def test_collision_monitor_observation_sources_nonempty_positive():
    """Nav2 Jazzy requires a declared observation_sources list (may be disabled)."""
    params = _collision_monitor_params()
    sources = params["observation_sources"]
    assert isinstance(sources, list)
    assert len(sources) >= 1
    for name in sources:
        assert name in params
        assert params[name].get("enabled") is False


def test_collision_monitor_empty_observation_sources_rejected_negative():
    """Empty observation_sources is the known crash config — must not ship."""
    params = _collision_monitor_params()
    assert params["observation_sources"] != []


def _controller_params():
    with PARAMS.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data["controller_server"]["ros__parameters"]


def test_yaw_tolerance_about_60_deg_positive():
    """±60° ≈ 1.047 rad — prefer driving over in-place spin for coarse misalignment."""
    params = _controller_params()
    assert params["general_goal_checker"]["yaw_goal_tolerance"] == pytest.approx(1.047, abs=1e-3)
    assert params["FollowPath"]["rotate_to_heading_min_angle"] == pytest.approx(1.047, abs=1e-3)


def test_yaw_tolerance_not_tight_30deg_negative():
    """Regression: ≤30° still caused visible Habitat turn jitter."""
    params = _controller_params()
    assert params["FollowPath"]["rotate_to_heading_min_angle"] > 0.6


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
