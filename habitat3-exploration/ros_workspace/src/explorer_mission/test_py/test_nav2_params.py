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


def test_yaw_goal_tolerance_about_60_deg_positive():
    """±60° goal yaw OK — do not demand perfect heading at arrival."""
    params = _controller_params()
    assert params["general_goal_checker"]["yaw_goal_tolerance"] == pytest.approx(1.047, abs=1e-3)


def test_rotate_to_heading_min_angle_allows_drive_positive():
    """~0.4 rad: leave pure-rotate soon enough that discrete steps can make XY progress."""
    params = _controller_params()
    assert params["FollowPath"]["rotate_to_heading_min_angle"] == pytest.approx(0.4, abs=1e-3)


def test_rotate_to_heading_not_60deg_negative():
    """Regression: ±60° pure-rotate + 10° discrete steps twitched and never drove."""
    params = _controller_params()
    assert params["FollowPath"]["rotate_to_heading_min_angle"] < 0.8


def test_xy_goal_tolerance_below_lookahead_positive():
    """RPP treats xy_goal_tolerance as carrot near-goal radius; must be < min_lookahead."""
    params = _controller_params()
    xy = params["general_goal_checker"]["xy_goal_tolerance"]
    min_la = params["FollowPath"]["min_lookahead_dist"]
    assert xy < min_la


def test_xy_goal_tolerance_not_1m_negative():
    """Regression: xy_goal_tolerance 1.0 always triggered rotate-to-goal (never drove)."""
    params = _controller_params()
    assert params["general_goal_checker"]["xy_goal_tolerance"] < 0.5


def test_xy_goal_tolerance_at_least_one_step_positive():
    """Keep acceptance at least one Habitat step (0.25 m) at the goal checker."""
    params = _controller_params()
    assert params["general_goal_checker"]["xy_goal_tolerance"] >= 0.25


def _planner_params():
    with PARAMS.open(encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data["planner_server"]["ros__parameters"]


def test_allow_unknown_false_positive():
    """Unknown cells must be lethal for planning (no jailbreak through unexplored)."""
    params = _planner_params()
    assert params["GridBased"]["allow_unknown"] is False


def test_allow_unknown_true_rejected_negative():
    """Regression: allow_unknown true let Navfn cut through unknown behind walls."""
    params = _planner_params()
    assert params["GridBased"]["allow_unknown"] is not True


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
