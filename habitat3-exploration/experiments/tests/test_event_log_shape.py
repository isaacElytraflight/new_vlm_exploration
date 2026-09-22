"""Tests for compact event payload shaping (no ROS)."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from experiments.event_shapes import (  # noqa: E402
    action_name_from_brain_action,
    compact_brain_decision_event,
    compact_graph_edges_event,
    compact_nav_fail_event,
    compact_scores_event,
    compact_status_event,
    compact_vlm_choice_event,
)


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_status_event_positive():
    ev = compact_status_event("navigating", "ok", current_node_id=1)
    assert ev["phase"] == "navigating"
    line = json.dumps(ev)
    assert "navigating" in line


def test_reasoning_truncated_negative():
    long = "x" * 500
    ev = compact_scores_event([1], [3], [long])
    assert len(ev["reasonings"][0]) == 160


def test_no_image_fields_in_payload_negative():
    ev = compact_status_event("idle", "waiting")
    assert "data" not in ev
    assert "image" not in ev


def test_brain_decision_includes_brain_and_visited_positive():
    ev = compact_brain_decision_event(
        brain_id="greedy_nearest",
        action="navigate",
        goal_id=2,
        goal_x=1.5,
        goal_y=0.0,
        detail="greedy nearest euclidean",
        visited_ids=[1],
        live_ids=[2, 3],
        robot_x=0.5,
        robot_y=-0.25,
        goal_distance_m=1.030776,
    )
    assert ev["topic"] == "exploration/brain/decision"
    assert ev["brain_id"] == "greedy_nearest"
    assert ev["action"] == "navigate"
    assert ev["visited_ids"] == [1]
    assert ev["live_ids"] == [2, 3]
    assert ev["robot_x"] == 0.5
    assert ev["goal_distance_m"] == pytest.approx(1.030776)
    assert "image" not in ev


def test_nav_fail_event_includes_clearance_and_codes_positive():
    ev = compact_nav_fail_event(
        stage="classified",
        fail_class="stuck",
        goal_id=7,
        nav_error_code=208,
        new_plan_code=208,
        start_clearance_m=0.18,
        start_clearance_ok=False,
        grid_occ_at_robot=0,
        grid_occ_at_goal=100,
        costmap_at_robot=254,
        costmap_at_goal=254,
    )
    assert ev["topic"] == "exploration/nav_fail"
    assert ev["stage"] == "classified"
    assert ev["fail_class"] == "stuck"
    assert ev["start_clearance_m"] == pytest.approx(0.18)
    assert ev["costmap_at_robot"] == 254
    assert "image" not in ev


def test_nav_fail_event_truncates_error_strings_negative():
    ev = compact_nav_fail_event(
        stage="resolved",
        resolution="mark_dead",
        nav_error="e" * 500,
        new_plan_err="p" * 500,
    )
    assert len(ev["nav_error"]) == 240
    assert len(ev["new_plan_err"]) == 240


def test_brain_decision_detail_truncated_negative():
    ev = compact_brain_decision_event(
        brain_id="vlm_tree_dfs",
        action="complete",
        detail="x" * 500,
    )
    assert len(ev["detail"]) == 240


def test_graph_edges_event_positive():
    ev = compact_graph_edges_event(
        brain_id="vlm_frontier_graph",
        from_ids=[1, 1],
        to_ids=[2, 3],
        costs=[4.5, 9.0],
    )
    assert ev["topic"] == "exploration/brain/graph_edges"
    assert ev["costs"] == [4.5, 9.0]


def test_graph_edges_length_mismatch_negative():
    with pytest.raises(ValueError, match="equal length"):
        compact_graph_edges_event(
            brain_id="vlm_frontier_graph",
            from_ids=[1],
            to_ids=[2, 3],
            costs=[1.0],
        )


def test_vlm_choice_event_positive():
    ev = compact_vlm_choice_event(
        brain_id="vlm_choice_dijkstra",
        prompt="Which frontier next?",
        response="2 — open doorway",
        selected_frontier_id=2,
        candidate_ids=[1, 2, 3],
    )
    assert ev["topic"] == "exploration/vlm/choice"
    assert ev["selected_frontier_id"] == 2
    assert ev["candidate_ids"] == [1, 2, 3]


def test_vlm_choice_prompt_truncated_negative():
    ev = compact_vlm_choice_event(
        brain_id="vlm_choice_dijkstra",
        prompt="p" * 5000,
        response="r" * 5000,
        selected_frontier_id=0,
    )
    assert len(ev["prompt"]) == 2000
    assert len(ev["response"]) == 2000


def test_action_name_from_brain_action_positive():
    assert action_name_from_brain_action("kNavigateTo") == "navigate"
    assert action_name_from_brain_action("complete") == "complete"


def test_action_name_unknown_negative():
    with pytest.raises(ValueError, match="unknown brain action"):
        action_name_from_brain_action("fly")
