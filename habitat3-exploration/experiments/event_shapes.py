"""Compact JSONL event shapes for ablation packages (text-only, no images)."""

from __future__ import annotations

from typing import Any, Dict, Iterable, List, Sequence


def compact_status_event(phase: str, detail: str, **fields: Any) -> Dict[str, Any]:
    return {
        "topic": "exploration/status",
        "phase": phase,
        "detail": detail[:240],
        **fields,
    }


def compact_scores_event(
    frontier_ids: Iterable[int],
    scores: Iterable[int],
    reasonings: Iterable[str],
) -> Dict[str, Any]:
    return {
        "topic": "exploration/vlm/scores",
        "frontier_ids": list(frontier_ids),
        "scores": list(scores),
        "reasonings": [str(r)[:160] for r in reasonings],
    }


def compact_brain_decision_event(
    *,
    brain_id: str,
    action: str,
    goal_id: int = 0,
    goal_x: float = 0.0,
    goal_y: float = 0.0,
    detail: str = "",
    visited_ids: Sequence[int] | None = None,
    live_ids: Sequence[int] | None = None,
    robot_x: float = 0.0,
    robot_y: float = 0.0,
    goal_distance_m: float = 0.0,
) -> Dict[str, Any]:
    return {
        "topic": "exploration/brain/decision",
        "brain_id": str(brain_id),
        "action": str(action),
        "goal_id": int(goal_id),
        "goal_x": float(goal_x),
        "goal_y": float(goal_y),
        "detail": str(detail)[:240],
        "visited_ids": [int(i) for i in (visited_ids or ())],
        "live_ids": [int(i) for i in (live_ids or ())],
        "robot_x": float(robot_x),
        "robot_y": float(robot_y),
        "goal_distance_m": float(goal_distance_m),
    }


def compact_nav_fail_event(
    *,
    stage: str,
    fail_class: str = "",
    resolution: str = "",
    goal_id: int = 0,
    goal_x: float = 0.0,
    goal_y: float = 0.0,
    prior_id: int = 0,
    robot_x: float = 0.0,
    robot_y: float = 0.0,
    nav_dt_s: float = 0.0,
    nav_error_code: int = 0,
    nav_error: str = "",
    prior_pose_known: bool = False,
    prior_plan_ok: bool = False,
    new_goal_plan_ok: bool = False,
    new_plan_code: int = 0,
    new_plan_err: str = "",
    definitive_unreachable: bool = False,
    start_clearance_m: float = 0.0,
    start_clearance_ok: bool = False,
    grid_occ_at_robot: int = -128,
    grid_occ_at_goal: int = -128,
    costmap_at_robot: int = -1,
    costmap_at_goal: int = -1,
) -> Dict[str, Any]:
    return {
        "topic": "exploration/nav_fail",
        "stage": str(stage)[:32],
        "fail_class": str(fail_class)[:32],
        "resolution": str(resolution)[:64],
        "goal_id": int(goal_id),
        "goal_x": float(goal_x),
        "goal_y": float(goal_y),
        "prior_id": int(prior_id),
        "robot_x": float(robot_x),
        "robot_y": float(robot_y),
        "nav_dt_s": float(nav_dt_s),
        "nav_error_code": int(nav_error_code),
        "nav_error": str(nav_error)[:240],
        "prior_pose_known": bool(prior_pose_known),
        "prior_plan_ok": bool(prior_plan_ok),
        "new_goal_plan_ok": bool(new_goal_plan_ok),
        "new_plan_code": int(new_plan_code),
        "new_plan_err": str(new_plan_err)[:240],
        "definitive_unreachable": bool(definitive_unreachable),
        "start_clearance_m": float(start_clearance_m),
        "start_clearance_ok": bool(start_clearance_ok),
        "grid_occ_at_robot": int(grid_occ_at_robot),
        "grid_occ_at_goal": int(grid_occ_at_goal),
        "costmap_at_robot": int(costmap_at_robot),
        "costmap_at_goal": int(costmap_at_goal),
    }


def compact_graph_edges_event(
    *,
    brain_id: str,
    from_ids: Sequence[int],
    to_ids: Sequence[int],
    costs: Sequence[float],
) -> Dict[str, Any]:
    if not (len(from_ids) == len(to_ids) == len(costs)):
        raise ValueError("from_ids, to_ids, and costs must have equal length")
    return {
        "topic": "exploration/brain/graph_edges",
        "brain_id": str(brain_id),
        "from_ids": [int(i) for i in from_ids],
        "to_ids": [int(i) for i in to_ids],
        "costs": [float(c) for c in costs],
    }


def compact_vlm_choice_event(
    *,
    brain_id: str,
    prompt: str,
    response: str,
    selected_frontier_id: int,
    candidate_ids: Sequence[int] | None = None,
) -> Dict[str, Any]:
    return {
        "topic": "exploration/vlm/choice",
        "brain_id": str(brain_id),
        "prompt": str(prompt)[:2000],
        "response": str(response)[:2000],
        "selected_frontier_id": int(selected_frontier_id),
        "candidate_ids": [int(i) for i in (candidate_ids or ())],
    }


def action_name_from_brain_action(action: str) -> str:
    """Normalize BrainAction-like labels to JSONL action strings."""
    key = str(action).strip().lower()
    mapping = {
        "knavigateto": "navigate",
        "navigate": "navigate",
        "kwait": "wait",
        "wait": "wait",
        "kcomplete": "complete",
        "complete": "complete",
    }
    if key not in mapping:
        raise ValueError(f"unknown brain action: {action!r}")
    return mapping[key]
