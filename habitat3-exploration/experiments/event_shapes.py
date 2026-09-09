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
