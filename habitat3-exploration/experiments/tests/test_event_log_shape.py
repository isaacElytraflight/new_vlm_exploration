"""Tests for compact event payload shaping (no ROS)."""

from __future__ import annotations

import json


def compact_status_event(phase: str, detail: str, **fields) -> dict:
    return {
        "topic": "exploration/status",
        "phase": phase,
        "detail": detail[:240],
        **fields,
    }


def compact_scores_event(frontier_ids, scores, reasonings) -> dict:
    return {
        "topic": "exploration/vlm/scores",
        "frontier_ids": list(frontier_ids),
        "scores": list(scores),
        "reasonings": [str(r)[:160] for r in reasonings],
    }


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
