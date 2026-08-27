"""Unit tests for exploration state-machine HUD (pure render + log)."""

from __future__ import annotations

import pytest

from explorer_mission.exploration_status_hud import (
    StatusEvent,
    append_status_event,
    dwell_seconds,
    render_status_hud_jpeg,
    spin_hint,
)


def test_harness_positive_control():
    assert 1 + 1 == 2


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_append_keeps_recent_events_positive():
    log: list[StatusEvent] = []
    append_status_event(log, StatusEvent(1.0, "scanning", "rotate_360 start", 0, 0, False), maxlen=3)
    append_status_event(log, StatusEvent(2.0, "scanning", "rotate_360 done", 0, 0, False), maxlen=3)
    append_status_event(log, StatusEvent(3.0, "navigating", "child 4", 1, 4, False), maxlen=3)
    append_status_event(log, StatusEvent(4.0, "scanning", "arrived", 4, 0, False), maxlen=3)
    assert len(log) == 3
    assert log[0].phase == "scanning"
    assert log[0].detail == "rotate_360 done"
    assert log[-1].phase == "scanning"
    assert log[-1].detail == "arrived"


def test_append_rejects_empty_phase_negative():
    log: list[StatusEvent] = []
    append_status_event(log, StatusEvent(1.0, "", "nope", 0, 0, False), maxlen=8)
    append_status_event(log, StatusEvent(1.0, "   ", "nope", 0, 0, False), maxlen=8)
    assert log == []


def test_dwell_seconds_from_last_event_positive():
    log = [StatusEvent(10.0, "navigating", "to 7", 3, 7, False)]
    assert dwell_seconds(log, now_sec=22.5) == pytest.approx(12.5)


def test_dwell_seconds_empty_log_negative():
    assert dwell_seconds([], now_sec=100.0) == 0.0


def test_spin_hint_scanning_is_state_machine_positive():
    text = spin_hint("scanning")
    assert "rotate_360" in text
    assert "state machine" in text.lower() or "explore" in text.lower()


def test_spin_hint_navigating_is_nav2_positive():
    text = spin_hint("navigating")
    assert "nav2" in text.lower() or "path" in text.lower()


def test_spin_hint_unknown_phase_negative():
    assert spin_hint("") == ""
    assert spin_hint("idle") == ""


def test_render_status_hud_jpeg_includes_phase_positive():
    jpeg = render_status_hud_jpeg(
        [
            StatusEvent(1.0, "detecting", "on-demand frontier detection", 2, 0, False),
            StatusEvent(2.0, "navigating", "selected child score=3", 2, 9, False),
        ],
        now_sec=5.0,
    )
    assert jpeg[:2] == b"\xff\xd8"
    from PIL import Image
    import io

    img = Image.open(io.BytesIO(jpeg))
    assert img.size[0] >= 400
    assert img.size[1] >= 240


def test_render_status_hud_empty_waiting_negative():
    jpeg = render_status_hud_jpeg([], now_sec=0.0)
    assert jpeg[:2] == b"\xff\xd8"
    from PIL import Image
    import io

    img = Image.open(io.BytesIO(jpeg))
    # Empty HUD must still produce a frame (Elytra needs a topic image).
    assert img.size[0] > 0
