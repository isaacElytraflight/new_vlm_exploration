"""Tests for cmd_vel -> discrete move translation logic."""

from __future__ import annotations

import pytest
from explorer_msgs.action import DiscreteMove

from explorer_bridge.cmd_vel_to_discrete import (
    apply_realtime_rate_cap,
    cmd_vel_to_intent,
)


def test_harness_positive_control():
    assert 1 + 1 == 2


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_cmd_vel_turn_left_positive():
    intent = cmd_vel_to_intent(0.0, 0.5)
    assert intent is not None
    assert intent.direction == DiscreteMove.Goal.TURN_LEFT
    assert intent.steps == 1


def test_cmd_vel_slow_rotate_accepted_positive():
    """Nav2 RPP often emits ~0.1 rad/s while aligning; must still discrete-turn."""
    intent = cmd_vel_to_intent(0.0, 0.1)
    assert intent is not None
    assert intent.direction == DiscreteMove.Goal.TURN_LEFT


def test_cmd_vel_forward_positive():
    intent = cmd_vel_to_intent(0.2, 0.0)
    assert intent is not None
    assert intent.direction == DiscreteMove.Goal.FORWARD


def test_cmd_vel_drive_with_curvature_prefers_forward_positive():
    """RPP path follow: linear + mild angular must step forward, not spin-jitter."""
    intent = cmd_vel_to_intent(0.25, 0.12)
    assert intent is not None
    assert intent.direction == DiscreteMove.Goal.FORWARD


def test_cmd_vel_curvature_must_not_force_turn_negative():
    """Negative: old angular-first priority caused align-then-step jitter."""
    intent = cmd_vel_to_intent(0.25, 0.12)
    assert intent is not None
    assert intent.direction != DiscreteMove.Goal.TURN_LEFT
    assert intent.direction != DiscreteMove.Goal.TURN_RIGHT


def test_cmd_vel_below_threshold_negative():
    intent = cmd_vel_to_intent(0.01, 0.01)
    assert intent is None


def test_turn_hysteresis_holds_direction_positive():
    """Once turning left, weak opposite angular must not flip (Habitat 10° twitch)."""
    left = DiscreteMove.Goal.TURN_LEFT
    intent = cmd_vel_to_intent(0.0, -0.1, last_turn_direction=left)
    assert intent is not None
    assert intent.direction == left


def test_turn_hysteresis_allows_strong_flip_positive():
    """Strong opposite angular may reverse (real heading-error sign change)."""
    left = DiscreteMove.Goal.TURN_LEFT
    intent = cmd_vel_to_intent(0.0, -0.4, last_turn_direction=left)
    assert intent is not None
    assert intent.direction == DiscreteMove.Goal.TURN_RIGHT


def test_turn_hysteresis_weak_flip_rejected_negative():
    """Negative: ±0.1 left/right flip is the observed yaw twitch failure mode."""
    left = DiscreteMove.Goal.TURN_LEFT
    intent = cmd_vel_to_intent(0.0, -0.1, last_turn_direction=left)
    assert intent is not None
    assert intent.direction != DiscreteMove.Goal.TURN_RIGHT


def test_realtime_rate_cap_positive():
    lin, ang = apply_realtime_rate_cap(
        1.0, 2.0, max_linear_m_s=0.1, max_angular_deg_s=30.0
    )
    assert lin == pytest.approx(0.1)
    assert ang == pytest.approx(30.0 * 3.141592653589793 / 180.0)


def test_realtime_rate_cap_negative():
    lin, ang = apply_realtime_rate_cap(
        0.0, 0.0, max_linear_m_s=0.1, max_angular_deg_s=30.0
    )
    assert lin == 0.0
    assert ang == 0.0
