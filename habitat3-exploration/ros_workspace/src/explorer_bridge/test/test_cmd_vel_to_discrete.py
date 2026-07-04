"""Tests for cmd_vel -> discrete move translation logic."""

from __future__ import annotations

import pytest
from explorer_msgs.action import DiscreteMove

from explorer_bridge.cmd_vel_to_discrete import (
    CmdVelThresholds,
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


def test_cmd_vel_forward_positive():
    intent = cmd_vel_to_intent(0.2, 0.0)
    assert intent is not None
    assert intent.direction == DiscreteMove.Goal.FORWARD


def test_cmd_vel_below_threshold_negative():
    intent = cmd_vel_to_intent(0.01, 0.01)
    assert intent is None


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
