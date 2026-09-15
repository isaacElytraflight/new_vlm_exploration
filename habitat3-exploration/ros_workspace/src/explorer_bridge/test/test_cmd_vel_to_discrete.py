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


def test_cmd_vel_slow_rotate_accepted_positive():
    """Nav2 RPP often emits ~0.1 rad/s while aligning; must still discrete-turn."""
    intent = cmd_vel_to_intent(0.0, 0.1)
    assert intent is not None
    assert intent.direction == DiscreteMove.Goal.TURN_LEFT


def test_cmd_vel_forward_straight_positive():
    """Straight path follow: linear with zero angular → FORWARD."""
    intent = cmd_vel_to_intent(0.25, 0.0)
    assert intent is not None
    assert intent.direction == DiscreteMove.Goal.FORWARD


def test_cmd_vel_mild_corner_prefers_turn_positive():
    """Mild RPP corner arc must TURN, not 0.25 m FORWARD into walls.

    |ang|/|lin| = 0.12/0.25 = 0.48 < old ratio=1.0 (drove) but hits
    drive_max_angular=0.12 heading gate → TURN.
    """
    intent = cmd_vel_to_intent(0.25, 0.12)
    assert intent is not None
    assert intent.direction == DiscreteMove.Goal.TURN_LEFT


def test_cmd_vel_mild_corner_must_not_drive_negative():
    """Negative: ratio=1.0 + no heading gate used to plow FORWARD on corners."""
    intent = cmd_vel_to_intent(0.25, 0.12)
    assert intent is not None
    assert intent.direction != DiscreteMove.Goal.FORWARD
    assert intent.direction != DiscreteMove.Goal.BACKWARD


def test_cmd_vel_high_curvature_prefers_turn_positive():
    """Wall-hit mode: strong |ang|/|lin| must turn instead of driving into obstacles."""
    # Observed stuck case: lin=0.25, ang≈0.47 → ratio ≈ 1.89
    intent = cmd_vel_to_intent(0.25, 0.47)
    assert intent is not None
    assert intent.direction == DiscreteMove.Goal.TURN_LEFT


def test_cmd_vel_ratio_and_gate_below_stays_forward_negative():
    """Negative: low curvature AND aligned heading must still drive (anti spin-jitter)."""
    # 0.08/0.25 = 0.32 < 0.5 and |ang| 0.08 < drive_max_angular 0.12 → FORWARD
    intent = cmd_vel_to_intent(0.25, 0.08)
    assert intent is not None
    assert intent.direction == DiscreteMove.Goal.FORWARD


def test_cmd_vel_ratio_alone_triggers_turn_positive():
    """Curvature above turn_over_drive_ratio turns even if under drive_max_angular."""
    # Override gate high so only ratio matters: 0.15/0.25 = 0.6 >= 0.5
    t = CmdVelThresholds(drive_max_angular=1.0)
    intent = cmd_vel_to_intent(0.25, 0.15, thresholds=t)
    assert intent is not None
    assert intent.direction == DiscreteMove.Goal.TURN_LEFT


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
    intent = cmd_vel_to_intent(0.0, -0.1, last_turn_direction=left, consecutive_turn_steps=3)
    assert intent is not None
    assert intent.direction != DiscreteMove.Goal.TURN_RIGHT


def test_turn_hysteresis_allows_flip_after_180deg_positive():
    """After ~180° committed one way, weak opposite may take the short path."""
    left = DiscreteMove.Goal.TURN_LEFT
    intent = cmd_vel_to_intent(
        0.0, -0.1, last_turn_direction=left, consecutive_turn_steps=18
    )
    assert intent is not None
    assert intent.direction == DiscreteMove.Goal.TURN_RIGHT


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
