"""Guards for ~1 m goal acceptance (mission-side, not Nav2 xy_goal_tolerance)."""

from __future__ import annotations

import math

import pytest


def within_goal_accept_radius(
    x: float, y: float, goal_x: float, goal_y: float, radius_m: float
) -> bool:
    if radius_m <= 0.0:
        return False
    return math.hypot(x - goal_x, y - goal_y) <= radius_m


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_accept_inside_1m_positive():
    assert within_goal_accept_radius(0.0, 0.0, 0.6, 0.6, 1.0)


def test_accept_outside_1m_negative():
    assert not within_goal_accept_radius(0.0, 0.0, 2.0, 0.0, 1.0)


def test_accept_zero_radius_negative():
    assert not within_goal_accept_radius(0.0, 0.0, 0.0, 0.0, 0.0)
