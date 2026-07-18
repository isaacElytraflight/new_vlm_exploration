"""Tests for Habitat → ROS planar pose conversion (spiral-map regression)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from habitat_ros_pose import habitat_agent_to_ros_pose


def test_spawn_facing_neg_z_is_yaw_zero_positive():
    x, y, yaw = habitat_agent_to_ros_pose(0.0, 0.0, forward_x=0.0, forward_z=-1.0)
    assert x == pytest.approx(0.0)
    assert y == pytest.approx(0.0)
    assert yaw == pytest.approx(0.0)


def test_turn_left_to_neg_x_increases_yaw_positive():
    """Facing -Z then CCW to -X must be +90° in ROS (not -90°)."""
    _, _, yaw = habitat_agent_to_ros_pose(0.0, 0.0, forward_x=-1.0, forward_z=0.0)
    assert yaw == pytest.approx(math.pi / 2.0)


def test_old_atan2_sign_would_negate_left_turn_negative():
    """Negative control: legacy atan2(forward.x, -forward.z) flips turn_left."""
    legacy = math.atan2(-1.0, -0.0)  # facing -X with old formula → -pi/2
    assert legacy == pytest.approx(-math.pi / 2.0)
    _, _, fixed = habitat_agent_to_ros_pose(0.0, 0.0, forward_x=-1.0, forward_z=0.0)
    assert fixed == pytest.approx(-legacy)


def test_move_along_neg_z_increases_ros_x_positive():
    x, y, _ = habitat_agent_to_ros_pose(0.0, -1.5, forward_x=0.0, forward_z=-1.0)
    assert x == pytest.approx(1.5)
    assert y == pytest.approx(0.0)


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
