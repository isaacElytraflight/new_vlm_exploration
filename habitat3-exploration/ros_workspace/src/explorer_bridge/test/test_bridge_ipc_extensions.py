"""Tests for habitat IPC get_pose / get_map extensions."""

from __future__ import annotations

import base64
import json
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from explorer_bridge.habitat_ipc import HabitatIpcClient, HabitatIpcError
from explorer_bridge.mock_driver import MockHabitatDriver
from explorer_bridge.explorer_bridge_node import TELEOP_NAME_TO_ACTION


def test_get_pose_roundtrip():
    driver = MockHabitatDriver()
    pose = driver.get_pose()
    assert pose.x == 0.0
    assert pose.y == 0.0
    driver.step("move_forward", 1)
    pose2 = driver.get_pose()
    assert pose2.x > pose.x


def test_get_map_roundtrip():
    driver = MockHabitatDriver()
    map_data = driver.get_map()
    assert map_data.grid.shape == (10, 10)
    assert map_data.resolution == 0.05


def test_ipc_unknown_cmd_error():
    client = HabitatIpcClient()
    with patch.object(
        client,
        "_request",
        side_effect=HabitatIpcError("unknown cmd 'foo'"),
    ):
        with pytest.raises(HabitatIpcError, match="unknown cmd"):
            client.get_pose()


def test_ipc_get_map_decode():
    grid = np.array([[0, 100], [0, 0]], dtype=np.int8)
    payload = {
        "ok": True,
        "grid_b64": base64.b64encode(grid.tobytes()).decode("ascii"),
        "grid_shape": [2, 2],
        "resolution": 0.05,
        "origin_x": 1.0,
        "origin_y": 2.0,
    }
    client = HabitatIpcClient()
    with patch.object(client, "_request", return_value=payload):
        map_data = client.get_map()
    assert map_data.grid.shape == (2, 2)
    assert map_data.origin_x == 1.0


def test_ipc_get_obs_and_pose_decodes_positive():
    rgb = np.zeros((2, 2, 3), dtype=np.uint8)
    depth = np.ones((2, 2), dtype=np.float32)
    payload = {
        "ok": True,
        "rgb_b64": base64.b64encode(rgb.tobytes()).decode("ascii"),
        "rgb_shape": [2, 2, 3],
        "depth_b64": base64.b64encode(depth.tobytes()).decode("ascii"),
        "depth_shape": [2, 2],
        "x": 1.5,
        "y": -2.0,
        "yaw_rad": 0.25,
    }
    client = HabitatIpcClient()
    with patch.object(client, "_request", return_value=payload) as req:
        obs, pose = client.get_observations_with_pose()
    req.assert_called_once_with({"cmd": "get_obs_and_pose"})
    assert pose.x == 1.5
    assert pose.y == -2.0
    assert pose.yaw_rad == 0.25
    assert obs.depth.shape == (2, 2)


def test_mock_get_observations_with_pose_positive():
    driver = MockHabitatDriver()
    obs, pose = driver.get_observations_with_pose()
    assert obs.depth.shape[0] > 0
    assert pose.x == 0.0


def test_teleop_name_mapping_positive():
    assert TELEOP_NAME_TO_ACTION["forward"] == "move_forward"
    assert TELEOP_NAME_TO_ACTION["turn_left"] == "turn_left"


def test_teleop_name_mapping_unknown_negative():
    assert "strafe" not in TELEOP_NAME_TO_ACTION


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
