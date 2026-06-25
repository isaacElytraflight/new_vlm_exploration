"""Tests for habitat IPC get_pose / get_map extensions."""

from __future__ import annotations

import base64
import json
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from explorer_bridge.habitat_ipc import HabitatIpcClient, HabitatIpcError
from explorer_bridge.mock_driver import MockHabitatDriver


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
