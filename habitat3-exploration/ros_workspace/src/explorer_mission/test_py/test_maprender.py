"""Map render overlay tests (positive/negative controls)."""

from __future__ import annotations

import numpy as np
import pytest
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

from explorer_mission.maprender_node import (
    NOT_RATED,
    occupancy_to_bgr,
    openness_label_text,
)


def _make_grid(width: int = 20, height: int = 20) -> OccupancyGrid:
    msg = OccupancyGrid()
    msg.header = Header(frame_id="map")
    msg.info.resolution = 0.05
    msg.info.width = width
    msg.info.height = height
    msg.info.origin.position.x = 0.0
    msg.info.origin.position.y = 0.0
    msg.info.origin.orientation.w = 1.0
    data = np.zeros((height, width), dtype=np.int8)
    data[0, :] = 100
    msg.data = data.flatten().tolist()
    return msg


def test_harness_positive():
    assert 1 + 1 == 2


def test_harness_negative():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_occupancy_grid_has_obstacle_border_positive():
    grid = _make_grid()
    arr = np.asarray(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)
    assert arr[0, 0] == 100
    assert arr[10, 10] == 0


def test_mismatched_grid_shape_negative():
    grid = OccupancyGrid()
    grid.header = Header(frame_id="map")
    grid.info.width = 2
    grid.info.height = 2
    grid.data = [0, 0, 0]  # wrong length
    with pytest.raises(ValueError):
        np.asarray(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)


def test_openness_label_0_to_5_positive():
    for score in range(6):
        assert openness_label_text(score) == str(score)


def test_openness_label_not_rated_negative():
    assert openness_label_text(NOT_RATED) is None


def test_occupancy_to_bgr_is_grayscale_positive():
    """Grid-only render uses gray/black/white — no chroma overlays."""
    grid = np.zeros((10, 10), dtype=np.int8)
    grid[0, :] = 100
    grid[5, 5] = -1
    bgr = occupancy_to_bgr(grid)
    # Every pixel channel-equal (true gray).
    assert np.all(bgr[:, :, 0] == bgr[:, :, 1])
    assert np.all(bgr[:, :, 1] == bgr[:, :, 2])


def test_occupancy_to_bgr_empty_negative():
    bgr = occupancy_to_bgr(np.zeros((0, 0), dtype=np.int8))
    assert bgr.size == 0
