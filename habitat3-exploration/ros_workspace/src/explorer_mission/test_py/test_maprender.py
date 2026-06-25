"""Map render overlay tests (positive/negative controls)."""

from __future__ import annotations

import numpy as np
import pytest
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header


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
