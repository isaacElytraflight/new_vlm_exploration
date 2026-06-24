"""Unit tests for image_utils (no ROS, no habitat)."""

import numpy as np
import pytest

from explorer_bridge.image_utils import (
    depth_array_to_image,
    image_to_depth_array,
    image_to_rgb_array,
    observation_shapes,
    rgb_array_to_image,
)


def test_rgb_roundtrip_shape_and_encoding():
    rgb = np.zeros((480, 640, 3), dtype=np.uint8)
    rgb[10, 20] = (1, 2, 3)
    msg = rgb_array_to_image(rgb)
    assert msg.encoding == "rgb8"
    assert msg.height == 480
    assert msg.width == 640
    assert msg.step == 640 * 3
    out = image_to_rgb_array(msg)
    assert out.shape == (480, 640, 3)
    assert tuple(out[10, 20]) == (1, 2, 3)


def test_depth_roundtrip_32fc1():
    depth = np.full((480, 640), 2.5, dtype=np.float32)
    depth[5, 5] = 9.75
    msg = depth_array_to_image(depth)
    assert msg.encoding == "32FC1"
    assert msg.height == 480
    assert msg.width == 640
    out = image_to_depth_array(msg)
    assert out.shape == (480, 640)
    assert out[5, 5] == pytest.approx(9.75)


def test_rgb_invalid_shape_raises():
    with pytest.raises(ValueError):
        rgb_array_to_image(np.zeros((480, 640), dtype=np.uint8))


def test_observation_shapes_helper():
    rgb = np.zeros((100, 200, 3), dtype=np.uint8)
    depth = np.zeros((100, 200), dtype=np.float32)
    assert observation_shapes(rgb, depth) == ((100, 200, 3), (100, 200))
