"""Convert between numpy arrays and sensor_msgs/Image."""

from __future__ import annotations

import struct
from typing import Tuple

import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def rgb_array_to_image(
    rgb: np.ndarray,
    header: Header | None = None,
    frame_id: str = "camera",
) -> Image:
    """Encode HxWx3 uint8 RGB array as sensor_msgs/Image (rgb8)."""
    if rgb.ndim != 3 or rgb.shape[2] != 3:
        raise ValueError("rgb must have shape (H, W, 3)")
    if rgb.dtype != np.uint8:
        rgb = rgb.astype(np.uint8, copy=False)

    msg = Image()
    msg.header = header if header is not None else Header()
    if msg.header.frame_id == "":
        msg.header.frame_id = frame_id
    msg.height = int(rgb.shape[0])
    msg.width = int(rgb.shape[1])
    msg.encoding = "rgb8"
    msg.is_bigendian = False
    msg.step = msg.width * 3
    msg.data = rgb.tobytes()
    return msg


def depth_array_to_image(
    depth: np.ndarray,
    header: Header | None = None,
    frame_id: str = "lidar_depth",
) -> Image:
    """Encode HxW float32 depth (meters) as sensor_msgs/Image (32FC1)."""
    if depth.ndim != 2:
        raise ValueError("depth must have shape (H, W)")
    depth_f = np.ascontiguousarray(depth, dtype=np.float32)

    msg = Image()
    msg.header = header if header is not None else Header()
    if msg.header.frame_id == "":
        msg.header.frame_id = frame_id
    msg.height = int(depth_f.shape[0])
    msg.width = int(depth_f.shape[1])
    msg.encoding = "32FC1"
    msg.is_bigendian = False
    msg.step = msg.width * 4
    msg.data = depth_f.tobytes()
    return msg


def image_to_rgb_array(msg: Image) -> np.ndarray:
    """Decode rgb8 Image to HxWx3 uint8 array."""
    if msg.encoding != "rgb8":
        raise ValueError(f"expected rgb8, got {msg.encoding}")
    expected = msg.height * msg.width * 3
    if len(msg.data) != expected:
        raise ValueError(f"rgb8 data length {len(msg.data)} != {expected}")
    return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)


def image_to_depth_array(msg: Image) -> np.ndarray:
    """Decode 32FC1 Image to HxW float32 array."""
    if msg.encoding != "32FC1":
        raise ValueError(f"expected 32FC1, got {msg.encoding}")
    expected = msg.height * msg.width * 4
    if len(msg.data) != expected:
        raise ValueError(f"32FC1 data length {len(msg.data)} != {expected}")
    return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)


def write_jpeg_frame(path: str, rgb: np.ndarray, quality: int = 85) -> None:
    """Write RGB frame to JPEG for noVNC live viewer (Pillow, imageio fallback)."""
    import os

    if rgb.ndim == 3 and rgb.shape[2] >= 3:
        rgb = rgb[..., :3]
    rgb = np.ascontiguousarray(rgb, dtype=np.uint8)
    tmp = path.replace(".jpg", ".tmp.jpg") if path.endswith(".jpg") else path + ".tmp"
    try:
        from PIL import Image as PILImage

        PILImage.fromarray(rgb, "RGB").save(tmp, "JPEG", quality=quality)
    except Exception:
        import imageio.v2 as imageio

        imageio.imwrite(tmp, rgb, quality=quality)
    os.replace(tmp, path)


def observation_shapes(rgb: np.ndarray, depth: np.ndarray) -> Tuple[Tuple[int, ...], Tuple[int, ...]]:
    """Return (rgb_shape, depth_shape) for validation."""
    return tuple(rgb.shape), tuple(depth.shape)
