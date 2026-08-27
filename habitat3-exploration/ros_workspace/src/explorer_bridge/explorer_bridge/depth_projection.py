"""Depth pixel → 3D point in base_link (testable without ROS)."""

from __future__ import annotations

import math

# Camera mount height above base_link (matches static TF in nav2_exploration.launch).
DEFAULT_CAMERA_Z_M = 0.1


def pixel_to_base_link_xyz(
    col: int,
    row: int,
    depth: float,
    *,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    camera_z: float = DEFAULT_CAMERA_Z_M,
) -> tuple[float, float, float] | None:
    """Project a depth pixel to base_link (x forward, y left, z up).

    Habitat depth is camera-frame **Z** (optical-axis depth). Pinhole::

        P_cam = ((u-cx)/fx * Z, (v-cy)/fy * Z, Z)
        x_bl, y_bl, z_bl = Z, -P_cam.x, -P_cam.y + camera_z
    """
    if not math.isfinite(depth) or depth <= 0.0:
        return None
    z_c = depth
    x_c = (col - cx) / fx * z_c
    y_c = (row - cy) / fy * z_c
    x_bl = z_c
    y_bl = -x_c
    z_bl = -y_c + camera_z
    if x_bl <= 0.01:
        return None
    return x_bl, y_bl, z_bl


def base_link_xy_to_map(
    x_bl: float,
    y_bl: float,
    *,
    robot_x: float,
    robot_y: float,
    yaw: float,
) -> tuple[float, float]:
    """Rotate base_link horizontal coords into the map frame."""
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        robot_x + x_bl * c - y_bl * s,
        robot_y + x_bl * s + y_bl * c,
    )
