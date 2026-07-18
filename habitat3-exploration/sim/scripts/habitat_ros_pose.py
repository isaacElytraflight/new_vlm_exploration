"""Convert Habitat agent pose (Y-up, camera -Z) into ROS plan pose (X forward, Y left)."""

from __future__ import annotations

import math


def habitat_agent_to_ros_pose(
    pos_x: float,
    pos_z: float,
    forward_x: float,
    forward_z: float,
) -> tuple[float, float, float]:
    """Return (ros_x, ros_y, yaw_rad) for mapping / Nav2 / known-pose scan integration.

    Habitat agents spawn looking along -Z. ROS base_link +X must match that
    heading at yaw=0, and turn_left (CCW from above) must increase yaw.
    """
    ros_x = -float(pos_z)
    ros_y = -float(pos_x)
    yaw = math.atan2(-float(forward_x), -float(forward_z))
    return ros_x, ros_y, yaw
