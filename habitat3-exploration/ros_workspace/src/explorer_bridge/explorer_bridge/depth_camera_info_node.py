#!/usr/bin/env python3
"""Publish CameraInfo synced to each /depth_data frame (depthimage_to_laserscan input)."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class DepthCameraInfoNode(Node):
    def __init__(self, **node_kwargs) -> None:
        super().__init__("depth_camera_info", **node_kwargs)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("frame_id", "depth_frame")
        self.declare_parameter("depth_topic", "/depth_data")
        self.declare_parameter("topic", "/depth/camera_info")

        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        frame_id = str(self.get_parameter("frame_id").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        topic = str(self.get_parameter("topic").value)

        # ~90 deg HFOV pinhole model for Habitat 640x480 depth.
        fx = fy = width / 2.0
        cx = width / 2.0
        cy = height / 2.0

        self._template = CameraInfo()
        self._template.width = width
        self._template.height = height
        self._template.distortion_model = "plumb_bob"
        self._template.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._template.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self._template.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self._template.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._default_frame_id = frame_id

        self._pub = self.create_publisher(CameraInfo, topic, 10)
        self.create_subscription(
            Image,
            depth_topic,
            self._depth_cb,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Publishing {topic} synced to {depth_topic} ({width}x{height}, frame={frame_id})"
        )

    def _depth_cb(self, depth: Image) -> None:
        msg = CameraInfo()
        msg.header = depth.header
        if not msg.header.frame_id:
            msg.header.frame_id = self._default_frame_id
        msg.width = self._template.width
        msg.height = self._template.height
        msg.distortion_model = self._template.distortion_model
        msg.d = list(self._template.d)
        msg.k = list(self._template.k)
        msg.r = list(self._template.r)
        msg.p = list(self._template.p)
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthCameraInfoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
