#!/usr/bin/env python3
"""Publish static CameraInfo for Habitat depth (depthimage_to_laserscan input)."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo


class DepthCameraInfoNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_camera_info")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("frame_id", "depth_frame")
        self.declare_parameter("topic", "/depth/camera_info")
        self.declare_parameter("publish_hz", 10.0)

        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        frame_id = str(self.get_parameter("frame_id").value)
        topic = str(self.get_parameter("topic").value)
        publish_hz = max(0.5, float(self.get_parameter("publish_hz").value))

        # ~90 deg HFOV pinhole model for Habitat 640x480 depth.
        fx = fy = width / 2.0
        cx = width / 2.0
        cy = height / 2.0

        self._msg = CameraInfo()
        self._msg.width = width
        self._msg.height = height
        self._msg.distortion_model = "plumb_bob"
        self._msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        self._msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self._msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self._msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        self._frame_id = frame_id

        self._pub = self.create_publisher(CameraInfo, topic, 10)
        self.create_timer(1.0 / publish_hz, self._publish)
        self.get_logger().info(f"Publishing {topic} ({width}x{height}, frame={frame_id})")

    def _publish(self) -> None:
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._msg.header.frame_id = self._frame_id
        self._pub.publish(self._msg)


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
