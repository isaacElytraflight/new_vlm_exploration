#!/usr/bin/env python3
"""Publish a JPEG HUD of /exploration/status for the Elytra dashboard."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

from explorer_msgs.msg import ExplorationStatus
from explorer_mission.exploration_status_hud import (
    StatusEvent,
    append_status_event,
    render_status_hud_jpeg,
)


class ExplorationStatusHudNode(Node):
    def __init__(self) -> None:
        super().__init__("exploration_status_hud")
        self.declare_parameter("status_topic", "exploration/status")
        self.declare_parameter("image_topic", "/exploration/debug/status_img")
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("log_maxlen", 12)
        self.declare_parameter("chart_width", 640)
        self.declare_parameter("chart_height", 360)

        status_topic = str(self.get_parameter("status_topic").value)
        image_topic = str(self.get_parameter("image_topic").value)
        publish_hz = max(0.5, float(self.get_parameter("publish_hz").value))
        self._maxlen = max(1, int(self.get_parameter("log_maxlen").value))
        self._width = int(self.get_parameter("chart_width").value)
        self._height = int(self.get_parameter("chart_height").value)

        self._log: list[StatusEvent] = []
        self._pub = self.create_publisher(CompressedImage, image_topic, 1)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ExplorationStatus, status_topic, self._status_cb, status_qos)
        self.create_timer(1.0 / publish_hz, self._tick)
        self.get_logger().info(f"Exploration status HUD → {image_topic}")

    def _stamp_sec(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _now_sec(self) -> float:
        t = self.get_clock().now()
        return float(t.nanoseconds) * 1e-9

    def _status_cb(self, msg: ExplorationStatus) -> None:
        t = self._stamp_sec(msg.header.stamp)
        if t <= 0.0:
            t = self._now_sec()
        append_status_event(
            self._log,
            StatusEvent(
                t_sec=t,
                phase=str(msg.phase),
                detail=str(msg.detail),
                current_node_id=int(msg.current_node_id),
                target_node_id=int(msg.target_node_id),
                complete=bool(msg.exploration_complete),
            ),
            maxlen=self._maxlen,
        )
        self._publish(now_sec=t)

    def _tick(self) -> None:
        self._publish(now_sec=self._now_sec())

    def _publish(self, *, now_sec: float) -> None:
        try:
            jpeg = render_status_hud_jpeg(
                self._log,
                now_sec=now_sec,
                width=self._width,
                height=self._height,
            )
        except Exception as exc:
            self.get_logger().warn(f"Status HUD render failed: {exc}", throttle_duration_sec=5.0)
            return
        out = CompressedImage()
        out.header = Header()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.format = "jpeg"
        out.data = jpeg
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorationStatusHudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
