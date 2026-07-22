#!/usr/bin/env python3
"""Publish the VLM view for the frontier currently being navigated toward."""

from __future__ import annotations

from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from explorer_msgs.msg import (
    ExplorationStatus,
    FrontierOpennessScores,
    FrontierViews,
)
from explorer_mission.frontier_debug_overlay import (
    FrontierOverlay,
    overlay_frontier_jpeg,
)


class CurrentFrontierViewNode(Node):
    def __init__(self) -> None:
        super().__init__("current_frontier_view")
        self._images: dict[int, CompressedImage] = {}
        self._scores: dict[int, int] = {}
        self._reasonings: dict[int, str] = {}
        self._target_id: Optional[int] = None
        self._phase: str = ""

        self._pub = self.create_publisher(
            CompressedImage, "exploration/debug/current_frontier_img", 1
        )
        self.create_subscription(
            FrontierViews, "exploration/vlm/views", self._views_cb, 1
        )
        self.create_subscription(
            FrontierOpennessScores, "exploration/vlm/scores", self._scores_cb, 1
        )
        self.create_subscription(
            ExplorationStatus, "exploration/status", self._status_cb, 1
        )
        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            "Current frontier debug view → /exploration/debug/current_frontier_img"
        )

    def _views_cb(self, msg: FrontierViews) -> None:
        for fid, image in zip(msg.frontier_ids, msg.images):
            self._images[int(fid)] = image

    def _scores_cb(self, msg: FrontierOpennessScores) -> None:
        for i, fid in enumerate(msg.frontier_ids):
            score = int(msg.scores[i]) if i < len(msg.scores) else 0
            reasoning = (
                str(msg.reasonings[i])
                if i < len(msg.reasonings)
                else ""
            )
            self._scores[int(fid)] = score
            self._reasonings[int(fid)] = reasoning

    def _status_cb(self, msg: ExplorationStatus) -> None:
        self._phase = str(msg.phase)
        self._target_id = int(msg.target_node_id) if msg.target_node_id else None
        self._publish_if_ready()

    def _tick(self) -> None:
        self._publish_if_ready()

    def _publish_if_ready(self) -> None:
        if self._phase != "navigating" or not self._target_id:
            return
        fid = int(self._target_id)
        image = self._images.get(fid)
        if image is None:
            return
        score = int(self._scores.get(fid, 255))
        reasoning = self._reasonings.get(fid, "")
        try:
            jpeg = overlay_frontier_jpeg(
                bytes(image.data),
                FrontierOverlay(score=score if score != 255 else -1, reasoning=reasoning),
            )
        except Exception as exc:
            self.get_logger().warn(f"Frontier overlay failed: {exc}", throttle_duration_sec=5.0)
            return
        out = CompressedImage()
        out.header = image.header
        out.format = "jpeg"
        out.data = jpeg
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CurrentFrontierViewNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
