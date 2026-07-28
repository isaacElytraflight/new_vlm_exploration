#!/usr/bin/env python3
"""Publish coverage-vs-distance JPEG chart from /odom + /grid_map + Habitat GT."""

from __future__ import annotations

from typing import Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray

from explorer_bridge.habitat_ipc import DEFAULT_SOCKET_PATH, HabitatIpcClient, HabitatIpcError
from explorer_mission.coverage_metrics import (
    CoverageSampleRing,
    coverage_ratio,
    integrate_path_meters,
    mapped_area_m2,
    render_coverage_chart_jpeg,
)


class CoverageMetricsNode(Node):
    def __init__(self) -> None:
        super().__init__("coverage_metrics")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("grid_topic", "/grid_map")
        self.declare_parameter("image_topic", "/exploration/debug/coverage_vs_distance_img")
        self.declare_parameter("socket_path", DEFAULT_SOCKET_PATH)
        self.declare_parameter("gt_floor_area_m2", -1.0)  # <0 → fetch via IPC
        self.declare_parameter("ring_maxlen", 2048)
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("chart_width", 640)
        self.declare_parameter("chart_height", 360)

        odom_topic = str(self.get_parameter("odom_topic").value)
        grid_topic = str(self.get_parameter("grid_topic").value)
        image_topic = str(self.get_parameter("image_topic").value)
        self._socket_path = str(self.get_parameter("socket_path").value)
        gt_param = float(self.get_parameter("gt_floor_area_m2").value)
        maxlen = int(self.get_parameter("ring_maxlen").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        self._chart_w = int(self.get_parameter("chart_width").value)
        self._chart_h = int(self.get_parameter("chart_height").value)

        self._meters = 0.0
        self._prev_xy: Optional[Tuple[float, float]] = None
        self._mapped_m2 = 0.0
        self._gt_m2 = gt_param if gt_param > 0.0 else 0.0
        self._gt_fetched = gt_param > 0.0
        self._ring = CoverageSampleRing(maxlen=max(1, maxlen))

        self._img_pub = self.create_publisher(CompressedImage, image_topic, 1)
        self._snap_pub = self.create_publisher(
            Float32MultiArray, "exploration/debug/coverage_snapshot", 1
        )
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(OccupancyGrid, grid_topic, self._grid_cb, 1)
        period = 1.0 / publish_hz if publish_hz > 0.0 else 0.5
        self.create_timer(period, self._tick)
        self.create_timer(1.0, self._try_fetch_gt)

        self.get_logger().info(
            f"Coverage metrics → {image_topic} (gt={'param' if self._gt_fetched else 'pending IPC'})"
        )

    def _try_fetch_gt(self) -> None:
        if self._gt_fetched:
            return
        try:
            area, _mpp = HabitatIpcClient(self._socket_path).get_floor_area()
        except HabitatIpcError as exc:
            self.get_logger().warn(
                f"get_floor_area IPC failed: {exc}", throttle_duration_sec=10.0
            )
            return
        self._gt_m2 = float(area)
        self._gt_fetched = True
        self.get_logger().info(f"GT mappable area (floor+walls) = {self._gt_m2:.2f} m²")

    def _odom_cb(self, msg: Odometry) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self._meters, self._prev_xy = integrate_path_meters(
            self._meters, self._prev_xy, x, y
        )

    def _grid_cb(self, msg: OccupancyGrid) -> None:
        w = int(msg.info.width)
        h = int(msg.info.height)
        if w <= 0 or h <= 0 or len(msg.data) < w * h:
            return
        grid = np.asarray(msg.data[: w * h], dtype=np.int8).reshape((h, w))
        res = float(msg.info.resolution)
        self._mapped_m2 = mapped_area_m2(grid, res)

    def _tick(self) -> None:
        cov = coverage_ratio(self._mapped_m2, self._gt_m2)
        self._ring.append(self._meters, cov)
        xs, ys = self._ring.as_series()
        try:
            jpeg = render_coverage_chart_jpeg(
                xs, ys, width=self._chart_w, height=self._chart_h
            )
        except Exception as exc:
            self.get_logger().warn(
                f"coverage chart render failed: {exc}", throttle_duration_sec=5.0
            )
            return
        out = CompressedImage()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "map"
        out.format = "jpeg"
        out.data = jpeg
        self._img_pub.publish(out)

        snap = Float32MultiArray()
        snap.data = [
            float(self._meters),
            float(self._mapped_m2),
            float(self._gt_m2),
            float(cov),
        ]
        self._snap_pub.publish(snap)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoverageMetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
