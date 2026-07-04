#!/usr/bin/env python3
"""End-to-end readiness check: /scan finite, /grid_map + map TF within timeout."""

from __future__ import annotations

import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class StackProbe(Node):
    def __init__(self) -> None:
        super().__init__("stack_probe")
        self.scan_finite = 0
        self.scan_total = 0
        self.have_grid = False
        self.create_subscription(
            LaserScan, "/scan", self._scan_cb, qos_profile_sensor_data
        )
        self.create_subscription(OccupancyGrid, "/grid_map", self._grid_cb, 1)

    def _scan_cb(self, msg: LaserScan) -> None:
        for r in msg.ranges:
            self.scan_total += 1
            if math.isfinite(r) and r > 0.1:
                self.scan_finite += 1

    def _grid_cb(self, _msg: OccupancyGrid) -> None:
        self.have_grid = True


def main() -> int:
    rclpy.init()
    node = StackProbe()
    deadline = time.time() + 60.0
    map_tf_ok = False
    errors: list[str] = []

    try:
        while time.time() < deadline and rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except Exception:
                break
            if node.scan_total > 0 and node.scan_finite == 0:
                errors.append("scan publishes but all ranges invalid")
                break
            if node.have_grid:
                try:
                    if not hasattr(node, "_tf_buffer"):
                        from tf2_ros import Buffer, TransformListener

                        node._tf_buffer = Buffer()  # type: ignore[attr-defined]
                        node._tf_listener = TransformListener(node._tf_buffer, node)  # type: ignore[attr-defined]
                    node._tf_buffer.lookup_transform(  # type: ignore[attr-defined]
                        "map", "base_link", rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.5)
                    )
                    map_tf_ok = True
                    break
                except Exception:
                    pass
    except KeyboardInterrupt:
        pass

    if not rclpy.ok():
        errors.append("ROS context shut down during probe")

    if node.scan_total == 0:
        errors.append("/scan not received")
    elif node.scan_finite == 0:
        errors.append("/scan has zero finite ranges")
    if not node.have_grid:
        errors.append("/grid_map not received")
    if not map_tf_ok:
        errors.append("map→base_link TF not available")

    node.destroy_node()
    rclpy.shutdown()

    if errors:
        print("STACK_NOT_READY:", "; ".join(errors), file=sys.stderr)
        print(
            f"scan_finite={node.scan_finite}/{node.scan_total} grid={node.have_grid} map_tf={map_tf_ok}",
            file=sys.stderr,
        )
        return 1

    print(
        f"STACK_OK scan_finite={node.scan_finite}/{node.scan_total} "
        f"grid={node.have_grid} map_tf={map_tf_ok}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
