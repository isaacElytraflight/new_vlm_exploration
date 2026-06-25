#!/usr/bin/env python3
"""Publish nav_msgs/OccupancyGrid from habitat_engine get_map IPC."""

from __future__ import annotations

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from std_msgs.msg import Header

from explorer_bridge.habitat_driver import HabitatDriver


class HabitatMapNode(Node):
    def __init__(self) -> None:
        super().__init__("habitat_map_node")
        self.declare_parameter("habitat_socket_path", "/tmp/habitat_engine.sock")
        self.declare_parameter("grid_topic", "/grid_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("publish_hz", 1.0)

        socket_path = self.get_parameter("habitat_socket_path").get_parameter_value().string_value
        grid_topic = self.get_parameter("grid_topic").get_parameter_value().string_value
        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        publish_hz = max(0.1, self.get_parameter("publish_hz").get_parameter_value().double_value)

        self._driver = HabitatDriver(socket_path)
        self._pub = self.create_publisher(OccupancyGrid, grid_topic, 1)
        self._timer = self.create_timer(1.0 / publish_hz, self._publish_map)
        self.get_logger().info(f"Habitat map node publishing {grid_topic} at {publish_hz:.1f} Hz")

    def _publish_map(self) -> None:
        try:
            map_data = self._driver.get_map()
        except Exception as exc:
            self.get_logger().warn(f"get_map failed: {exc}", throttle_duration_sec=5.0)
            return

        grid = map_data.grid
        msg = OccupancyGrid()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self._map_frame,
        )
        msg.info.resolution = map_data.resolution
        msg.info.width = int(grid.shape[1])
        msg.info.height = int(grid.shape[0])
        msg.info.origin.position.x = map_data.origin_x
        msg.info.origin.position.y = map_data.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HabitatMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
