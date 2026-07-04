#!/usr/bin/env python3
"""Publish /scan from /depth_data with SensorDataQoS (slam_toolbox compatible)."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from explorer_bridge.scan_from_depth import depth_to_laserscan_bins
from explorer_bridge.image_utils import image_to_depth_array


class DepthToLaserScanNode(Node):
    def __init__(self) -> None:
        super().__init__("depth_to_laserscan")
        self.declare_parameter("range_min", 0.1)
        self.declare_parameter("range_max", 5.0)
        self.declare_parameter("scan_height", 24)
        self.declare_parameter("output_frame", "base_link")
        self.declare_parameter("depth_topic", "/depth_data")
        self.declare_parameter("camera_info_topic", "/depth/camera_info")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("full_360", True)
        self.declare_parameter("band_anchor", "bottom")
        self.declare_parameter("num_bins", 360)

        self._range_min = float(self.get_parameter("range_min").value)
        self._range_max = float(self.get_parameter("range_max").value)
        self._scan_height = int(self.get_parameter("scan_height").value)
        self._output_frame = str(self.get_parameter("output_frame").value)
        self._full_360 = bool(self.get_parameter("full_360").value)
        self._band_anchor = str(self.get_parameter("band_anchor").value)
        self._num_bins = int(self.get_parameter("num_bins").value)
        scan_topic = str(self.get_parameter("scan_topic").value)
        depth_topic = str(self.get_parameter("depth_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)

        self._fx = 320.0
        self._fy = 320.0
        self._cx = 320.0
        self._cy = 240.0
        self._have_info = False

        self._scan_pub = self.create_publisher(LaserScan, scan_topic, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self._info_cb, 10)
        self.create_subscription(Image, depth_topic, self._depth_cb, qos_profile_sensor_data)
        self.get_logger().info(
            f"Publishing {scan_topic} from {depth_topic} "
            f"(frame={self._output_frame}, range=[{self._range_min}, {self._range_max}], "
            f"360={self._full_360}, band={self._band_anchor})"
        )

    def _info_cb(self, msg: CameraInfo) -> None:
        self._fx = float(msg.k[0])
        self._fy = float(msg.k[4])
        self._cx = float(msg.k[2])
        self._cy = float(msg.k[5])
        self._have_info = True

    def _depth_cb(self, msg: Image) -> None:
        if not self._have_info or msg.encoding != "32FC1":
            return
        depth = image_to_depth_array(msg)
        ranges, angle_min, angle_max, angle_increment = depth_to_laserscan_bins(
            depth,
            fx=self._fx,
            fy=self._fy,
            cx=self._cx,
            cy=self._cy,
            range_min=self._range_min,
            clear_range=self._range_max,
            scan_height=self._scan_height,
            band_anchor=self._band_anchor,  # type: ignore[arg-type]
            full_360=self._full_360,
            num_bins=self._num_bins,
        )

        scan = LaserScan()
        scan.header = msg.header
        scan.header.frame_id = self._output_frame
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_increment
        scan.time_increment = 0.0
        scan.scan_time = 0.033
        scan.range_min = self._range_min
        scan.range_max = self._range_max
        scan.ranges = ranges
        self._scan_pub.publish(scan)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthToLaserScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
