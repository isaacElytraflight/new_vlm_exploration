#!/usr/bin/env python3
"""Build /grid_map from /scan + stamped /odom (privileged pose; no slam_toolbox).

Uses /odom stamp matching instead of TF lookup-with-timeout in the scan callback.
A blocking TF lookup on a single-threaded executor prevents TransformListener from
receiving /tf, so scan stamps stay "in the future" and every integrate is skipped.
"""

from __future__ import annotations

import math
from collections import OrderedDict, deque

import rclpy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

from explorer_bridge.scan_to_occupancy import (
    OccupancyMap,
    inflate_occupied,
    inflation_radius_cells,
    integrate_scan,
    scan_content_signature,
    should_integrate_scan,
    should_integrate_with_tf,
    stamp_msg_to_ns,
)


def _yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class KnownPoseMapperNode(Node):
    def __init__(self) -> None:
        super().__init__("known_pose_mapper")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("grid_topic", "/grid_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("initial_size_m", 20.0)
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("odom_cache_size", 2048)
        self.declare_parameter("max_stamp_skew_sec", 0.0)
        self.declare_parameter("pending_scan_limit", 128)
        self.declare_parameter("obstacle_inflation_m", 0.10)

        scan_topic = str(self.get_parameter("scan_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        grid_topic = str(self.get_parameter("grid_topic").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        resolution = float(self.get_parameter("resolution").value)
        initial_size = float(self.get_parameter("initial_size_m").value)
        publish_hz = max(0.2, float(self.get_parameter("publish_hz").value))
        self._cache_size = max(8, int(self.get_parameter("odom_cache_size").value))
        skew_sec = max(0.0, float(self.get_parameter("max_stamp_skew_sec").value))
        self._max_skew_ns = int(skew_sec * 1_000_000_000)
        pending_limit = max(1, int(self.get_parameter("pending_scan_limit").value))
        inflation_m = max(0.0, float(self.get_parameter("obstacle_inflation_m").value))
        self._inflate_cells = inflation_radius_cells(resolution, inflation_m)

        self._grid = OccupancyMap(resolution=resolution, initial_size_m=initial_size)
        self._last_sig: tuple | None = None
        self._last_yaw: float | None = None
        # stamp_ns → (x, y, yaw); map≡odom identity.
        self._odom_by_stamp: OrderedDict[int, tuple[float, float, float]] = OrderedDict()
        self._pending_scans: deque[LaserScan] = deque(maxlen=pending_limit)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(OccupancyGrid, grid_topic, map_qos)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 50)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, qos_profile_sensor_data)
        self.create_timer(1.0 / publish_hz, self._publish)
        self.get_logger().info(
            f"Known-pose mapper: {scan_topic} + {odom_topic} (exact stamp) → {grid_topic} "
            f"(obstacle_inflation={inflation_m:.2f}m / {self._inflate_cells} cells)"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        stamp_ns = stamp_msg_to_ns(msg.header.stamp)
        pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            _yaw_from_odom(msg),
        )
        self._odom_by_stamp[stamp_ns] = pose
        while len(self._odom_by_stamp) > self._cache_size:
            self._odom_by_stamp.popitem(last=False)
        self._drain_pending_scans()

    def _scan_cb(self, msg: LaserScan) -> None:
        if not self._try_integrate_scan(msg):
            self._pending_scans.append(msg)
            self.get_logger().warn(
                "No /odom with exact scan stamp "
                f"(cache={len(self._odom_by_stamp)}; deferred)",
                throttle_duration_sec=2.0,
            )

    def _drain_pending_scans(self) -> None:
        if not self._pending_scans:
            return
        remaining: list[LaserScan] = []
        while self._pending_scans:
            scan = self._pending_scans.popleft()
            stamp_ns = stamp_msg_to_ns(scan.header.stamp)
            if stamp_ns in self._odom_by_stamp:
                self._try_integrate_scan(scan)
                continue
            # Keep waiting only while the matching odom might still arrive.
            # Drop only when the stamp is older than the oldest cached odom.
            if self._odom_by_stamp:
                oldest = next(iter(self._odom_by_stamp))
                if stamp_ns < oldest:
                    continue
            remaining.append(scan)
        self._pending_scans.clear()
        self._pending_scans.extend(remaining)

    def _try_integrate_scan(self, msg: LaserScan) -> bool:
        stamp_ns = stamp_msg_to_ns(msg.header.stamp)
        pose = self._odom_by_stamp.get(stamp_ns)
        if self._max_skew_ns > 0 and pose is None:
            # Optional skew path kept for tests/config; default launch uses 0.
            from explorer_bridge.scan_to_occupancy import find_pose_for_stamp

            cache = [(ts, *xyyaw) for ts, xyyaw in self._odom_by_stamp.items()]
            pose = find_pose_for_stamp(cache, stamp_ns, max_skew_ns=self._max_skew_ns)

        stamp_ok = pose is not None
        if not should_integrate_with_tf(stamp_lookup_ok=stamp_ok) or pose is None:
            return False

        robot_x, robot_y, yaw = pose
        ranges = list(msg.ranges)
        sig = scan_content_signature(ranges)
        if not should_integrate_scan(
            signature=sig,
            yaw=yaw,
            last_signature=self._last_sig,
            last_yaw=self._last_yaw,
        ):
            return True
        angles = [
            yaw + (msg.angle_min + i * msg.angle_increment)
            for i in range(len(ranges))
        ]
        integrate_scan(
            self._grid,
            robot_x=robot_x,
            robot_y=robot_y,
            ranges=ranges,
            angles=angles,
            range_min=float(msg.range_min),
            range_max=float(msg.range_max),
        )
        self._last_sig = sig
        self._last_yaw = yaw
        return True

    def _publish(self) -> None:
        msg = OccupancyGrid()
        msg.header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self._map_frame,
        )
        info = MapMetaData()
        info.resolution = self._grid.resolution
        info.width = self._grid.width
        info.height = self._grid.height
        info.origin.position.x = self._grid.origin_x
        info.origin.position.y = self._grid.origin_y
        info.origin.orientation.w = 1.0
        msg.info = info
        published = inflate_occupied(self._grid.data, radius_cells=self._inflate_cells)
        msg.data = published.flatten().tolist()
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KnownPoseMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
