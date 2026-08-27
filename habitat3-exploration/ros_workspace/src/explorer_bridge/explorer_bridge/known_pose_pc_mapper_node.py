#!/usr/bin/env python3
"""Build /grid_map from /depth_data + stamped /odom (privileged pose; no SLAM).

Point-cloud-style mapping: ray-carve free space from the full depth FOV, but
mark occupied only for hits between floor and robot height (~1 m).
"""

from __future__ import annotations

import math
from collections import OrderedDict, deque

import rclpy
from nav_msgs.msg import MapMetaData, OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header

from explorer_bridge.image_utils import image_to_depth_array
from explorer_bridge.pc_to_occupancy import (
    DEFAULT_WALL_HEIGHT_MAX_M,
    DEFAULT_WALL_HEIGHT_MIN_M,
    depth_content_signature,
    integrate_depth_frame,
)
from explorer_bridge.scan_to_occupancy import (
    OccupancyMap,
    inflate_occupied,
    inflation_radius_cells,
    should_integrate_scan,
    should_integrate_with_tf,
    stamp_msg_to_ns,
)


def _yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class KnownPosePcMapperNode(Node):
    def __init__(self) -> None:
        super().__init__("known_pose_pc_mapper")
        self.declare_parameter("depth_topic", "/depth_data")
        self.declare_parameter("camera_info_topic", "/depth/camera_info")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("grid_topic", "/grid_map")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("initial_size_m", 20.0)
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("odom_cache_size", 2048)
        self.declare_parameter("max_stamp_skew_sec", 0.0)
        self.declare_parameter("pending_depth_limit", 128)
        self.declare_parameter("obstacle_inflation_m", 0.10)
        self.declare_parameter("range_min", 0.1)
        self.declare_parameter("range_max", 10.0)
        self.declare_parameter("sensor_far", 50.0)
        self.declare_parameter("sat_eps", 0.5)
        self.declare_parameter("camera_z", 0.1)
        self.declare_parameter("wall_height_min_m", DEFAULT_WALL_HEIGHT_MIN_M)
        self.declare_parameter("wall_height_max_m", DEFAULT_WALL_HEIGHT_MAX_M)
        self.declare_parameter("subsample", 4)

        depth_topic = str(self.get_parameter("depth_topic").value)
        info_topic = str(self.get_parameter("camera_info_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        grid_topic = str(self.get_parameter("grid_topic").value)
        self._map_frame = str(self.get_parameter("map_frame").value)
        resolution = float(self.get_parameter("resolution").value)
        initial_size = float(self.get_parameter("initial_size_m").value)
        publish_hz = max(0.2, float(self.get_parameter("publish_hz").value))
        self._cache_size = max(8, int(self.get_parameter("odom_cache_size").value))
        skew_sec = max(0.0, float(self.get_parameter("max_stamp_skew_sec").value))
        self._max_skew_ns = int(skew_sec * 1_000_000_000)
        pending_limit = max(1, int(self.get_parameter("pending_depth_limit").value))
        inflation_m = max(0.0, float(self.get_parameter("obstacle_inflation_m").value))
        self._inflate_cells = inflation_radius_cells(resolution, inflation_m)

        self._range_min = float(self.get_parameter("range_min").value)
        self._range_max = float(self.get_parameter("range_max").value)
        self._sensor_far = float(self.get_parameter("sensor_far").value)
        self._sat_eps = float(self.get_parameter("sat_eps").value)
        self._camera_z = float(self.get_parameter("camera_z").value)
        self._wall_height_min = float(self.get_parameter("wall_height_min_m").value)
        self._wall_height_max = float(self.get_parameter("wall_height_max_m").value)
        self._subsample = max(1, int(self.get_parameter("subsample").value))

        self._fx = 320.0
        self._fy = 320.0
        self._cx = 320.0
        self._cy = 240.0
        self._have_info = False

        self._grid = OccupancyMap(resolution=resolution, initial_size_m=initial_size)
        self._last_sig: tuple | None = None
        self._last_yaw: float | None = None
        self._odom_by_stamp: OrderedDict[int, tuple[float, float, float]] = OrderedDict()
        self._pending_depth: deque[Image] = deque(maxlen=pending_limit)

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._pub = self.create_publisher(OccupancyGrid, grid_topic, map_qos)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 50)
        self.create_subscription(CameraInfo, info_topic, self._info_cb, 10)
        self.create_subscription(Image, depth_topic, self._depth_cb, qos_profile_sensor_data)
        self.create_timer(1.0 / publish_hz, self._publish)
        self.get_logger().info(
            f"Known-pose PC mapper: {depth_topic} + {odom_topic} → {grid_topic} "
            f"(wall band [{self._wall_height_min:.2f}, {self._wall_height_max:.2f}] m, "
            f"subsample={self._subsample})"
        )

    def _info_cb(self, msg: CameraInfo) -> None:
        self._fx = float(msg.k[0])
        self._fy = float(msg.k[4])
        self._cx = float(msg.k[2])
        self._cy = float(msg.k[5])
        self._have_info = True

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
        self._drain_pending_depth()

    def _depth_cb(self, msg: Image) -> None:
        if not self._have_info or msg.encoding != "32FC1":
            return
        if not self._try_integrate_depth(msg):
            self._pending_depth.append(msg)
            self.get_logger().warn(
                "No /odom with exact depth stamp "
                f"(cache={len(self._odom_by_stamp)}; deferred)",
                throttle_duration_sec=2.0,
            )

    def _drain_pending_depth(self) -> None:
        if not self._pending_depth:
            return
        remaining: list[Image] = []
        while self._pending_depth:
            img = self._pending_depth.popleft()
            stamp_ns = stamp_msg_to_ns(img.header.stamp)
            if stamp_ns in self._odom_by_stamp:
                self._try_integrate_depth(img)
                continue
            if self._odom_by_stamp:
                oldest = next(iter(self._odom_by_stamp))
                if stamp_ns < oldest:
                    continue
            remaining.append(img)
        self._pending_depth.clear()
        self._pending_depth.extend(remaining)

    def _try_integrate_depth(self, msg: Image) -> bool:
        stamp_ns = stamp_msg_to_ns(msg.header.stamp)
        pose = self._odom_by_stamp.get(stamp_ns)
        if self._max_skew_ns > 0 and pose is None:
            from explorer_bridge.scan_to_occupancy import find_pose_for_stamp

            cache = [(ts, *xyyaw) for ts, xyyaw in self._odom_by_stamp.items()]
            pose = find_pose_for_stamp(cache, stamp_ns, max_skew_ns=self._max_skew_ns)

        stamp_ok = pose is not None
        if not should_integrate_with_tf(stamp_lookup_ok=stamp_ok) or pose is None:
            return False

        depth = image_to_depth_array(msg)
        sig = depth_content_signature(depth, stride=self._subsample)
        robot_x, robot_y, yaw = pose
        if not should_integrate_scan(
            signature=sig,
            yaw=yaw,
            last_signature=self._last_sig,
            last_yaw=self._last_yaw,
        ):
            return True

        integrate_depth_frame(
            self._grid,
            depth,
            robot_x=robot_x,
            robot_y=robot_y,
            yaw=yaw,
            fx=self._fx,
            fy=self._fy,
            cx=self._cx,
            cy=self._cy,
            range_min=self._range_min,
            range_max=self._range_max,
            camera_z=self._camera_z,
            sensor_far=self._sensor_far,
            sat_eps=self._sat_eps,
            wall_height_min=self._wall_height_min,
            wall_height_max=self._wall_height_max,
            subsample=self._subsample,
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
    node = KnownPosePcMapperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
