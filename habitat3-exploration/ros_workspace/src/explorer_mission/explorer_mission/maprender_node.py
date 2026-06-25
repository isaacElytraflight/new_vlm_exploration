#!/usr/bin/env python3
"""Render occupancy grid with graph, trajectory, and frontier overlays."""

from __future__ import annotations

import math
from threading import Lock

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
import tf2_ros

from explorer_msgs.msg import FrontierArray, Graph


class MapRenderNode(Node):
    def __init__(self) -> None:
        super().__init__("map_renderer")

        self.map_msg: OccupancyGrid | None = None
        self.map_arr: np.ndarray | None = None
        self.map_res: float | None = None
        self.map_origin: tuple[float, float] | None = None
        self.graph_msg: Graph | None = None
        self.graph_vertices = None
        self.frontiers_msg: FrontierArray | None = None
        self.trajectory: list[tuple[dict, float]] = []
        self.map_lock = Lock()

        self.pub = self.create_publisher(CompressedImage, "map_renderer/map_img", 1)
        self.pub_raw = self.create_publisher(Image, "map_renderer/map_img_raw", 1)

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/grid_map", self.map_cb, 1)
        self.graph_sub = self.create_subscription(
            Graph, "/graph_node/graph", self.graph_cb, 1)
        self.frontier_sub = self.create_subscription(
            FrontierArray, "/frontiers/filtered_frontiers", self.frontier_cb, 1)

        self.target_frame = "map"
        self.source_frame = "base_link"
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self._graph_render_throttle_s = self.declare_parameter(
            "graph_render_throttle_s", 5.0).value
        self._last_graph_render_time = 0.0

        rate_hz = self.declare_parameter("rate_hz", 0.1).value
        period = 1.0 / max(rate_hz, 1e-6)
        self.create_timer(period, self.tick)

    def map_cb(self, msg: OccupancyGrid) -> None:
        with self.map_lock:
            self.map_msg = msg
            self.map_arr = np.asarray(msg.data, dtype=np.int8).reshape(
                (msg.info.height, msg.info.width))
            self.map_res = msg.info.resolution
            self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def graph_cb(self, msg: Graph) -> None:
        self.graph_msg = msg
        self.graph_vertices = msg.vertices
        now = self.get_clock().now().nanoseconds / 1e9
        if self.map_arr is not None and (now - self._last_graph_render_time) >= self._graph_render_throttle_s:
            self._last_graph_render_time = now
            self.render_and_publish()

    def frontier_cb(self, msg: FrontierArray) -> None:
        self.frontiers_msg = msg

    def get_pose(self) -> tuple[float, float, float]:
        trans: TransformStamped = self.tf_buffer.lookup_transform(
            self.target_frame, self.source_frame, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.5))
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        q = trans.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return x, y, math.degrees(yaw)

    def overlay_graph_on_occupancy_map(
        self,
        robot_color=(0, 0, 255),
        arrow_color=(0, 165, 255),
        vertex_color=(0, 255, 0),
        trajectory_color=(200, 0, 0),
        frontier_outline_color=(0, 255, 255),
        frontier_label_color=(255, 0, 255),
        vertex_radius=3,
        robot_radius=3,
        arrow_len_m=0.5,
        crop_margin_px=10,
    ):
        with self.map_lock:
            if self.map_msg is None:
                raise RuntimeError("No /grid_map message received yet.")
            grid = self.map_arr.copy()
            res = self.map_res
            ox, oy = self.map_origin
            h, w = self.map_arr.shape

        robot_x, robot_y, robot_yaw_deg = self.get_pose()

        gray = np.where(grid == -1, 127, np.where(grid > 50, 0, 255)).astype(np.uint8)
        color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        color = cv2.flip(color, 1)

        def world_to_flipped_pixel(wx, wy):
            col = int(round((wx - ox) / res))
            row = int(round((wy - oy) / res))
            col = w - 1 - col
            return col, row

        font = cv2.FONT_HERSHEY_SIMPLEX
        if self.frontiers_msg and getattr(self.frontiers_msg, "frontiers", None):
            for frontier in self.frontiers_msg.frontiers:
                pts = np.array([
                    [world_to_flipped_pixel(p.x, p.y)]
                    for p in frontier.points
                ], dtype=np.int32)
                cv2.drawContours(color, [pts], -1, frontier_outline_color, 1)
                px, py = world_to_flipped_pixel(frontier.midpoint.x, frontier.midpoint.y)
                label_id = frontier.id
                cv2.putText(color, str(label_id), (px - 2, py + 2), font, 0.4, (0, 0, 0), 2, cv2.LINE_AA)
                cv2.putText(
                    color, str(label_id), (px - 2, py + 2), font, 0.4,
                    frontier_label_color, 1, cv2.LINE_AA)

        if len(self.trajectory) > 1:
            pixel_points = [world_to_flipped_pixel(pos["x"], pos["y"]) for pos, _yaw in self.trajectory]
            pts = np.array(pixel_points, np.int32).reshape((-1, 1, 2))
            cv2.polylines(color, [pts], isClosed=False, color=trajectory_color, thickness=2)

        if self.graph_msg is not None and self.graph_vertices is not None:
            for v_node in self.graph_vertices:
                vx, vy = world_to_flipped_pixel(v_node.x, v_node.y)
                cv2.circle(color, (vx, vy), vertex_radius, vertex_color, -1)

        rpx, rpy = world_to_flipped_pixel(robot_x, robot_y)
        cv2.circle(color, (rpx, rpy), robot_radius, robot_color, -1)
        yaw_rad = math.radians(robot_yaw_deg)
        arrow_px = arrow_len_m / res
        dx_px = -arrow_px * math.cos(yaw_rad)
        dy_px = arrow_px * math.sin(yaw_rad)
        cv2.arrowedLine(
            color, (rpx, rpy), (int(rpx + dx_px), int(rpy + dy_px)),
            arrow_color, 2, tipLength=0.3)

        mask = np.any(color != 127, axis=2)
        ys, xs = np.where(mask)
        if xs.size and ys.size:
            m = crop_margin_px
            y0 = max(int(ys.min()) - m, 0)
            y1 = min(int(ys.max()) + m, color.shape[0] - 1)
            x0 = max(int(xs.min()) - m, 0)
            x1 = min(int(xs.max()) + m, color.shape[1] - 1)
            color = color[y0:y1 + 1, x0:x1 + 1]

        footer_h = 20
        color = cv2.copyMakeBorder(
            color, 0, footer_h, 0, 0, cv2.BORDER_CONSTANT, value=(127, 127, 127))
        return color

    def render_and_publish(self) -> None:
        if self.map_arr is None:
            return
        try:
            color = self.overlay_graph_on_occupancy_map()
        except Exception as exc:
            self.get_logger().warn(f"Render failed: {exc}")
            return

        ok, buf = cv2.imencode(".png", color)
        if not ok:
            self.get_logger().warn("PNG encode failed")
            return

        msg = CompressedImage()
        with self.map_lock:
            msg.header = self.map_msg.header
        msg.format = "png"
        msg.data = buf.tobytes()

        msg_raw = Image()
        msg_raw.header.stamp = self.get_clock().now().to_msg()
        msg_raw.header.frame_id = "map"
        color = np.ascontiguousarray(color)
        msg_raw.height = color.shape[0]
        msg_raw.width = color.shape[1]
        msg_raw.encoding = "bgr8"
        msg_raw.is_bigendian = False
        msg_raw.step = color.shape[1] * color.shape[2] * color.dtype.itemsize
        msg_raw.data = color.tobytes()

        self.pub_raw.publish(msg_raw)
        self.pub.publish(msg)

    def tick(self) -> None:
        self.render_and_publish()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapRenderNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
