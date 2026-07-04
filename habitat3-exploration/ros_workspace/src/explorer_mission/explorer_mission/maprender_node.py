#!/usr/bin/env python3
"""Render occupancy grid with frontier tree, trajectory, and plan overlays."""

from __future__ import annotations

import math
from threading import Lock

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import CompressedImage
import tf2_ros

from explorer_msgs.msg import FrontierTree

NOT_RATED = 255


class MapRenderNode(Node):
    def __init__(self) -> None:
        super().__init__("map_renderer")

        self.map_msg: OccupancyGrid | None = None
        self.map_arr: np.ndarray | None = None
        self.map_res: float | None = None
        self.map_origin: tuple[float, float] | None = None
        self.tree_msg: FrontierTree | None = None
        self.global_plan: Path | None = None
        self.local_plan: Path | None = None
        self.trajectory: list[tuple[dict, float]] = []
        self.map_lock = Lock()

        self.pub = self.create_publisher(CompressedImage, "map_renderer/map_img", 1)

        self.map_sub = self.create_subscription(
            OccupancyGrid, "/grid_map", self.map_cb, 1)
        tree_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.tree_sub = self.create_subscription(
            FrontierTree, "/exploration/frontier_tree", self.tree_cb, tree_qos)
        self.global_plan_sub = self.create_subscription(
            Path, "/plan", self.global_plan_cb, 1)
        self.local_plan_sub = self.create_subscription(
            Path, "/local_plan", self.local_plan_cb, 1)

        self.target_frame = "map"
        self.source_frame = "base_link"
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        rate_hz = self.declare_parameter("rate_hz", 1.0).value
        period = 1.0 / max(rate_hz, 1e-6)
        self.create_timer(period, self.tick)

    def map_cb(self, msg: OccupancyGrid) -> None:
        first_map = False
        with self.map_lock:
            first_map = self.map_arr is None
            self.map_msg = msg
            self.map_arr = np.asarray(msg.data, dtype=np.int8).reshape(
                (msg.info.height, msg.info.width))
            self.map_res = msg.info.resolution
            self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        if first_map:
            self.render_and_publish()

    def tree_cb(self, msg: FrontierTree) -> None:
        self.tree_msg = msg
        if self.map_arr is not None:
            self.render_and_publish()

    def global_plan_cb(self, msg: Path) -> None:
        self.global_plan = msg
        if self.map_arr is not None:
            self.render_and_publish()

    def local_plan_cb(self, msg: Path) -> None:
        self.local_plan = msg
        if self.map_arr is not None:
            self.render_and_publish()

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

    def overlay_map(
        self,
        robot_color=(0, 0, 255),
        arrow_color=(0, 165, 255),
        node_color=(0, 255, 0),
        current_node_color=(255, 128, 0),
        explored_color=(128, 128, 128),
        trajectory_color=(200, 0, 0),
        label_color=(255, 0, 255),
        global_plan_color=(255, 165, 0),
        local_plan_color=(255, 255, 0),
        node_radius=4,
        current_node_radius=6,
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
        current_id = self.tree_msg.current_node_id if self.tree_msg else 0
        if self.tree_msg and self.tree_msg.nodes:
            for node in self.tree_msg.nodes:
                px, py = world_to_flipped_pixel(node.position.x, node.position.y)
                if node.fully_explored:
                    dot_color = explored_color
                elif node.id == current_id:
                    dot_color = current_node_color
                else:
                    dot_color = node_color
                radius = current_node_radius if node.id == current_id else node_radius
                cv2.circle(color, (px, py), radius, dot_color, -1)
                if node.openness_score != NOT_RATED:
                    label = str(node.openness_score)
                    cv2.putText(color, label, (px - 2, py + 2), font, 0.4, (0, 0, 0), 2, cv2.LINE_AA)
                    cv2.putText(color, label, (px - 2, py + 2), font, 0.4, label_color, 1, cv2.LINE_AA)

        if len(self.trajectory) > 1:
            pixel_points = [world_to_flipped_pixel(pos["x"], pos["y"]) for pos, _yaw in self.trajectory]
            pts = np.array(pixel_points, np.int32).reshape((-1, 1, 2))
            cv2.polylines(color, [pts], isClosed=False, color=trajectory_color, thickness=2)

        def draw_path(path_msg: Path | None, plan_color) -> None:
            if path_msg is None or len(path_msg.poses) < 2:
                return
            pixel_points = [
                world_to_flipped_pixel(p.pose.position.x, p.pose.position.y)
                for p in path_msg.poses
            ]
            pts = np.array(pixel_points, np.int32).reshape((-1, 1, 2))
            cv2.polylines(color, [pts], isClosed=False, color=plan_color, thickness=2)

        draw_path(self.global_plan, global_plan_color)
        draw_path(self.local_plan, local_plan_color)

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
            x, y, yaw = self.get_pose()
            self.trajectory.append(({"x": x, "y": y}, yaw))
            color = self.overlay_map()
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
