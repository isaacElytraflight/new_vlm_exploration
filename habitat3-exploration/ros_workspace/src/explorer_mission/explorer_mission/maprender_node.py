#!/usr/bin/env python3
"""Render occupancy grid: plain grid view + annotated nav-plan view."""

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


def openness_label_text(score: int) -> str | None:
    """Return 0–5 label for a rated frontier; None if not yet rated."""
    if score == NOT_RATED:
        return None
    return str(int(score))


def occupancy_to_bgr(grid: np.ndarray) -> np.ndarray:
    """Occupancy grid → BGR image (flipped horizontally). No overlays."""
    if grid.size == 0:
        return np.zeros((0, 0, 3), dtype=np.uint8)
    gray = np.where(grid == -1, 127, np.where(grid > 50, 0, 255)).astype(np.uint8)
    color = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    return cv2.flip(color, 1)


def encode_png(bgr: np.ndarray) -> bytes:
    ok, buf = cv2.imencode(".png", bgr)
    if not ok:
        raise RuntimeError("PNG encode failed")
    return buf.tobytes()


def crop_to_content(color: np.ndarray, *, crop_margin_px: int = 10) -> np.ndarray:
    mask = np.any(color != 127, axis=2)
    ys, xs = np.where(mask)
    if not xs.size or not ys.size:
        return color
    m = crop_margin_px
    y0 = max(int(ys.min()) - m, 0)
    y1 = min(int(ys.max()) + m, color.shape[0] - 1)
    x0 = max(int(xs.min()) - m, 0)
    x1 = min(int(xs.max()) + m, color.shape[1] - 1)
    return color[y0 : y1 + 1, x0 : x1 + 1]


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

        # Plain occupancy (dashboard Grid Map).
        self.grid_pub = self.create_publisher(CompressedImage, "map_renderer/grid_img", 1)
        # Annotated nav plan (dashboard Nav Plan).
        self.nav_pub = self.create_publisher(CompressedImage, "map_renderer/nav_plan_img", 1)
        # Back-compat alias used by older view configs.
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

    def render_grid_only(self, *, crop_margin_px: int = 10) -> np.ndarray:
        """Occupancy only — no frontiers, plans, trajectory, or robot."""
        with self.map_lock:
            if self.map_arr is None:
                raise RuntimeError("No /grid_map message received yet.")
            grid = self.map_arr.copy()
        color = occupancy_to_bgr(grid)
        color = crop_to_content(color, crop_margin_px=crop_margin_px)
        footer_h = 20
        return cv2.copyMakeBorder(
            color, 0, footer_h, 0, 0, cv2.BORDER_CONSTANT, value=(127, 127, 127))

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
        color = occupancy_to_bgr(grid)

        def world_to_flipped_pixel(wx, wy):
            col = int(round((wx - ox) / res))
            row = int(round((wy - oy) / res))
            col = w - 1 - col
            return col, row

        font = cv2.FONT_HERSHEY_SIMPLEX
        current_id = self.tree_msg.current_node_id if self.tree_msg else 0
        if self.tree_msg and self.tree_msg.nodes:
            by_id = {int(n.id): n for n in self.tree_msg.nodes}
            tree_edge_color = (0, 200, 0)  # thin green parent→child
            for node in self.tree_msg.nodes:
                if int(node.parent_id) < 0:
                    continue
                parent = by_id.get(int(node.parent_id))
                if parent is None:
                    continue
                p0 = world_to_flipped_pixel(parent.position.x, parent.position.y)
                p1 = world_to_flipped_pixel(node.position.x, node.position.y)
                cv2.line(color, p0, p1, tree_edge_color, 1, cv2.LINE_AA)
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
                label = openness_label_text(int(node.openness_score))
                if label is not None:
                    tx, ty = px - 4, py - radius - 4
                    cv2.putText(color, label, (tx, ty), font, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
                    cv2.putText(color, label, (tx, ty), font, 0.55, label_color, 1, cv2.LINE_AA)

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

        color = crop_to_content(color, crop_margin_px=crop_margin_px)
        footer_h = 20
        color = cv2.copyMakeBorder(
            color, 0, footer_h, 0, 0, cv2.BORDER_CONSTANT, value=(127, 127, 127))
        return color

    def _publish_png(self, publisher, bgr: np.ndarray) -> None:
        msg = CompressedImage()
        with self.map_lock:
            if self.map_msg is not None:
                msg.header = self.map_msg.header
        msg.format = "png"
        msg.data = encode_png(bgr)
        publisher.publish(msg)

    def render_and_publish(self) -> None:
        if self.map_arr is None:
            return
        try:
            grid_img = self.render_grid_only()
            self._publish_png(self.grid_pub, grid_img)

            x, y, yaw = self.get_pose()
            self.trajectory.append(({"x": x, "y": y}, yaw))
            nav_img = self.overlay_map()
            self._publish_png(self.nav_pub, nav_img)
            self._publish_png(self.pub, nav_img)
        except Exception as exc:
            self.get_logger().warn(f"Render failed: {exc}")
            return

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
