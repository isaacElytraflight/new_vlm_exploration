#!/usr/bin/env python3
"""ROS 2 bridge: /image_data, /depth_data, /movement/discrete_move."""

from __future__ import annotations

import math
import os
import socket
import threading
from concurrent.futures import ThreadPoolExecutor
from typing import Optional

import numpy as np
import rclpy
from explorer_msgs.action import DiscreteMove
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster

from explorer_bridge.driver_protocol import ExplorerDriver, PoseData
from explorer_bridge.habitat_driver import HabitatDriver
from explorer_bridge.hardware_driver import HardwareDriver
from explorer_bridge.image_utils import depth_array_to_image, rgb_array_to_image, write_jpeg_frame
from explorer_bridge.mock_driver import MockHabitatDriver

DIRECTION_TO_ACTION = {
    DiscreteMove.Goal.FORWARD: "move_forward",
    DiscreteMove.Goal.BACKWARD: "move_backward",
    DiscreteMove.Goal.TURN_LEFT: "turn_left",
    DiscreteMove.Goal.TURN_RIGHT: "turn_right",
}

TELEOP_NAME_TO_ACTION = {
    "forward": "move_forward",
    "backward": "move_backward",
    "turn_left": "turn_left",
    "left": "turn_left",
    "turn_right": "turn_right",
    "right": "turn_right",
}

VALID_DIRECTIONS = set(DIRECTION_TO_ACTION.keys())
DEFAULT_LIVE_FRAME = "/tmp/habitat_live/frame.jpg"
DEFAULT_BIRDSEYE_FRAME = "/tmp/habitat_live/birdseye.jpg"
DEFAULT_TELEOP_SOCKET = "/tmp/elytra_teleop.sock"


def create_driver(backend: str, socket_path: str) -> ExplorerDriver:
    backend = backend.strip().lower()
    if backend == "habitat":
        return HabitatDriver(socket_path)
    if backend == "hardware":
        return HardwareDriver()
    if backend == "mock":
        return MockHabitatDriver()
    raise ValueError(f"unknown driver_backend={backend!r}")


class ExplorerBridgeNode(Node):
    def __init__(self, driver: Optional[ExplorerDriver] = None) -> None:
        super().__init__("explorer_bridge_node")
        self.declare_parameter("driver_backend", "habitat")
        self.declare_parameter("habitat_socket_path", "/tmp/habitat_engine.sock")
        self.declare_parameter("publish_hz", float(os.environ.get("HABITAT_VIEW_FPS", "15")))
        self.declare_parameter("live_frame_path", DEFAULT_LIVE_FRAME)
        self.declare_parameter("birdseye_frame_path", DEFAULT_BIRDSEYE_FRAME)
        self.declare_parameter("rgb_topic", "/image_data")
        self.declare_parameter("depth_topic", "/depth_data")
        self.declare_parameter("birdseye_topic", "/birdseye_data")
        self.declare_parameter("action_name", "/movement/discrete_move")
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("depth_frame", "depth_frame")
        self.declare_parameter("teleop_socket_path", DEFAULT_TELEOP_SOCKET)

        backend = self.get_parameter("driver_backend").get_parameter_value().string_value
        socket_path = self.get_parameter("habitat_socket_path").get_parameter_value().string_value
        self._driver = driver if driver is not None else create_driver(backend, socket_path)

        rgb_topic = self.get_parameter("rgb_topic").get_parameter_value().string_value
        depth_topic = self.get_parameter("depth_topic").get_parameter_value().string_value
        birdseye_topic = self.get_parameter("birdseye_topic").get_parameter_value().string_value
        action_name = self.get_parameter("action_name").get_parameter_value().string_value

        self._live_frame = self.get_parameter("live_frame_path").get_parameter_value().string_value
        self._birdseye_frame = self.get_parameter("birdseye_frame_path").get_parameter_value().string_value
        publish_hz = max(0.1, self.get_parameter("publish_hz").get_parameter_value().double_value)

        self._rgb_pub = self.create_publisher(Image, rgb_topic, qos_profile_sensor_data)
        self._depth_pub = self.create_publisher(Image, depth_topic, qos_profile_sensor_data)
        self._birdseye_pub = self.create_publisher(Image, birdseye_topic, qos_profile_sensor_data)

        self._cb_group = ReentrantCallbackGroup()
        self._io_lock = threading.Lock()
        self._motion_active = 0
        # JPEG bind-mount writes must not block odom/TF/depth publish.
        self._jpeg_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="bridge-jpeg")
        self._teleop_socket_path = self.get_parameter("teleop_socket_path").get_parameter_value().string_value
        self._teleop_stop = threading.Event()
        self._teleop_thread = threading.Thread(target=self._teleop_loop, daemon=True, name="elytra-teleop")
        self._teleop_thread.start()
        self._action_server = ActionServer(
            self,
            DiscreteMove,
            action_name,
            execute_callback=self._execute_move,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._cb_group,
        )

        self._publish_timer = self.create_timer(
            1.0 / publish_hz,
            self._publish_sensor_data,
            callback_group=self._cb_group,
        )

        self._map_frame = self.get_parameter("map_frame").get_parameter_value().string_value
        self._odom_frame = self.get_parameter("odom_frame").get_parameter_value().string_value
        self._base_frame = self.get_parameter("base_frame").get_parameter_value().string_value
        self._depth_frame = self.get_parameter("depth_frame").get_parameter_value().string_value
        self._odom_origin: tuple[float, float, float] | None = None
        publish_odom = self.get_parameter("publish_odom").get_parameter_value().bool_value
        if publish_odom:
            odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
            self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
            self._tf_broadcaster = TransformBroadcaster(self)
            # Always refresh odom→base_link TF so Nav2 never looks microseconds
            # into the future of a stale transform. TF-only (no /odom) so the
            # mapper's exact-stamp cache is not flooded.
            self._tf_timer = self.create_timer(
                1.0 / 30.0,
                self._on_tf_keepalive_timer,
                callback_group=self._cb_group,
            )

        self._driver_ready = True
        self.get_logger().info(
            f"Explorer bridge started (backend={backend}, publish={publish_hz:.1f} Hz)"
        )

    def _goal_callback(self, goal_request: DiscreteMove.Goal) -> GoalResponse:
        if goal_request.direction not in VALID_DIRECTIONS:
            self.get_logger().warn(f"Rejecting invalid direction={goal_request.direction}")
            return GoalResponse.REJECT
        if goal_request.steps == 0:
            self.get_logger().warn("Rejecting goal with steps=0")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _publish_sensor_data(self) -> None:
        # During DiscreteMove, only post-step publishes depth/RGB. TF is kept
        # fresh by ``_on_tf_keepalive_timer`` (TF-only, no /odom).
        if self._motion_active > 0:
            return
        with self._io_lock:
            self._publish_sensor_data_locked()

    def _on_tf_keepalive_timer(self) -> None:
        with self._io_lock:
            self._publish_tf_keepalive_locked()

    def _publish_tf_keepalive_locked(self) -> None:
        """Publish odom→base_link TF only (not /odom) for Nav2 transform lookups."""
        if not self._driver_ready or not hasattr(self, "_tf_broadcaster"):
            return
        try:
            pose = self._driver.get_pose()
        except Exception as exc:
            self.get_logger().warn(f"get_pose keepalive failed: {exc}", throttle_duration_sec=2.0)
            return
        stamp = self.get_clock().now().to_msg()
        self._publish_tf_from_pose(pose, stamp)

    def _publish_sensor_data_locked(self) -> None:
        if not self._driver_ready:
            return
        try:
            if hasattr(self._driver, "get_observations_with_pose") and hasattr(self, "_odom_pub"):
                obs, pose = self._driver.get_observations_with_pose()
            else:
                obs = self._driver.get_observations()
                pose = self._driver.get_pose() if hasattr(self, "_odom_pub") else None
        except Exception as exc:
            self.get_logger().warn(f"get_observations failed: {exc}")
            self._driver_ready = False
            return

        stamp = self.get_clock().now().to_msg()

        # Publish privileged pose/TF BEFORE depth/RGB so stamp-matched consumers
        # see odom before the depth→scan pipeline runs.
        if pose is not None:
            self._publish_odom_tf_from_pose(pose, stamp)

        header = Header(stamp=stamp, frame_id="camera")
        self._rgb_pub.publish(rgb_array_to_image(obs.rgb, header=header, frame_id="camera"))
        depth_header = Header(stamp=stamp, frame_id=self._depth_frame)
        self._depth_pub.publish(
            depth_array_to_image(obs.depth, header=depth_header, frame_id=self._depth_frame)
        )

        if obs.birdseye is not None:
            birdseye_header = Header(stamp=stamp, frame_id="birdseye")
            self._birdseye_pub.publish(
                rgb_array_to_image(obs.birdseye, header=birdseye_header, frame_id="birdseye")
            )

        # Async JPEG so bind-mount latency cannot stall the next odom/depth cycle.
        if obs.birdseye is not None:
            self._enqueue_jpeg(self._birdseye_frame, obs.birdseye)
        self._enqueue_jpeg(self._live_frame, obs.rgb)

    def _enqueue_jpeg(self, path: str, rgb) -> None:
        try:
            frame = np.ascontiguousarray(rgb[..., :3] if getattr(rgb, "ndim", 0) == 3 else rgb).copy()
            self._jpeg_pool.submit(self._write_jpeg_safe, path, frame)
        except Exception as exc:
            self.get_logger().debug(f"jpeg enqueue skipped: {exc}")

    def _write_jpeg_safe(self, path: str, rgb) -> None:
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            write_jpeg_frame(path, rgb)
        except Exception as exc:
            self.get_logger().debug(f"jpeg write skipped: {exc}")

    @staticmethod
    def _yaw_to_quaternion(yaw_rad: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw_rad / 2.0)
        q.w = math.cos(yaw_rad / 2.0)
        return q

    def _habitat_pose_to_odom(self, pose: PoseData) -> tuple[float, float, float]:
        """Express privileged pose relative to episode start (odom frame)."""
        if self._odom_origin is None:
            self._odom_origin = (pose.x, pose.y, pose.yaw_rad)
        ox, oy, oyaw = self._odom_origin
        dx = pose.x - ox
        dy = pose.y - oy
        cos_y = math.cos(-oyaw)
        sin_y = math.sin(-oyaw)
        odom_x = cos_y * dx - sin_y * dy
        odom_y = sin_y * dx + cos_y * dy
        odom_yaw = pose.yaw_rad - oyaw
        while odom_yaw > math.pi:
            odom_yaw -= 2.0 * math.pi
        while odom_yaw < -math.pi:
            odom_yaw += 2.0 * math.pi
        return odom_x, odom_y, odom_yaw

    def _publish_tf_from_pose(self, pose: PoseData, stamp) -> None:
        odom_x, odom_y, odom_yaw = self._habitat_pose_to_odom(pose)
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self._odom_frame
        tf_msg.child_frame_id = self._base_frame
        tf_msg.transform.translation.x = odom_x
        tf_msg.transform.translation.y = odom_y
        tf_msg.transform.rotation = self._yaw_to_quaternion(odom_yaw)
        self._tf_broadcaster.sendTransform(tf_msg)

    def _publish_odom_tf_from_pose(self, pose: PoseData, stamp) -> None:
        odom_x, odom_y, odom_yaw = self._habitat_pose_to_odom(pose)
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = odom_x
        odom.pose.pose.position.y = odom_y
        odom.pose.pose.orientation = self._yaw_to_quaternion(odom_yaw)
        self._odom_pub.publish(odom)
        self._publish_tf_from_pose(pose, stamp)

    def _teleop_step_name(self, name: str) -> tuple[bool, str]:
        action = TELEOP_NAME_TO_ACTION.get(str(name).strip().lower())
        if not action:
            return False, "bad_direction"
        self._motion_active += 1
        try:
            with self._io_lock:
                step_result = self._driver.step(action, 1)
                if not step_result.success:
                    return False, step_result.message or "step_failed"
                self._publish_sensor_data_locked()
            return True, "ok"
        except Exception as exc:
            return False, str(exc)
        finally:
            self._motion_active = max(0, self._motion_active - 1)

    def _teleop_loop(self) -> None:
        path = self._teleop_socket_path
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        except OSError:
            pass
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            server.bind(path)
            server.listen(8)
            server.settimeout(0.5)
            self.get_logger().info(f"Teleop socket listening on {path}")
            while not self._teleop_stop.is_set() and rclpy.ok():
                try:
                    conn, _ = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    break
                with conn:
                    try:
                        raw = conn.recv(64).decode("utf-8", errors="replace").strip()
                        ok, msg = self._teleop_step_name(raw.split()[0] if raw else "")
                        conn.sendall(f"{'ok' if ok else 'err'}:{msg}\n".encode("utf-8"))
                    except Exception as exc:
                        try:
                            conn.sendall(f"err:{exc}\n".encode("utf-8"))
                        except OSError:
                            pass
        finally:
            try:
                server.close()
            except OSError:
                pass
            try:
                os.unlink(path)
            except OSError:
                pass

    async def _execute_move(self, goal_handle):
        goal = goal_handle.request
        action = DIRECTION_TO_ACTION[goal.direction]
        feedback = DiscreteMove.Feedback()
        result = DiscreteMove.Result()

        self._motion_active += 1
        completed = 0
        collided = False
        try:
            for _ in range(goal.steps):
                with self._io_lock:
                    step_result = self._driver.step(action, 1)
                    if not step_result.success:
                        result.success = False
                        result.collided = step_result.collided
                        result.message = step_result.message
                        goal_handle.abort()
                        return result
                    completed += step_result.steps_completed
                    collided = collided or step_result.collided
                    # Same lock: depth + odom share one pose (prevents spiral maps).
                    self._publish_sensor_data_locked()
                feedback.steps_completed = completed
                goal_handle.publish_feedback(feedback)

            result.success = True
            result.collided = collided
            result.message = "OK"
            goal_handle.succeed()
            return result
        finally:
            self._motion_active = max(0, self._motion_active - 1)

    def destroy_node(self) -> bool:
        self._teleop_stop.set()
        try:
            # Unblock accept() by connecting once.
            s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            s.settimeout(0.2)
            s.connect(self._teleop_socket_path)
            s.close()
        except OSError:
            pass
        try:
            self._jpeg_pool.shutdown(wait=False, cancel_futures=True)
        except Exception:
            pass
        try:
            self._driver.shutdown()
        except Exception:
            pass
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ExplorerBridgeNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
