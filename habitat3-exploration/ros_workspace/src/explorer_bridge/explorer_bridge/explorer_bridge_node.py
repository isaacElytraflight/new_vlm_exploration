#!/usr/bin/env python3
"""ROS 2 bridge: /image_data, /depth_data, /movement/discrete_move."""

from __future__ import annotations

import os
from typing import Optional

import math

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

from explorer_bridge.driver_protocol import ExplorerDriver
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

VALID_DIRECTIONS = set(DIRECTION_TO_ACTION.keys())
DEFAULT_LIVE_FRAME = "/tmp/habitat_live/frame.jpg"
DEFAULT_BIRDSEYE_FRAME = "/tmp/habitat_live/birdseye.jpg"


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
        if not self._driver_ready:
            return
        try:
            obs = self._driver.get_observations()
        except Exception as exc:
            self.get_logger().warn(f"get_observations failed: {exc}")
            self._driver_ready = False
            return

        stamp = self.get_clock().now().to_msg()
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
            try:
                os.makedirs(os.path.dirname(self._birdseye_frame), exist_ok=True)
                write_jpeg_frame(self._birdseye_frame, obs.birdseye)
            except Exception as exc:
                self.get_logger().debug(f"birdseye frame write skipped: {exc}")

        try:
            os.makedirs(os.path.dirname(self._live_frame), exist_ok=True)
            write_jpeg_frame(self._live_frame, obs.rgb)
        except Exception as exc:
            self.get_logger().debug(f"live frame write skipped: {exc}")

        if hasattr(self, "_odom_pub"):
            self._publish_odom_tf(stamp)

    @staticmethod
    def _yaw_to_quaternion(yaw_rad: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw_rad / 2.0)
        q.w = math.cos(yaw_rad / 2.0)
        return q

    def _habitat_pose_to_odom(self, pose) -> tuple[float, float, float]:
        """Express Habitat ground-truth pose relative to episode start (odom frame)."""
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

    def _publish_odom_tf(self, stamp=None) -> None:
        if not self._driver_ready:
            return
        try:
            pose = self._driver.get_pose()
        except Exception as exc:
            self.get_logger().debug(f"get_pose failed: {exc}")
            return

        odom_x, odom_y, odom_yaw = self._habitat_pose_to_odom(pose)
        if stamp is None:
            stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = odom_x
        odom.pose.pose.position.y = odom_y
        odom.pose.pose.orientation = self._yaw_to_quaternion(odom_yaw)
        self._odom_pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self._odom_frame
        tf_msg.child_frame_id = self._base_frame
        tf_msg.transform.translation.x = odom_x
        tf_msg.transform.translation.y = odom_y
        tf_msg.transform.rotation = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf_msg)

    async def _execute_move(self, goal_handle):
        goal = goal_handle.request
        action = DIRECTION_TO_ACTION[goal.direction]
        feedback = DiscreteMove.Feedback()
        result = DiscreteMove.Result()

        completed = 0
        collided = False
        for _ in range(goal.steps):
            step_result = self._driver.step(action, 1)
            if not step_result.success:
                result.success = False
                result.collided = step_result.collided
                result.message = step_result.message
                goal_handle.abort()
                return result
            completed += step_result.steps_completed
            collided = collided or step_result.collided
            feedback.steps_completed = completed
            goal_handle.publish_feedback(feedback)
            # Keep depth/odom stamps aligned after each discrete step (critical during 360° turns).
            self._publish_sensor_data()

        result.success = True
        result.collided = collided
        result.message = "OK"
        goal_handle.succeed()
        return result

    def destroy_node(self) -> bool:
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
