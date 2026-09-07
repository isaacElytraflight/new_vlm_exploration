#!/usr/bin/env python3
"""Subscribe to /cmd_vel and dispatch DiscreteMove goals (ROS 2 cmd_vel bridge)."""

from __future__ import annotations

import math
import threading
from typing import Optional

import rclpy
from explorer_msgs.action import DiscreteMove
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node

from explorer_bridge.cmd_vel_to_discrete import (
    CmdVelThresholds,
    apply_realtime_rate_cap,
    cmd_vel_to_intent,
)
from explorer_bridge.wall_collision_diag import (
    direction_name,
    log_event,
    nearest_plan_metrics,
    path_to_xy,
)


class CmdVelToDiscreteNode(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_to_discrete")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("action_name", "/movement/discrete_move")
        self.declare_parameter("angular_threshold", 0.05)
        self.declare_parameter("linear_threshold", 0.03)
        self.declare_parameter("turn_over_drive_ratio", 1.0)
        self.declare_parameter("dispatch_rate_hz", 20.0)
        self.declare_parameter("min_command_interval", 0.15)
        self.declare_parameter("realtime_mode", False)
        self.declare_parameter("realtime_max_linear_m_s", 0.1)
        self.declare_parameter("realtime_max_angular_deg_s", 30.0)
        # TEMP: wall collision diag
        self.declare_parameter("plan_topic", "/plan")
        self.declare_parameter("odom_topic", "/odom")

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        action_name = self.get_parameter("action_name").value
        self._thresholds = CmdVelThresholds(
            angular_threshold=float(self.get_parameter("angular_threshold").value),
            linear_threshold=float(self.get_parameter("linear_threshold").value),
            turn_over_drive_ratio=float(self.get_parameter("turn_over_drive_ratio").value),
        )
        dispatch_hz = max(1.0, float(self.get_parameter("dispatch_rate_hz").value))
        self._min_interval = max(0.0, float(self.get_parameter("min_command_interval").value))
        self._realtime_mode = bool(self.get_parameter("realtime_mode").value)
        self._max_linear = float(self.get_parameter("realtime_max_linear_m_s").value)
        self._max_angular_deg = float(self.get_parameter("realtime_max_angular_deg_s").value)

        self._client = ActionClient(self, DiscreteMove, action_name)
        self._lock = threading.Lock()
        self._pending: Optional[tuple[int, int]] = None
        self._busy = False
        self._last_dispatch_ns = 0
        self._last_turn_direction: Optional[int] = None
        self._consecutive_turn_steps = 0
        self._last_lin = 0.0
        self._last_ang = 0.0
        # TEMP: wall collision diag — latest plan + odom for cross-track logs
        self._plan_xy: list[tuple[float, float]] = []
        self._odom_xyyaw: Optional[tuple[float, float, float]] = None

        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_cb, 10)
        self.create_subscription(
            Path, str(self.get_parameter("plan_topic").value), self._plan_cb, 10
        )
        self.create_subscription(
            Odometry, str(self.get_parameter("odom_topic").value), self._odom_cb, 20
        )
        self.create_timer(1.0 / dispatch_hz, self._dispatch_tick)
        self.get_logger().info(
            f"cmd_vel bridge listening on {cmd_vel_topic} -> {action_name} "
            f"(realtime_mode={self._realtime_mode})"
        )
        # TEMP: wall collision diag
        self.get_logger().info(
            "TEMP_WALL_COLLISION_DIAG writing /data/temp_wall_collision_diag.csv"
        )

    def _plan_cb(self, msg: Path) -> None:
        self._plan_xy = path_to_xy(msg.poses)

    def _odom_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._odom_xyyaw = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw,
        )

    def _cmd_vel_cb(self, msg: Twist) -> None:
        linear_x = float(msg.linear.x)
        angular_z = float(msg.angular.z)
        if self._realtime_mode:
            linear_x, angular_z = apply_realtime_rate_cap(
                linear_x,
                angular_z,
                max_linear_m_s=self._max_linear,
                max_angular_deg_s=self._max_angular_deg,
            )
        self._last_lin = linear_x
        self._last_ang = angular_z
        intent = cmd_vel_to_intent(
            linear_x,
            angular_z,
            self._thresholds,
            last_turn_direction=self._last_turn_direction,
            consecutive_turn_steps=self._consecutive_turn_steps,
        )
        if intent is None:
            self._last_turn_direction = None
            self._consecutive_turn_steps = 0
            return
        if intent.direction in (
            DiscreteMove.Goal.TURN_LEFT,
            DiscreteMove.Goal.TURN_RIGHT,
        ):
            if intent.direction == self._last_turn_direction:
                self._consecutive_turn_steps += 1
            else:
                self._consecutive_turn_steps = 1
            self._last_turn_direction = intent.direction
        else:
            self._last_turn_direction = None
            self._consecutive_turn_steps = 0
        with self._lock:
            self._pending = (intent.direction, intent.steps)

    def _dispatch_tick(self) -> None:
        if self._busy:
            return
        with self._lock:
            pending = self._pending
            self._pending = None
        if pending is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._min_interval > 0 and (now_ns - self._last_dispatch_ns) < int(
            self._min_interval * 1e9
        ):
            with self._lock:
                if self._pending is None:
                    self._pending = pending
            return

        if not self._client.server_is_ready():
            with self._lock:
                self._pending = pending
            return

        direction, steps = pending
        goal = DiscreteMove.Goal()
        goal.direction = direction
        goal.steps = steps
        self._busy = True
        self._last_dispatch_ns = now_ns

        # TEMP: wall collision diag — cmd_vel intent + plan geometry at dispatch
        fields = {
            "lin_x": self._last_lin,
            "ang_z": self._last_ang,
            "direction": direction_name(direction),
            "note": f"consec_turns={self._consecutive_turn_steps}",
        }
        if self._odom_xyyaw is not None:
            ox, oy, yaw = self._odom_xyyaw
            fields.update(
                nearest_plan_metrics(ox, oy, yaw, self._plan_xy),
                pose_x=ox,
                pose_y=oy,
                yaw_rad=yaw,
            )
        else:
            fields["plan_n"] = len(self._plan_xy)
        log_event("dispatch", **fields)

        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_done)

    def _goal_done(self, future) -> None:
        try:
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                self._busy = False
                return
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda _f: setattr(self, "_busy", False))
        except Exception as exc:
            self.get_logger().warn(f"DiscreteMove dispatch failed: {exc}")
            self._busy = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToDiscreteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
