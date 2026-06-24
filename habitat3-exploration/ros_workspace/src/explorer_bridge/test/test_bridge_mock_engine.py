"""Positive integration tests with MockHabitatDriver."""

from __future__ import annotations

import threading

import pytest
import rclpy
from explorer_msgs.action import DiscreteMove
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from explorer_bridge.explorer_bridge_node import ExplorerBridgeNode
from explorer_bridge.image_utils import image_to_depth_array, image_to_rgb_array
from explorer_bridge.mock_driver import MARKER_DEPTH, MARKER_RGB, MockHabitatDriver
from conftest import (
    MessageCollector,
    spin_node_background,
    wait_until,
)


def test_bridge_publishes_image_topics_with_marker_pixels(ros_context):
    driver = MockHabitatDriver()
    bridge = ExplorerBridgeNode(driver=driver)
    stop = threading.Event()
    spin_node_background(bridge, stop)

    rgb_collector = MessageCollector()
    depth_collector = MessageCollector()

    sub_node = Node("test_subscriber")
    sub_node.create_subscription(Image, "/image_data", rgb_collector.callback, qos_profile_sensor_data)
    sub_node.create_subscription(Image, "/depth_data", depth_collector.callback, qos_profile_sensor_data)
    sub_thread = spin_node_background(sub_node, stop)

    assert wait_until(lambda: len(rgb_collector.messages) > 0)
    assert wait_until(lambda: len(depth_collector.messages) > 0)

    rgb_msg = rgb_collector.messages[-1]
    depth_msg = depth_collector.messages[-1]
    assert rgb_msg.encoding == "rgb8"
    assert depth_msg.encoding == "32FC1"
    assert rgb_msg.height == 480 and rgb_msg.width == 640
    assert depth_msg.height == 480 and depth_msg.width == 640

    rgb = image_to_rgb_array(rgb_msg)
    depth = image_to_depth_array(depth_msg)
    assert tuple(rgb[0, 0]) == MARKER_RGB
    assert depth[1, 1] == pytest.approx(MARKER_DEPTH + 0.5)

    stop.set()
    sub_thread.join(timeout=2.0)
    sub_node.destroy_node()
    bridge.destroy_node()


def test_discrete_move_action_success(ros_context):
    driver = MockHabitatDriver()
    bridge = ExplorerBridgeNode(driver=driver)
    stop = threading.Event()
    spin_node_background(bridge, stop)

    client_node = Node("test_action_client")
    client = ActionClient(client_node, DiscreteMove, "/movement/discrete_move")
    spin_node_background(client_node, stop)
    assert wait_until(lambda: client.server_is_ready())

    goal = DiscreteMove.Goal()
    goal.direction = DiscreteMove.Goal.FORWARD
    goal.steps = 2
    future = client.send_goal_async(goal)
    assert wait_until(lambda: future.done())
    goal_handle = future.result()
    assert goal_handle.accepted

    result_future = goal_handle.get_result_async()
    assert wait_until(lambda: result_future.done())
    result = result_future.result().result
    assert result.success is True
    assert result.collided is False

    stop.set()
    client_node.destroy_node()
    bridge.destroy_node()
