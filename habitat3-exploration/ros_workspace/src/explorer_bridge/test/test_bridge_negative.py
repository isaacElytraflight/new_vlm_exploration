"""Negative integration tests — invalid goals, wrong topics, dead driver."""

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
from explorer_bridge.mock_driver import MockHabitatDriver
from conftest import (
    MessageCollector,
    spin_node_background,
    wait_until,
)


def test_invalid_direction_rejected(ros_context):
    driver = MockHabitatDriver()
    bridge = ExplorerBridgeNode(driver=driver)
    stop = threading.Event()
    spin_node_background(bridge, stop)

    client_node = Node("test_invalid_client")
    client = ActionClient(client_node, DiscreteMove, "/movement/discrete_move")
    spin_node_background(client_node, stop)
    assert wait_until(lambda: client.server_is_ready())

    goal = DiscreteMove.Goal()
    goal.direction = 99  # invalid
    goal.steps = 1
    future = client.send_goal_async(goal)
    assert wait_until(lambda: future.done())
    goal_handle = future.result()
    assert not goal_handle.accepted

    stop.set()
    client_node.destroy_node()
    bridge.destroy_node()


def test_wrong_topic_receives_no_messages(ros_context):
    driver = MockHabitatDriver()
    bridge = ExplorerBridgeNode(driver=driver)
    stop = threading.Event()
    spin_node_background(bridge, stop)

    collector = MessageCollector()
    sub_node = Node("test_wrong_topic")
    sub_node.create_subscription(
        Image, "/image_data_fake", collector.callback, qos_profile_sensor_data
    )
    spin_node_background(sub_node, stop)

    # Wait long enough that real topic would have published several times.
    assert wait_until(lambda: len(collector.messages) > 0, timeout_sec=1.5) is False
    assert len(collector.messages) == 0

    stop.set()
    sub_node.destroy_node()
    bridge.destroy_node()


def test_dead_driver_stops_publishing(ros_context):
    driver = MockHabitatDriver()
    bridge = ExplorerBridgeNode(driver=driver)
    stop = threading.Event()
    spin_node_background(bridge, stop)

    rgb_collector = MessageCollector()
    sub_node = Node("test_dead_driver_sub")
    sub_node.create_subscription(Image, "/image_data", rgb_collector.callback, qos_profile_sensor_data)
    spin_node_background(sub_node, stop)

    assert wait_until(lambda: len(rgb_collector.messages) > 0)
    before = len(rgb_collector.messages)

    driver.kill()
    assert wait_until(lambda: len(rgb_collector.messages) == before, timeout_sec=3.0)

    stop.set()
    sub_node.destroy_node()
    bridge.destroy_node()


def test_dead_driver_action_aborts(ros_context):
    driver = MockHabitatDriver()
    bridge = ExplorerBridgeNode(driver=driver)
    stop = threading.Event()
    spin_node_background(bridge, stop)

    client_node = Node("test_dead_action_client")
    client = ActionClient(client_node, DiscreteMove, "/movement/discrete_move")
    spin_node_background(client_node, stop)
    assert wait_until(lambda: client.server_is_ready())

    driver.kill()
    goal = DiscreteMove.Goal()
    goal.direction = DiscreteMove.Goal.FORWARD
    goal.steps = 1
    future = client.send_goal_async(goal)
    assert wait_until(lambda: future.done())
    goal_handle = future.result()
    assert goal_handle.accepted

    result_future = goal_handle.get_result_async()
    assert wait_until(lambda: result_future.done())
    result = result_future.result().result
    assert result.success is False

    stop.set()
    client_node.destroy_node()
    bridge.destroy_node()
