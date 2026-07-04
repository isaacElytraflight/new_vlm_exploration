"""Depth CameraInfo must share the depth image stamp for depthimage_to_laserscan."""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from explorer_bridge.depth_camera_info_node import DepthCameraInfoNode
from explorer_bridge.explorer_bridge_node import ExplorerBridgeNode
from explorer_bridge.mock_driver import MockHabitatDriver
from conftest import MessageCollector, spin_node_background, wait_until

_TEST_DEPTH_TOPIC = "/test/depth_data_isolated"
_TEST_INFO_TOPIC = "/test/depth/camera_info_isolated"
_TEST_NODE_PARAMS = [
    Parameter("depth_topic", Parameter.Type.STRING, _TEST_DEPTH_TOPIC),
    Parameter("topic", Parameter.Type.STRING, _TEST_INFO_TOPIC),
]


def test_depth_camera_info_matches_depth_stamp_positive(ros_context):
    info_node = DepthCameraInfoNode(parameter_overrides=_TEST_NODE_PARAMS)
    stop = threading.Event()
    spin_node_background(info_node, stop)

    info_collector = MessageCollector()
    sub = Node("info_sub")
    sub.create_subscription(CameraInfo, _TEST_INFO_TOPIC, info_collector.callback, 10)
    spin_node_background(sub, stop)

    pub = Node("depth_pub")
    pub_pub = pub.create_publisher(Image, _TEST_DEPTH_TOPIC, qos_profile_sensor_data)
    spin_node_background(pub, stop)

    msg = Image()
    msg.header.stamp.sec = 42
    msg.header.stamp.nanosec = 123456789
    msg.header.frame_id = "depth_frame"
    msg.height = 480
    msg.width = 640
    msg.encoding = "32FC1"
    msg.step = 640 * 4
    msg.data = bytes(msg.step * msg.height)

    pub_pub.publish(msg)
    assert wait_until(lambda: len(info_collector.messages) > 0)

    info = info_collector.messages[-1]
    assert info.header.stamp.sec == 42
    assert info.header.stamp.nanosec == 123456789
    assert info.header.frame_id == "depth_frame"

    stop.set()
    pub.destroy_node()
    sub.destroy_node()
    info_node.destroy_node()


def test_odom_stamp_matches_depth_stamp_positive(ros_context):
    driver = MockHabitatDriver()
    bridge = ExplorerBridgeNode(driver=driver)
    stop = threading.Event()
    spin_node_background(bridge, stop)

    depth_collector = MessageCollector()
    from nav_msgs.msg import Odometry

    odom_collector = MessageCollector()
    sub = Node("sync_sub")
    sub.create_subscription(Image, "/depth_data", depth_collector.callback, qos_profile_sensor_data)
    sub.create_subscription(Odometry, "/odom", odom_collector.callback, 10)
    spin_node_background(sub, stop)

    assert wait_until(
        lambda: len(depth_collector.messages) > 0
        and len(depth_collector.messages) == len(odom_collector.messages)
    )

    for depth, odom in zip(depth_collector.messages, odom_collector.messages):
        assert depth.header.stamp.sec == odom.header.stamp.sec
        assert depth.header.stamp.nanosec == odom.header.stamp.nanosec

    stop.set()
    sub.destroy_node()
    bridge.destroy_node()


def test_depth_camera_info_without_depth_negative(ros_context):
    info_node = DepthCameraInfoNode(
        parameter_overrides=_TEST_NODE_PARAMS,
        namespace="test_depth_camera_info_negative",
    )
    stop = threading.Event()
    spin_node_background(info_node, stop)

    info_collector = MessageCollector()
    sub = Node("info_sub_negative")
    sub.create_subscription(CameraInfo, _TEST_INFO_TOPIC, info_collector.callback, 10)
    spin_node_background(sub, stop)

    import time

    time.sleep(0.5)
    assert len(info_collector.messages) == 0

    stop.set()
    sub.destroy_node()
    info_node.destroy_node()
