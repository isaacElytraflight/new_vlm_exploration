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


def test_tf_lookup_at_depth_stamp_without_fallback_positive(ros_context):
    """On depth arrival, odom→base_link at that stamp must already be in the TF buffer."""
    from rclpy.duration import Duration
    from tf2_ros import Buffer, TransformListener

    driver = MockHabitatDriver()
    bridge = ExplorerBridgeNode(driver=driver)
    stop = threading.Event()
    spin_node_background(bridge, stop)

    listener = Node("tf_depth_order_listener")
    tf_buffer = Buffer()
    TransformListener(tf_buffer, listener)
    spin_node_background(listener, stop)

    results: list[bool] = []

    def on_depth(msg: Image) -> None:
        try:
            tf_buffer.lookup_transform(
                "odom",
                "base_link",
                msg.header.stamp,
                timeout=Duration(seconds=0.0),
            )
            results.append(True)
        except Exception:
            results.append(False)

    sub = Node("depth_order_sub")
    sub.create_subscription(Image, "/depth_data", on_depth, qos_profile_sensor_data)
    spin_node_background(sub, stop)

    assert wait_until(lambda: len(results) >= 3, timeout_sec=5.0)
    # Allow a brief warm-up miss on the first frame; after that stamp lookups must work.
    assert results.count(True) >= 2, f"stamp TF lookups failed: {results}"

    stop.set()
    sub.destroy_node()
    listener.destroy_node()
    bridge.destroy_node()


def test_stamp_tf_miss_must_not_use_latest_policy_negative():
    """Negative control: mapper policy rejects stamp misses (no silent latest TF)."""
    from explorer_bridge.scan_to_occupancy import should_integrate_with_tf

    assert should_integrate_with_tf(stamp_lookup_ok=False) is False
    assert should_integrate_with_tf(stamp_lookup_ok=True) is True


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
