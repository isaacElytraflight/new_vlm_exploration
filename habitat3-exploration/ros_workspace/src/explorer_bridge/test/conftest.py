import sys
from pathlib import Path

# Allow `from test_helpers import ...` in sibling test modules.
sys.path.insert(0, str(Path(__file__).resolve().parent))

import threading
import time
from typing import Callable, List

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class MessageCollector:
    def __init__(self) -> None:
        self.messages: List = []

    def callback(self, msg) -> None:
        self.messages.append(msg)


def spin_node_background(node: Node, stop_event: threading.Event) -> threading.Thread:
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def _spin() -> None:
        while not stop_event.is_set() and rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.05)
            except Exception:
                break

    thread = threading.Thread(target=_spin, daemon=True)
    thread.start()
    return thread


def wait_until(condition: Callable[[], bool], timeout_sec: float = 5.0) -> bool:
    deadline = time.time() + timeout_sec
    while time.time() < deadline:
        if condition():
            return True
        time.sleep(0.05)
    return False


def init_ros_once() -> None:
    if not rclpy.ok():
        rclpy.init()


def shutdown_ros_if_ok() -> None:
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def ros_context():
    init_ros_once()
    yield
    shutdown_ros_if_ok()
