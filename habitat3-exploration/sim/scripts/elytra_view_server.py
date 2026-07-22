#!/usr/bin/env python3
"""HTTP server exposing latest JPEG frames from ROS topics or files.

Reads view definitions from /tmp/elytra_views.json (written by Elytra on sim connect).
Serves GET /views/{id}/frame.jpg on port 8090 (default).

Supports rgb8/bgr8 images and 32FC1 depth (debug colormap via depth_debug_viz).
"""

from __future__ import annotations

import json
import os
import sys
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import unquote

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image

# Prefer bind-mounted /workspace/scripts so depth_debug_viz edits hot-reload.
_SCRIPTS = Path("/workspace/scripts")
if _SCRIPTS.is_dir():
    sys.path.insert(0, str(_SCRIPTS))
else:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from depth_debug_viz import (  # noqa: E402
    DEFAULT_RANGE_MAX,
    DEFAULT_RANGE_MIN,
    DEFAULT_SAT_EPS,
    DEFAULT_SENSOR_FAR,
    depth_to_debug_bgr,
)

CONFIG_PATH = os.environ.get("ELYTRA_VIEWS_CONFIG", "/tmp/elytra_views.json")
PORT = int(os.environ.get("ELYTRA_VIEW_SERVER_PORT", "8090"))
FILE_POLL_HZ = float(os.environ.get("ELYTRA_VIEW_FILE_POLL_HZ", "10"))
DEPTH_RANGE_MIN = float(os.environ.get("ELYTRA_DEPTH_RANGE_MIN", str(DEFAULT_RANGE_MIN)))
DEPTH_RANGE_MAX = float(os.environ.get("ELYTRA_DEPTH_RANGE_MAX", str(DEFAULT_RANGE_MAX)))
DEPTH_SENSOR_FAR = float(os.environ.get("ELYTRA_DEPTH_SENSOR_FAR", str(DEFAULT_SENSOR_FAR)))
DEPTH_SAT_EPS = float(os.environ.get("ELYTRA_DEPTH_SAT_EPS", str(DEFAULT_SAT_EPS)))


def _load_config() -> list[dict[str, Any]]:
    path = Path(CONFIG_PATH)
    if not path.is_file():
        return []
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return []
    return data if isinstance(data, list) else []


class FrameStore:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._frames: Dict[str, bytes] = {}

    def set_frame(self, view_id: str, jpeg_bytes: bytes) -> None:
        with self._lock:
            self._frames[view_id] = jpeg_bytes

    def get_frame(self, view_id: str) -> Optional[bytes]:
        with self._lock:
            return self._frames.get(view_id)


class ElytraViewRelay(Node):
    def __init__(self, views: list[dict[str, Any]], store: FrameStore) -> None:
        super().__init__("elytra_view_relay")
        self._store = store
        self._views = views

        for view in views:
            view_id = str(view.get("id", "")).strip()
            view_type = str(view.get("type", "")).strip()
            if not view_id:
                continue

            if view_type == "ros-image":
                topic = str(view.get("topic", "")).strip()
                if topic:
                    self.create_subscription(
                        Image,
                        topic,
                        lambda msg, vid=view_id: self._on_image(vid, msg),
                        qos_profile_sensor_data,
                    )
                    self.get_logger().info(f"Subscribed {view_id} -> {topic} (ros-image)")
            elif view_type == "ros-compressed":
                topic = str(view.get("topic", "")).strip()
                if topic:
                    self.create_subscription(
                        CompressedImage,
                        topic,
                        lambda msg, vid=view_id: self._on_compressed(vid, msg),
                        1,
                    )
                    self.get_logger().info(f"Subscribed {view_id} -> {topic} (ros-compressed)")
            elif view_type == "file":
                file_path = str(view.get("path", "")).strip()
                if file_path:
                    period = 1.0 / max(FILE_POLL_HZ, 0.1)
                    self.create_timer(
                        period,
                        lambda fp=file_path, vid=view_id: self._poll_file(vid, fp),
                    )
                    self.get_logger().info(f"Polling {view_id} -> {file_path} (file)")

    def _encode_jpeg(self, bgr: np.ndarray) -> Optional[bytes]:
        ok, buf = cv2.imencode(".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not ok:
            return None
        return buf.tobytes()

    def _depth_to_bgr(self, msg: Image) -> np.ndarray:
        depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        return depth_to_debug_bgr(
            depth,
            range_min=DEPTH_RANGE_MIN,
            range_max=DEPTH_RANGE_MAX,
            sensor_far=DEPTH_SENSOR_FAR,
            sat_eps=DEPTH_SAT_EPS,
            with_legend=True,
        )

    def _on_image(self, view_id: str, msg: Image) -> None:
        try:
            enc = (msg.encoding or "").lower()
            if enc == "rgb8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            elif enc == "bgr8":
                bgr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif enc in ("32fc1", "32fc"):
                bgr = self._depth_to_bgr(msg)
            else:
                self.get_logger().warn(f"{view_id}: unsupported encoding {msg.encoding}")
                return
            jpeg = self._encode_jpeg(np.ascontiguousarray(bgr))
            if jpeg:
                self._store.set_frame(view_id, jpeg)
        except Exception as exc:
            self.get_logger().warn(f"{view_id}: image decode failed: {exc}")

    def _on_compressed(self, view_id: str, msg: CompressedImage) -> None:
        fmt = (msg.format or "").lower()
        data = bytes(msg.data)
        if not data:
            return
        if "png" in fmt:
            arr = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if arr is None:
                return
            jpeg = self._encode_jpeg(arr)
            if jpeg:
                self._store.set_frame(view_id, jpeg)
            return
        self._store.set_frame(view_id, data)

    def _poll_file(self, view_id: str, file_path: str) -> None:
        try:
            raw = Path(file_path).read_bytes()
        except OSError:
            return
        if not raw:
            return
        if raw[:2] == b"\xff\xd8":
            self._store.set_frame(view_id, raw)
            return
        arr = cv2.imdecode(np.frombuffer(raw, dtype=np.uint8), cv2.IMREAD_COLOR)
        if arr is None:
            return
        jpeg = self._encode_jpeg(arr)
        if jpeg:
            self._store.set_frame(view_id, jpeg)


def make_handler(store: FrameStore):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format: str, *args: Any) -> None:
            return

        def do_GET(self) -> None:
            path = unquote(self.path.split("?", 1)[0])
            prefix = "/views/"
            suffix = "/frame.jpg"
            if not (path.startswith(prefix) and path.endswith(suffix)):
                self.send_error(404)
                return
            view_id = path[len(prefix): -len(suffix)]
            frame = store.get_frame(view_id)
            if not frame:
                self.send_error(404, "No frame yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(frame)

    return Handler


def main() -> None:
    views = _load_config()
    store = FrameStore()

    rclpy.init()
    node = ElytraViewRelay(views, store)

    server = ThreadingHTTPServer(("0.0.0.0", PORT), make_handler(store))
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    node.get_logger().info(f"Elytra view server listening on :{PORT} ({len(views)} views)")

    try:
        rclpy.spin(node)
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
