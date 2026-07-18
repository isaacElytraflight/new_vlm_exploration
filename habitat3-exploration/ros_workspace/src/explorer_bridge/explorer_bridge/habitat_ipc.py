"""Unix domain socket JSON-line client for habitat_engine.py."""

from __future__ import annotations

import base64
import json
import socket
from typing import Any, Dict

import numpy as np

from explorer_bridge.driver_protocol import MapData, ObservationData, PoseData, StepResult

DEFAULT_SOCKET_PATH = "/tmp/habitat_engine.sock"
CONNECT_TIMEOUT_SEC = 5.0
REQUEST_TIMEOUT_SEC = 30.0


class HabitatIpcError(RuntimeError):
    pass


class HabitatIpcClient:
    def __init__(self, socket_path: str = DEFAULT_SOCKET_PATH) -> None:
        self._socket_path = socket_path

    def _request(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            sock.settimeout(CONNECT_TIMEOUT_SEC)
            sock.connect(self._socket_path)
            sock.settimeout(REQUEST_TIMEOUT_SEC)
            line = json.dumps(payload) + "\n"
            sock.sendall(line.encode("utf-8"))
            chunks = []
            while True:
                chunk = sock.recv(65536)
                if not chunk:
                    break
                chunks.append(chunk)
                if b"\n" in chunk:
                    break
            raw = b"".join(chunks).split(b"\n", 1)[0]
            if not raw:
                raise HabitatIpcError("empty response from habitat engine")
            data = json.loads(raw.decode("utf-8"))
            if not data.get("ok", False):
                raise HabitatIpcError(data.get("error", "habitat engine request failed"))
            return data
        except (OSError, json.JSONDecodeError) as exc:
            raise HabitatIpcError(str(exc)) from exc
        finally:
            sock.close()

    @staticmethod
    def _decode_array(b64: str, shape: list, dtype: str) -> np.ndarray:
        raw = base64.b64decode(b64)
        np_dtype = np.dtype(dtype)
        arr = np.frombuffer(raw, dtype=np_dtype).reshape(shape)
        return np.ascontiguousarray(arr)

    def get_observations(self) -> ObservationData:
        data = self._request({"cmd": "get_obs"})
        rgb = self._decode_array(data["rgb_b64"], data["rgb_shape"], "uint8")
        depth = self._decode_array(data["depth_b64"], data["depth_shape"], "float32")
        birdseye = None
        if "birdseye_b64" in data and "birdseye_shape" in data:
            birdseye = self._decode_array(data["birdseye_b64"], data["birdseye_shape"], "uint8")
        return ObservationData(
            rgb=rgb,
            depth=depth,
            collided=bool(data.get("collided", False)),
            birdseye=birdseye,
        )

    def get_observations_with_pose(self) -> tuple[ObservationData, PoseData]:
        data = self._request({"cmd": "get_obs_and_pose"})
        rgb = self._decode_array(data["rgb_b64"], data["rgb_shape"], "uint8")
        depth = self._decode_array(data["depth_b64"], data["depth_shape"], "float32")
        birdseye = None
        if "birdseye_b64" in data and "birdseye_shape" in data:
            birdseye = self._decode_array(data["birdseye_b64"], data["birdseye_shape"], "uint8")
        obs = ObservationData(
            rgb=rgb,
            depth=depth,
            collided=bool(data.get("collided", False)),
            birdseye=birdseye,
        )
        pose = PoseData(
            x=float(data["x"]),
            y=float(data["y"]),
            yaw_rad=float(data["yaw_rad"]),
        )
        return obs, pose

    def step(self, action: str, count: int = 1) -> StepResult:
        data = self._request({"cmd": "step", "action": action, "count": int(count)})
        return StepResult(
            success=True,
            collided=bool(data.get("collided", False)),
            steps_completed=int(data.get("steps_completed", count)),
            message=data.get("message", "OK"),
        )

    def reset(self) -> None:
        self._request({"cmd": "reset"})

    def get_pose(self) -> PoseData:
        data = self._request({"cmd": "get_pose"})
        return PoseData(
            x=float(data["x"]),
            y=float(data["y"]),
            yaw_rad=float(data["yaw_rad"]),
        )

    def get_map(self) -> MapData:
        data = self._request({"cmd": "get_map"})
        grid = self._decode_array(data["grid_b64"], data["grid_shape"], "int8")
        return MapData(
            grid=grid,
            resolution=float(data["resolution"]),
            origin_x=float(data["origin_x"]),
            origin_y=float(data["origin_y"]),
        )

    def shutdown(self) -> None:
        try:
            self._request({"cmd": "shutdown"})
        except HabitatIpcError:
            pass
