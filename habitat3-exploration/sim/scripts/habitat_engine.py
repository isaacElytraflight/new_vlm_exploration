#!/usr/bin/env python3
"""Habitat sim engine with Unix-socket JSON-line IPC (no ROS dependencies).

Commands (one JSON object per line):
  {"cmd": "get_obs"}
  {"cmd": "get_pose"}
  {"cmd": "get_map"}
  {"cmd": "step", "action": "move_forward", "count": 1}
  {"cmd": "reset"}
  {"cmd": "shutdown"}
"""

from __future__ import annotations

import base64
import json
import os
import signal
import socket
import sys
from typing import Any, Dict, Optional, Tuple

import habitat_sim
import magnum as mn
import numpy as np

from explored_map import compute_revealed_grid

MAP_METERS_PER_PIXEL = float(os.environ.get("HABITAT_MAP_RESOLUTION", "0.05"))
# Sensor range for the incremental "explored" map reveal. Frontiers appear at
# this boundary, so it must be > the frontier filter min radius (1.0 m).
SENSOR_RANGE_M = float(os.environ.get("HABITAT_SENSOR_RANGE_M", "4.0"))

SCENE = os.environ.get(
    "HABITAT_SCENE",
    "/data/scene_datasets/habitat-test-scenes/skokloster-castle.glb",
)
SOCKET_PATH = os.environ.get("HABITAT_ENGINE_SOCKET", "/tmp/habitat_engine.sock")

_running = True


def _handle_stop(signum, frame) -> None:
    global _running
    _running = False


def make_sim() -> habitat_sim.Simulator:
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = SCENE
    sim_cfg.enable_physics = True
    sim_cfg.load_semantic_mesh = False
    sim_cfg.gpu_device_id = int(os.environ.get("HABITAT_GPU_DEVICE_ID", "0"))

    rgb = habitat_sim.CameraSensorSpec()
    rgb.uuid = "rgb"
    rgb.sensor_type = habitat_sim.SensorType.COLOR
    rgb.resolution = [480, 640]
    rgb.position = [0.0, 1.5, 0.0]

    depth = habitat_sim.CameraSensorSpec()
    depth.uuid = "depth"
    depth.sensor_type = habitat_sim.SensorType.DEPTH
    depth.resolution = [480, 640]
    depth.position = [0.0, 1.5, 0.0]

    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [rgb, depth]

    # Explicit actuation so move_backward is available alongside defaults.
    agent_cfg.action_space = {
        "move_forward": habitat_sim.agent.ActionSpec(
            "move_forward",
            habitat_sim.agent.ActuationSpec(amount=0.25),
        ),
        "move_backward": habitat_sim.agent.ActionSpec(
            "move_backward",
            habitat_sim.agent.ActuationSpec(amount=0.25),
        ),
        "turn_left": habitat_sim.agent.ActionSpec(
            "turn_left",
            habitat_sim.agent.ActuationSpec(amount=10.0),
        ),
        "turn_right": habitat_sim.agent.ActionSpec(
            "turn_right",
            habitat_sim.agent.ActuationSpec(amount=10.0),
        ),
    }

    return habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg]))


class HabitatEngine:
    def __init__(self) -> None:
        if not os.path.exists(SCENE):
            raise SystemExit(f"Scene not found: {SCENE} — run download_data.sh first.")
        self._sim = make_sim()
        self._collided = False
        self._last_obs: Dict[str, Any] = self._sim.get_sensor_observations()
        if self._last_obs:
            self._collided = False
        # Accumulated mask of navmesh cells the agent has observed so far.
        self._explored: Optional[np.ndarray] = None

    def close(self) -> None:
        self._sim.close()

    def reset(self) -> None:
        self._sim.reset()
        self._last_obs = self._sim.get_sensor_observations()
        self._collided = False
        self._explored = None

    def get_obs(self) -> Tuple[np.ndarray, np.ndarray, bool]:
        rgb = self._last_obs.get("rgb")
        depth = self._last_obs.get("depth")
        if rgb is None or depth is None:
            obs = self._sim.get_sensor_observations()
            rgb = obs.get("rgb")
            depth = obs.get("depth")
        rgb_arr = np.ascontiguousarray(rgb[..., :3], dtype=np.uint8)
        depth_arr = np.ascontiguousarray(depth.squeeze(), dtype=np.float32)
        return rgb_arr, depth_arr, self._collided

    def step(self, action: str, count: int) -> Tuple[bool, int]:
        allowed = {"move_forward", "move_backward", "turn_left", "turn_right"}
        if action not in allowed:
            raise ValueError(f"unknown action {action!r}")
        completed = 0
        collided = False
        for _ in range(max(0, int(count))):
            obs = self._sim.step(action)
            self._last_obs = obs
            collided = bool(obs.get("collided", False))
            self._collided = collided
            completed += 1
        return collided, completed

    def get_pose(self) -> Tuple[float, float, float]:
        """Return (x, y, yaw_rad) in a 2D map frame (habitat X-Z plane)."""
        state = self._sim.get_agent(0).get_state()
        pos = state.position
        rot = state.rotation
        if not isinstance(rot, mn.Quaternion):
            # habitat returns a numpy-quaternion (w, x, y, z); magnum's
            # constructor needs an explicit (Vector3 xyz, scalar w).
            rot = mn.Quaternion(
                mn.Vector3(float(rot.x), float(rot.y), float(rot.z)), float(rot.w)
            )
        forward = rot.transform_vector_normalized(mn.Vector3(0.0, 0.0, -1.0))
        yaw = float(np.arctan2(forward.x, -forward.z))
        return float(pos[0]), float(pos[2]), yaw

    def get_map(self) -> Tuple[np.ndarray, float, float, float]:
        """Return (grid int8 HxW, resolution, origin_x, origin_y).

        The grid is an *incrementally revealed* occupancy map: FREE(0) /
        OCCUPIED(100) for cells the agent has observed, UNKNOWN(-1) otherwise.
        This is what makes frontier detection possible (vs. the raw navmesh,
        which is fully known and therefore frontier-free)."""
        pf = self._sim.pathfinder
        if not pf.is_loaded:
            raise RuntimeError("pathfinder not loaded")
        pos = self._sim.get_agent(0).get_state().position
        # get_topdown_view needs the vertical slice height; use the agent's
        # current floor height so the navmesh slice matches where it stands.
        height = float(pos[1])
        top_down = pf.get_topdown_view(MAP_METERS_PER_PIXEL, height)
        navigable = np.asarray(top_down, dtype=bool)

        lower, _upper = pf.get_bounds()
        origin_x = float(lower[0])
        origin_y = float(lower[2])
        # Column = world x, row = world z (consistent with the bridge's TF /
        # odom and the C++ pixelToWorld in frontier_detection).
        agent_col = (float(pos[0]) - origin_x) / MAP_METERS_PER_PIXEL
        agent_row = (float(pos[2]) - origin_y) / MAP_METERS_PER_PIXEL
        radius_px = SENSOR_RANGE_M / MAP_METERS_PER_PIXEL

        grid, self._explored = compute_revealed_grid(
            navigable, self._explored, agent_col, agent_row, radius_px
        )
        return grid, MAP_METERS_PER_PIXEL, origin_x, origin_y


def _encode_obs_response(engine: HabitatEngine) -> Dict[str, Any]:
    rgb, depth, collided = engine.get_obs()
    return {
        "ok": True,
        "rgb_b64": base64.b64encode(rgb.tobytes()).decode("ascii"),
        "rgb_shape": list(rgb.shape),
        "depth_b64": base64.b64encode(depth.tobytes()).decode("ascii"),
        "depth_shape": list(depth.shape),
        "collided": collided,
    }


def _handle_request(engine: HabitatEngine, payload: Dict[str, Any]) -> Dict[str, Any]:
    cmd = payload.get("cmd")
    if cmd == "get_obs":
        return _encode_obs_response(engine)
    if cmd == "get_pose":
        try:
            x, y, yaw = engine.get_pose()
        except Exception as exc:
            return {"ok": False, "error": str(exc)}
        return {"ok": True, "x": x, "y": y, "yaw_rad": yaw}
    if cmd == "get_map":
        try:
            grid, resolution, origin_x, origin_y = engine.get_map()
        except Exception as exc:
            return {"ok": False, "error": str(exc)}
        return {
            "ok": True,
            "grid_b64": base64.b64encode(grid.tobytes()).decode("ascii"),
            "grid_shape": list(grid.shape),
            "resolution": resolution,
            "origin_x": origin_x,
            "origin_y": origin_y,
        }
    if cmd == "step":
        action = payload.get("action", "")
        count = int(payload.get("count", 1))
        try:
            collided, completed = engine.step(action, count)
        except ValueError as exc:
            return {"ok": False, "error": str(exc)}
        return {
            "ok": True,
            "collided": collided,
            "steps_completed": completed,
            "message": "OK",
        }
    if cmd == "reset":
        engine.reset()
        return {"ok": True, "message": "reset"}
    if cmd == "shutdown":
        global _running
        _running = False
        return {"ok": True, "message": "shutdown"}
    return {"ok": False, "error": f"unknown cmd {cmd!r}"}


def _serve_client(engine: HabitatEngine, conn: socket.socket) -> None:
    buffer = b""
    with conn:
        while _running:
            chunk = conn.recv(65536)
            if not chunk:
                break
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                if not line.strip():
                    continue
                try:
                    payload = json.loads(line.decode("utf-8"))
                    response = _handle_request(engine, payload)
                except json.JSONDecodeError as exc:
                    response = {"ok": False, "error": f"invalid json: {exc}"}
                conn.sendall((json.dumps(response) + "\n").encode("utf-8"))
                if payload.get("cmd") == "shutdown":
                    return


def run_server(engine: HabitatEngine, socket_path: str = SOCKET_PATH) -> None:
    if os.path.exists(socket_path):
        os.unlink(socket_path)
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(socket_path)
    server.listen(5)
    server.settimeout(1.0)
    print(f"Habitat engine listening on {socket_path} (scene={SCENE})")
    try:
        while _running:
            try:
                conn, _ = server.accept()
            except socket.timeout:
                continue
            _serve_client(engine, conn)
    finally:
        server.close()
        if os.path.exists(socket_path):
            os.unlink(socket_path)


def main() -> None:
    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)
    engine = HabitatEngine()
    try:
        run_server(engine)
    finally:
        engine.close()
        print("Habitat engine stopped.")


if __name__ == "__main__":
    main()
