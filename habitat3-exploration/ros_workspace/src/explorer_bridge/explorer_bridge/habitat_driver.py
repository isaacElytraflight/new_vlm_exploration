"""Habitat sim driver via IPC to habitat_engine.py."""

from __future__ import annotations

from explorer_bridge.driver_protocol import ObservationData, StepResult
from explorer_bridge.habitat_ipc import DEFAULT_SOCKET_PATH, HabitatIpcClient, HabitatIpcError


class HabitatDriver:
    def __init__(self, socket_path: str = DEFAULT_SOCKET_PATH) -> None:
        self._client = HabitatIpcClient(socket_path)

    def get_observations(self) -> ObservationData:
        return self._client.get_observations()

    def step(self, action: str, count: int = 1) -> StepResult:
        return self._client.step(action, count)

    def reset(self) -> None:
        self._client.reset()

    def shutdown(self) -> None:
        self._client.shutdown()


def is_engine_alive(socket_path: str = DEFAULT_SOCKET_PATH) -> bool:
    try:
        HabitatIpcClient(socket_path).get_observations()
        return True
    except HabitatIpcError:
        return False
