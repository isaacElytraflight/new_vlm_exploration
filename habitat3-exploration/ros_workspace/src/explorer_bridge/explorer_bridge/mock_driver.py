"""Mock habitat driver for unit/integration tests (no GPU)."""

from __future__ import annotations

import numpy as np

from explorer_bridge.driver_protocol import ObservationData, StepResult

# Known marker pixel so tests can detect false positives.
MARKER_RGB = (42, 84, 126)
MARKER_DEPTH = 1.25


class MockHabitatDriver:
    def __init__(self, height: int = 480, width: int = 640) -> None:
        self._height = height
        self._width = width
        self._collided = False
        self._alive = True
        self._step_count = 0
        self._last_action = ""

    @property
    def alive(self) -> bool:
        return self._alive

    def kill(self) -> None:
        self._alive = False

    def get_observations(self) -> ObservationData:
        if not self._alive:
            raise RuntimeError("mock engine is dead")
        rgb = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        rgb[0, 0] = MARKER_RGB
        depth = np.full((self._height, self._width), MARKER_DEPTH, dtype=np.float32)
        depth[1, 1] = MARKER_DEPTH + 0.5
        return ObservationData(rgb=rgb, depth=depth, collided=self._collided)

    def step(self, action: str, count: int = 1) -> StepResult:
        if not self._alive:
            return StepResult(
                success=False,
                collided=False,
                steps_completed=0,
                message="mock engine is dead",
            )
        self._last_action = action
        completed = max(0, int(count))
        self._step_count += completed
        if action == "move_forward":
            self._collided = False
        return StepResult(
            success=True,
            collided=self._collided,
            steps_completed=completed,
            message="OK (mock)",
        )

    def reset(self) -> None:
        self._collided = False
        self._step_count = 0

    def shutdown(self) -> None:
        self._alive = False
