"""Mock habitat driver for unit/integration tests (no GPU)."""

from __future__ import annotations

import numpy as np

from explorer_bridge.driver_protocol import MapData, ObservationData, PoseData, StepResult

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
        self._x = 0.0
        self._y = 0.0
        self._yaw_rad = 0.0
        # When True, the next move_forward reports collided (for bridge tests).
        self.force_collide_on_forward = False

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
        birdseye = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        birdseye[2, 2] = (200, 100, 50)
        depth = np.full((self._height, self._width), MARKER_DEPTH, dtype=np.float32)
        depth[1, 1] = MARKER_DEPTH + 0.5
        return ObservationData(rgb=rgb, depth=depth, collided=self._collided, birdseye=birdseye)

    def get_observations_with_pose(self):
        return self.get_observations(), self.get_pose()

    def step(self, action: str, count: int = 1) -> StepResult:
        if not self._alive:
            return StepResult(
                success=False,
                collided=False,
                steps_completed=0,
                message="mock engine is dead",
            )
        self._last_action = action
        completed = 0
        for _ in range(max(0, int(count))):
            if action == "move_forward" and self.force_collide_on_forward:
                self._collided = True
                completed += 1
                return StepResult(
                    success=True,
                    collided=True,
                    steps_completed=completed,
                    message="OK (mock collide)",
                )
            if action == "move_forward":
                self._collided = False
                self._x += 0.25 * np.cos(self._yaw_rad)
                self._y += 0.25 * np.sin(self._yaw_rad)
            elif action == "move_backward":
                self._collided = False
                self._x -= 0.25 * np.cos(self._yaw_rad)
                self._y -= 0.25 * np.sin(self._yaw_rad)
            elif action == "turn_left":
                self._yaw_rad += np.radians(10.0)
            elif action == "turn_right":
                self._yaw_rad -= np.radians(10.0)
            completed += 1
            self._step_count += 1
        return StepResult(
            success=True,
            collided=self._collided,
            steps_completed=completed,
            message="OK (mock)",
        )

    def get_pose(self) -> PoseData:
        if not self._alive:
            raise RuntimeError("mock engine is dead")
        return PoseData(x=self._x, y=self._y, yaw_rad=self._yaw_rad)

    def get_map(self) -> MapData:
        if not self._alive:
            raise RuntimeError("mock engine is dead")
        grid = np.full((10, 10), 0, dtype=np.int8)
        grid[0, :] = 100
        grid[-1, :] = 100
        grid[:, 0] = 100
        grid[:, -1] = 100
        return MapData(grid=grid, resolution=0.05, origin_x=0.0, origin_y=0.0)

    def get_floor_area(self) -> tuple[float, float]:
        if not self._alive:
            raise RuntimeError("mock engine is dead")
        # 8x8 free interior of the 10x10 mock map at 0.05 m/px
        return 64 * (0.05 ** 2), 0.05

    def get_coverage_stats(self) -> tuple[float, float, float]:
        if not self._alive:
            raise RuntimeError("mock engine is dead")
        gt_m2, mpp = self.get_floor_area()
        # Mock: half the GT is "explored".
        return 0.5 * gt_m2, gt_m2, mpp

    def reset(self) -> None:
        self._collided = False
        self._step_count = 0
        self._x = 0.0
        self._y = 0.0
        self._yaw_rad = 0.0

    def shutdown(self) -> None:
        self._alive = False
