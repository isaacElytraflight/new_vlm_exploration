"""Shared types for explorer driver backends."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass
class ObservationData:
    rgb: np.ndarray
    depth: np.ndarray
    collided: bool


@dataclass
class StepResult:
    success: bool
    collided: bool
    steps_completed: int
    message: str = ""


@dataclass
class PoseData:
    x: float
    y: float
    yaw_rad: float


@dataclass
class MapData:
    grid: np.ndarray  # int8 HxW
    resolution: float
    origin_x: float
    origin_y: float


class ExplorerDriver(Protocol):
    def get_observations(self) -> ObservationData:
        ...

    def step(self, action: str, count: int = 1) -> StepResult:
        ...

    def get_pose(self) -> PoseData:
        ...

    def get_map(self) -> MapData:
        ...

    def reset(self) -> None:
        ...

    def shutdown(self) -> None:
        ...
