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


class ExplorerDriver(Protocol):
    def get_observations(self) -> ObservationData:
        ...

    def step(self, action: str, count: int = 1) -> StepResult:
        ...

    def reset(self) -> None:
        ...

    def shutdown(self) -> None:
        ...
