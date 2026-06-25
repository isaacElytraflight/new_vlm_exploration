#!/usr/bin/env python3
"""Incremental "explored" occupancy map for frontier-based exploration.

The Habitat pathfinder exposes the *complete* navmesh as a top-down view, i.e.
a fully-known map (navigable vs. not). Frontier exploration, however, needs an
*incrementally revealed* map: a frontier is the boundary between observed-free
space and not-yet-observed (UNKNOWN) space. On a fully-known map there are no
unknown cells, so no frontiers can ever be found.

This module reveals navmesh cells within sensor range of the agent via a
navmesh-bounded flood fill (so the robot cannot "see" through walls). Cells that
have never been observed are reported as UNKNOWN (-1). As the agent moves and
rotates, the observed region grows and frontiers naturally appear at sensor-range
boundaries and at doorways leading to unexplored rooms.

Pure NumPy — no habitat / ROS dependencies, so it is unit-testable in isolation.
"""

from __future__ import annotations

from collections import deque
from typing import Optional, Tuple

import numpy as np

FREE = 0
OCCUPIED = 100
UNKNOWN = -1


def _reveal_adjacent_walls(seen_free: np.ndarray, navigable: np.ndarray) -> np.ndarray:
    """Mark non-navigable cells in the 8-neighbourhood of any seen-free cell.

    Once a room's floor is observed, the walls bounding it become known too.
    Implemented with NumPy slicing (no SciPy dependency).
    """
    h, w = seen_free.shape
    dilated = np.zeros((h, w), dtype=bool)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            r0, r1 = max(0, dr), h + min(0, dr)
            c0, c1 = max(0, dc), w + min(0, dc)
            dilated[r0:r1, c0:c1] |= seen_free[r0 - dr:r1 - dr, c0 - dc:c1 - dc]
    return dilated & (~navigable)


def _find_navigable_seed(
    navigable: np.ndarray, row: int, col: int, search: int = 4
) -> Optional[Tuple[int, int]]:
    """Return the agent cell if navigable, else the nearest navigable cell in a
    small window (the agent can sit slightly off the discretised navmesh)."""
    h, w = navigable.shape
    if 0 <= row < h and 0 <= col < w and navigable[row, col]:
        return row, col
    best = None
    best_d2 = None
    for dr in range(-search, search + 1):
        for dc in range(-search, search + 1):
            nr, nc = row + dr, col + dc
            if 0 <= nr < h and 0 <= nc < w and navigable[nr, nc]:
                d2 = dr * dr + dc * dc
                if best_d2 is None or d2 < best_d2:
                    best_d2 = d2
                    best = (nr, nc)
    return best


def compute_revealed_grid(
    navigable: np.ndarray,
    explored: Optional[np.ndarray],
    agent_col: float,
    agent_row: float,
    radius_px: float,
) -> Tuple[np.ndarray, np.ndarray]:
    """Reveal navmesh cells within ``radius_px`` of the agent and return an
    occupancy grid plus the accumulated ``explored`` mask.

    Args:
        navigable: bool HxW, True where the navmesh is navigable (free).
        explored:  bool HxW accumulator of previously observed cells, or None.
        agent_col / agent_row: agent cell (column = world x, row = world y/z).
        radius_px: sensor range in pixels.

    Returns:
        (grid, explored_out):
            grid: int8 HxW with FREE(0) / OCCUPIED(100) / UNKNOWN(-1).
            explored_out: updated bool mask of observed cells.
    """
    navigable = np.asarray(navigable, dtype=bool)
    h, w = navigable.shape

    if explored is None or explored.shape != navigable.shape:
        explored = np.zeros((h, w), dtype=bool)
    else:
        explored = np.asarray(explored, dtype=bool).copy()

    seen_free = np.zeros((h, w), dtype=bool)

    seed = _find_navigable_seed(navigable, int(round(agent_row)), int(round(agent_col)))
    if seed is not None:
        seed_r, seed_c = seed
        r2 = float(radius_px) * float(radius_px)
        visited = np.zeros((h, w), dtype=bool)
        queue = deque()
        queue.append((seed_r, seed_c))
        visited[seed_r, seed_c] = True
        while queue:
            r, c = queue.popleft()
            seen_free[r, c] = True
            for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nr, nc = r + dr, c + dc
                if (
                    0 <= nr < h
                    and 0 <= nc < w
                    and not visited[nr, nc]
                    and navigable[nr, nc]
                    and (nr - seed_r) ** 2 + (nc - seed_c) ** 2 <= r2
                ):
                    visited[nr, nc] = True
                    queue.append((nr, nc))

    walls = _reveal_adjacent_walls(seen_free, navigable)
    explored |= seen_free
    explored |= walls

    grid = np.full((h, w), UNKNOWN, dtype=np.int8)
    grid[explored & navigable] = FREE
    grid[explored & (~navigable)] = OCCUPIED
    return grid, explored
