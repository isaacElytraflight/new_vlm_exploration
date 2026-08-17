#!/usr/bin/env python3
"""Ground-floor spawn + stair disconnection helpers for Habitat navmesh.

MP3D houses like JmbYfDe2QKZ ship a navmesh where the default agent pose is on
an upper floor and stair risers can remain walkable under the default climb
limit. This module:

1. Selects the ground-floor navmesh island (lowest mean Y among large islands).
2. Exposes the agent_max_climb used to rebake so stairs stop bridging floors.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence


# Empirically: Habitat default agent_max_climb (0.2) still connects stair
# flights on JmbYfDe2QKZ; 0.15 splits floors into separate islands.
DEFAULT_AGENT_MAX_CLIMB = 0.15
DEFAULT_MIN_ISLAND_AREA_M2 = 5.0


@dataclass(frozen=True)
class IslandInfo:
    index: int
    mean_y: float
    area: float


def select_ground_floor_island(
    islands: Sequence[IslandInfo],
    *,
    min_area: float = DEFAULT_MIN_ISLAND_AREA_M2,
) -> int:
    """Return the navmesh island index for the bottom floor.

    Prefer the lowest ``mean_y`` among islands with ``area >= min_area``.
    If none qualify, fall back to the globally lowest ``mean_y`` (then largest
    area as a tie-break).
    """
    if not islands:
        raise ValueError("no islands")

    eligible: List[IslandInfo] = [i for i in islands if i.area >= min_area]
    pool: Sequence[IslandInfo] = eligible if eligible else islands
    best = min(pool, key=lambda i: (i.mean_y, -i.area, i.index))
    return int(best.index)


def summarize_islands(
    island_samples_y: Iterable[tuple[int, float]],
    island_areas: dict[int, float],
) -> List[IslandInfo]:
    """Build IslandInfo rows from (island_index, y) samples and area map."""
    buckets: dict[int, List[float]] = {}
    for idx, y in island_samples_y:
        buckets.setdefault(int(idx), []).append(float(y))
    out: List[IslandInfo] = []
    for idx, ys in buckets.items():
        if not ys:
            continue
        out.append(
            IslandInfo(
                index=idx,
                mean_y=sum(ys) / len(ys),
                area=float(island_areas.get(idx, 0.0)),
            )
        )
    return out
