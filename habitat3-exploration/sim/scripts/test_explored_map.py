#!/usr/bin/env python3
"""Unit tests for the incremental explored-map reveal logic.

Positive controls assert that partial observation produces UNKNOWN cells and
therefore frontiers (free cells adjacent to unknown). Negative controls assert
that a fully-observed connected region produces NO frontiers — exactly the
failure mode of the old "publish the whole navmesh" map, where exploration
finished instantly because no frontier could ever exist.

Pure NumPy; runs under pytest or standalone (`python test_explored_map.py`).
"""

from __future__ import annotations

import numpy as np

from explored_map import FREE, OCCUPIED, UNKNOWN, compute_revealed_grid


def _count_frontier_cells(grid: np.ndarray) -> int:
    """Count FREE cells that have at least one UNKNOWN 8-neighbour.

    Mirrors the C++ findFrontierContours definition of a frontier.
    """
    h, w = grid.shape
    free = grid == FREE
    unknown = grid == UNKNOWN
    unknown_neighbor = np.zeros((h, w), dtype=bool)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            r0, r1 = max(0, dr), h + min(0, dr)
            c0, c1 = max(0, dc), w + min(0, dc)
            unknown_neighbor[r0:r1, c0:c1] |= unknown[r0 - dr:r1 - dr, c0 - dc:c1 - dc]
    return int(np.count_nonzero(free & unknown_neighbor))


def _open_room(h: int = 120, w: int = 120) -> np.ndarray:
    """A large open navigable room bounded by a one-cell wall."""
    navigable = np.zeros((h, w), dtype=bool)
    navigable[1:h - 1, 1:w - 1] = True
    return navigable


def test_partial_reveal_creates_unknown_and_frontiers():
    """POSITIVE: a small sensor radius in a large room leaves unobserved space,
    so the revealed grid must contain UNKNOWN cells and at least one frontier."""
    navigable = _open_room()
    grid, explored = compute_revealed_grid(
        navigable, None, agent_col=60, agent_row=60, radius_px=20.0
    )
    assert np.count_nonzero(grid == FREE) > 0, "expected observed free space"
    assert np.count_nonzero(grid == UNKNOWN) > 0, "expected unobserved (unknown) space"
    assert _count_frontier_cells(grid) > 0, "expected at least one frontier"


def test_full_reveal_has_no_frontiers_negative_control():
    """NEGATIVE CONTROL: a radius covering the whole connected region observes
    everything, so there must be NO frontiers (the old fully-known-map bug)."""
    navigable = _open_room()
    grid, _ = compute_revealed_grid(
        navigable, None, agent_col=60, agent_row=60, radius_px=10_000.0
    )
    assert np.count_nonzero(grid == UNKNOWN) == 0, "connected region should be fully known"
    assert _count_frontier_cells(grid) == 0, "fully-observed map must have no frontiers"


def test_walls_block_reveal_into_far_room():
    """A solid wall between two rooms must prevent the far room from being
    revealed (the robot cannot see through walls). The doorway-less wall keeps
    the second room UNKNOWN."""
    h, w = 60, 120
    navigable = np.zeros((h, w), dtype=bool)
    navigable[5:h - 5, 5:55] = True      # left room
    navigable[5:h - 5, 65:115] = True     # right room (no opening; col 55-65 wall)
    grid, _ = compute_revealed_grid(
        navigable, None, agent_col=30, agent_row=30, radius_px=10_000.0
    )
    # Left room observed as free; right room never reached -> stays unknown.
    assert grid[30, 30] == FREE
    assert np.all(grid[5:h - 5, 65:115] == UNKNOWN), "far room must stay unknown"


def test_reveal_through_doorway_reaches_far_room():
    """With a doorway, the flood fill reaches the far room (frontier exploration
    can progress room-to-room)."""
    h, w = 60, 120
    navigable = np.zeros((h, w), dtype=bool)
    navigable[5:h - 5, 5:55] = True
    navigable[5:h - 5, 65:115] = True
    navigable[28:32, 55:65] = True        # doorway connecting the rooms
    grid, _ = compute_revealed_grid(
        navigable, None, agent_col=30, agent_row=30, radius_px=10_000.0
    )
    assert np.count_nonzero(grid[5:h - 5, 65:115] == FREE) > 0, "far room reachable via door"


def test_explored_mask_accumulates_across_calls():
    """Observed area must persist and grow as the agent moves."""
    navigable = _open_room()
    grid1, explored1 = compute_revealed_grid(
        navigable, None, agent_col=30, agent_row=30, radius_px=15.0
    )
    seen1 = int(np.count_nonzero(explored1))
    grid2, explored2 = compute_revealed_grid(
        navigable, explored1, agent_col=90, agent_row=90, radius_px=15.0
    )
    seen2 = int(np.count_nonzero(explored2))
    assert seen2 > seen1, "explored region should grow after moving"
    assert np.all(explored2[explored1]), "previously observed cells stay observed"


def test_agent_off_navmesh_seeds_from_nearest_navigable():
    """If the agent cell is marginally off the discretised navmesh, the reveal
    still seeds from a nearby navigable cell instead of revealing nothing."""
    navigable = _open_room()
    navigable[60, 60] = False  # poke a hole exactly under the agent
    grid, _ = compute_revealed_grid(
        navigable, None, agent_col=60, agent_row=60, radius_px=20.0
    )
    assert np.count_nonzero(grid == FREE) > 0, "should still reveal via nearby seed"


def test_occupied_cells_are_revealed_walls():
    """Walls adjacent to observed free space are reported OCCUPIED (known), not
    left UNKNOWN, so the room boundary is a hard edge rather than a frontier."""
    navigable = _open_room()
    grid, _ = compute_revealed_grid(
        navigable, None, agent_col=60, agent_row=60, radius_px=10_000.0
    )
    assert np.count_nonzero(grid == OCCUPIED) > 0, "bounding wall should be known-occupied"


def _run_all() -> int:
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    failures = 0
    for test in tests:
        try:
            test()
            print(f"PASS {test.__name__}")
        except AssertionError as exc:
            failures += 1
            print(f"FAIL {test.__name__}: {exc}")
    print(f"\n{len(tests) - failures}/{len(tests)} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(_run_all())
