#!/usr/bin/env python3
"""Unit tests for ground-floor island selection (no Habitat import).

Positive: pick the large low-floor island when upper floors exist.
Negative: reject tiny low islands / empty input safely.
Harness: intentional AssertionError is detectable.
"""

from __future__ import annotations

import pytest

from floor_constraint import IslandInfo, select_ground_floor_island


def test_harness_positive_control():
    assert 1 + 1 == 2


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_selects_lowest_large_island_positive():
    """JmbYf-like: big ground floor + upstairs + tiny landings → ground."""
    islands = [
        IslandInfo(index=0, mean_y=0.08, area=63.0),
        IslandInfo(index=1, mean_y=2.67, area=25.0),
        IslandInfo(index=2, mean_y=0.08, area=2.0),
        IslandInfo(index=3, mean_y=3.28, area=1.2),
    ]
    assert select_ground_floor_island(islands, min_area=5.0) == 0


def test_prefers_larger_when_same_height_positive():
    islands = [
        IslandInfo(index=4, mean_y=0.1, area=10.0),
        IslandInfo(index=7, mean_y=0.1, area=40.0),
        IslandInfo(index=1, mean_y=2.0, area=50.0),
    ]
    assert select_ground_floor_island(islands, min_area=5.0) == 7


def test_tiny_low_ignored_when_large_upper_exists_negative():
    """Tiny low pads are not eligible; spawn on the only large (upper) island."""
    islands = [
        IslandInfo(index=2, mean_y=0.05, area=1.0),
        IslandInfo(index=1, mean_y=2.5, area=30.0),
    ]
    assert select_ground_floor_island(islands, min_area=5.0) == 1


def test_all_tiny_falls_back_to_lowest_y_positive():
    islands = [
        IslandInfo(index=2, mean_y=0.05, area=1.0),
        IslandInfo(index=1, mean_y=2.5, area=2.0),
    ]
    assert select_ground_floor_island(islands, min_area=5.0) == 2


def test_empty_islands_raises_negative():
    with pytest.raises(ValueError, match="no islands"):
        select_ground_floor_island([], min_area=5.0)


def test_default_climb_disconnects_stairs_constant():
    """Document the climb threshold used to split stair-connected islands."""
    from floor_constraint import DEFAULT_AGENT_MAX_CLIMB

    assert DEFAULT_AGENT_MAX_CLIMB <= 0.15
    assert DEFAULT_AGENT_MAX_CLIMB > 0.0
