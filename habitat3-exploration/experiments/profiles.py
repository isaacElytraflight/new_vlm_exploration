"""Named algorithm profiles for experiment matrix cells (Goal C brains)."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
from typing import Any, Dict, Mapping


# Canonical brain plugin ids accepted by explore_node / createExplorationBrain.
CANONICAL_BRAINS = frozenset(
    {
        "vlm_tree_dfs",
        "greedy_nearest",
        "vlm_frontier_graph",
        "vlm_choice_dijkstra",
    }
)

# Aliases → canonical (matches C++ factory).
BRAIN_ALIASES: Dict[str, str] = {
    "vlm_dfs": "vlm_tree_dfs",
    "vlm_tree_dfs": "vlm_tree_dfs",
    "greedy_nearest": "greedy_nearest",
    "vlm_frontier_graph": "vlm_frontier_graph",
    "vlm_choice_dijkstra": "vlm_choice_dijkstra",
}


@dataclass(frozen=True)
class ExplorationProfile:
    """ROS / launch knobs frozen into each run's config_json."""

    id: str
    brain_id: str
    dfs_prefer_highest_openness: bool = True
    parent_to_nearest_node: bool = True
    description: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


PROFILES: Dict[str, ExplorationProfile] = {
    "exploration_policy_vlm_default": ExplorationProfile(
        id="exploration_policy_vlm_default",
        brain_id="vlm_tree_dfs",
        dfs_prefer_highest_openness=True,
        parent_to_nearest_node=True,
        description="VLM tree DFS, highest openness first (default stack).",
    ),
    "exploration_policy_greedy": ExplorationProfile(
        id="exploration_policy_greedy",
        brain_id="greedy_nearest",
        dfs_prefer_highest_openness=True,
        parent_to_nearest_node=True,
        description="True nearest-frontier greedy (no tree, no VLM).",
    ),
}


def normalize_brain_id(brain_id: str) -> str:
    key = str(brain_id).strip()
    if key not in BRAIN_ALIASES:
        known = ", ".join(sorted(set(BRAIN_ALIASES) | CANONICAL_BRAINS))
        raise KeyError(f"Unknown brain {brain_id!r}; known: {known}")
    return BRAIN_ALIASES[key]


def get_profile(profile_id: str) -> ExplorationProfile:
    if profile_id not in PROFILES:
        known = ", ".join(sorted(PROFILES))
        raise KeyError(f"Unknown profile {profile_id!r}; known: {known}")
    return PROFILES[profile_id]


def default_profile_for_brain(brain_id: str) -> ExplorationProfile:
    """Synthetic profile when YAML specifies brain without a named profile."""
    nid = normalize_brain_id(brain_id)
    descriptions = {
        "vlm_tree_dfs": "VLM tree DFS (brain-only cell).",
        "greedy_nearest": "True nearest-frontier greedy (brain-only cell).",
        "vlm_frontier_graph": "Frontier graph + numeric VLM (Euclidean kNN cost v1).",
        "vlm_choice_dijkstra": "Dijkstra neighborhood + VLM choice log (score-argmax v1).",
    }
    return ExplorationProfile(
        id=f"brain:{nid}",
        brain_id=nid,
        dfs_prefer_highest_openness=True,
        parent_to_nearest_node=True,
        description=descriptions.get(nid, nid),
    )


def resolve_algorithm_profile(algo: Mapping[str, Any]) -> ExplorationProfile:
    """Resolve ExplorationProfile from algorithms[] cell (brain and/or profile)."""
    algorithm_id = str(algo["id"])
    brain_raw = algo.get("brain")
    profile_raw = algo.get("profile")

    if profile_raw:
        base = get_profile(str(profile_raw))
    elif brain_raw:
        base = default_profile_for_brain(str(brain_raw))
    elif algorithm_id in PROFILES:
        base = get_profile(algorithm_id)
    else:
        base = default_profile_for_brain(algorithm_id)

    if brain_raw:
        base = replace(base, brain_id=normalize_brain_id(str(brain_raw)))
    return base
