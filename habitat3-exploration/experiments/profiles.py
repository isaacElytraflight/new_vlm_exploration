"""Named algorithm profiles for experiment matrix cells."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Dict


@dataclass(frozen=True)
class ExplorationProfile:
    """ROS / launch knobs frozen into each run's config_json."""

    id: str
    dfs_prefer_highest_openness: bool
    parent_to_nearest_node: bool
    description: str = ""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


PROFILES: Dict[str, ExplorationProfile] = {
    "exploration_policy_vlm_default": ExplorationProfile(
        id="exploration_policy_vlm_default",
        dfs_prefer_highest_openness=True,
        parent_to_nearest_node=True,
        description="VLM-rated DFS, highest openness first (default stack).",
    ),
    "exploration_policy_greedy": ExplorationProfile(
        id="exploration_policy_greedy",
        dfs_prefer_highest_openness=False,
        parent_to_nearest_node=False,
        description="VLM-rated DFS, lowest openness first (greedy baseline proxy).",
    ),
}


def get_profile(profile_id: str) -> ExplorationProfile:
    if profile_id not in PROFILES:
        known = ", ".join(sorted(PROFILES))
        raise KeyError(f"Unknown profile {profile_id!r}; known: {known}")
    return PROFILES[profile_id]
