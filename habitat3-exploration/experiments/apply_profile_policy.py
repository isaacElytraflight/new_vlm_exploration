"""Policy for ablation apply_exploration_profile after explore launch.

Brain selection is authoritative at launch (selected_brain.id / brain_id:=).
``ros2 param set /explore brain_id`` does not recreate the brain. Profile apply
only refreshes DFS knobs and is therefore best-effort for ablation cells.
"""

from __future__ import annotations


def apply_profile_failure_is_fatal(*, brain_set_at_launch: bool) -> bool:
    """Whether a failed apply_exploration_profile should abort the run cell.

    When the brain was already selected at launch, never abort the cell on
    apply failure (including transient ``Node not found`` / DDS blips).
    """
    return not bool(brain_set_at_launch)
