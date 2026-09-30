# ADR-001: Retreat about a meter when stuck, not back to the previous frontier

## Status
Accepted

## Date
2026-09-30

## Context
Discrete lattice navigation plans in DiscreteMove steps on `/grid_map`. When a new frontier cannot be reached and the failure is classified as stuck, recovery used to deflate inflation and navigate all the way back to the previous scan pose. That full return was a debugging sanctuary from the Nav2 quantization era. With a lattice planner it often walked 5 m or more back across the room before retrying.

The robot only needs a nearby pose that is no longer wedged.

## Decision
After wall thrash, retreat at most `stuck_retreat_m` (default 1.0 m) from the current pose toward the previous scan pose. If that pose is already inside the budget, go there. The hop uses an arrival radius of at most 0.40 m so the default 1 m goal-accept radius does not count the retreat as already finished.

Theoretical DFS backtrack still adopts the parent in place and does not move.

## Alternatives considered
- Keep the full return to the previous frontier. Rejected: it crosses the room to undo a local wedge.
- Retrace the exact breadcrumb path. Rejected for this milestone: a straight cap toward the last known-good scan pose is enough, and the lattice planner paths to that point.

## Consequences
- `ablation_run_20260930_140918` (`greedy_nearest`, `JmbYfDe2QKZ`): both seeds mapped 75.0 / 84.1 m² (89.2%). Seed 1 ended `success`. Seed 0 ended `stuck` with the same mapped area because the final pose was wedged and no live frontier was reachable.
- If the short hop fails, the existing alternate scan-pose fallback can still travel farther.
