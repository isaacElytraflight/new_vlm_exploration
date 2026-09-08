# Future goals

Durable design backlog for **VLM-aided room exploration** (Habitat 3.0 + ROS 2 Jazzy + elytra-bridge).  
Day-to-day debugging stays in [JOURNAL.md](JOURNAL.md). Specs for **not-yet-built** work live here.

**How to use this file**

1. Pick **one** open goal at a time.
2. Record decisions here before coding.
3. Implement in a dedicated session (TDD).
4. When shipped, move the item to [Completed](#completed) and trim this file.

Contract reference: [habitat3-exploration/ros_workspace/design_doc.md](habitat3-exploration/ros_workspace/design_doc.md).

---

## Current baseline (as of 2026-09-07)

```text
/depth_data + /camera_info + /odom
        → known_pose_pc_mapper (C++, default)  →  /grid_map
        → explore_node (frontier DFS + VLM scores)   ← monolithic “brain”
        → Nav2 NavigateToPose  →  /cmd_vel  →  DiscreteMove (Habitat)
        → on nav fail: explore thrash BACK/FWD DiscreteMove (×5), then mark frontier
```

| Piece | Location | Notes |
|-------|----------|--------|
| Mapping | C++ `known_pose_pc_mapper` (`use_pc_mapper:=true`) | Subsample 8; pose-gated integrate; Bresenham early-stop on OCCUPIED; grid inflate 0.05 m |
| Exploration | `explore_node` | **Monolithic** frontier-tree DFS + VLM; return-home; thrash unstick |
| Motion | `cmd_vel_to_discrete` | Drive vs turn via `|ang|/|lin|` ratio (default 1.0) |
| Nav2 | `nav2_params.yaml` | No-recovery BT; `allow_unknown: true`; costmap inflation **0.15 m** |
| Ablations | `experiments/` + Elytra Start/Stop Ablation | Packages + WAL resume; **Resume / Fresh** campaign UI |

**Known baseline debt:** ablation `greedy_nearest` is still tree+VLM with knobs flipped (`dfs_prefer_highest=false`, `parent_to_nearest=false`) — **not** true nearest-frontier greedy. Fixing that is part of Goal **C** below, not a profile tweak.

---

## Completed

| Goal | Shipped | Pointers |
|------|---------|----------|
| **A — PC → 2D occupancy** | 2026-08-26 … 2026-09-06 | C++ mapper, wall band 0.05–1.0 m |
| **B — Ablation harness v1** | 2026-08-31 | YAML matrix, SQLite, aggregate tables, Elytra ablation buttons |
| **Nav robustness (partial)** | 2026-09-06 | Return-home; stuck policy; wall-unstick; cmd_vel ratio; inflation 0.15 m |
| **B2 — Run packages + resume** | 2026-09-06 | Artifact package, media/events, `render_run.py`, WAL resume |
| **B2.1 — Campaign ops + thrash recovery** | 2026-09-07 | Fresh vs resume UI; per-run scratch layout; interrupt media cleanup; tmux start retries; return-home abandon; DiscreteMove thrash BACK/FWD×growing (max 5) |

---

## Open goals (primary next steps)

### C — Swappable exploration brains *(co-top with paper eval)*

**Why:** Ablations must compare *algorithms*, not ROS param toggles. High-level decision-making must be pluggable so each matrix cell loads a distinct brain implementation.

**Architecture (decision recorded 2026-09-07)**

```text
shared stack (keep):
  frontier detection, mapping, Nav2, DiscreteMove, thrash recovery, packaging

swappable brain (new):
  ExplorationBrain interface
    on_map / on_frontiers / on_arrived / select_next_goal / …
  one implementation file (or package) per algorithm
  ablation YAML → algorithm_id → brain plugin id (not ad-hoc param soup)
```

**Shared vs brain-owned**

| Shared (orchestration) | Owned by each brain |
|------------------------|---------------------|
| Occupancy `/grid_map`, frontier *detection* geometry | Frontier *memory* structure (none / tree / graph) |
| Nav2 + thrash recovery to a goal pose | Goal selection policy |
| Episode lifecycle, metrics, media | Whether/when to call VLM and how to interpret it |
| Ablation matrix wiring | Visited / dead / backtrack semantics |

**Algorithms to ship under this interface**

| ID (proposed) | Description | Tree? | VLM? |
|---------------|-------------|-------|------|
| `vlm_tree_dfs` | **Current** explore_node behavior: frontier tree DFS, numeric openness scores, highest-first (today’s default) | Yes (tree) | Yes (0–5 scores) |
| `greedy_nearest` | **True greedy** (thesis baseline): among all live frontiers, go to the **nearest** (path length preferred; Euclidean OK for v1 if Nav2 distance deferred). **No tree. No VLM wait.** Re-detect / re-pick after each arrival. | No | No |
| `vlm_frontier_graph` | Frontiers as a **graph**: on instantiate, link each node to **N≈5** neighbors by **Nav2 route cost** (not pure Cartesian). Degree ~3–5 after pruning. Traverse neighbor→neighbor; mark visited on arrival; pick next among *alive* neighbors via **same numeric VLM scores**. | Graph | Yes (0–5 scores) |
| `vlm_choice_dijkstra` | **Thesis-faithful copy**: Dijkstra / graph traversal as in [aarush_thesis.pdf](aarush_thesis.pdf), but VLM is asked **which frontier to take next** (multi-image: each candidate view + overview map + “efficient explorer” instructions) instead of labeling each frontier with a scalar. | Graph + Dijkstra | Yes (**choice**, not score) |

**Refactor steps (implementation order)**

1. Extract `ExplorationBrain` API + thin `explore_node` loop that only detects frontiers, calls brain, navigates, recovers.
2. Lift current tree+VLM DFS into `vlm_tree_dfs` plugin (behavior parity with today’s default).
3. Implement `greedy_nearest` (replace today’s false “greedy” profile).
4. Implement `vlm_frontier_graph` (Nav2-distance kNN edges + numeric VLM).
5. Implement `vlm_choice_dijkstra` (thesis procedure + multi-view VLM choice).
6. Wire ablation `algorithms[].brain` (or `profile` → brain id) + smoke matrix cells for all four.

**Exit criteria**

- [ ] Matrix can run ≥2 brains without recompiling knobs by hand
- [ ] `greedy_nearest` never builds a frontier tree and never blocks on VLM
- [ ] Graph brains log edge costs (Nav2) and visited sets in event JSONL
- [ ] Choice brain logs VLM prompt/response + selected frontier id
- [ ] Smoke: 1 scene × 2 seeds × 4 brains completes under resume/fresh

**Non-goals for C**

- Hot-swapping brains mid-episode
- Learning / RL policies
- Changing mapper or Nav2 stack as part of the brain

---

### 1 — Paper-ready evaluation *(co-top with C)*

Depends on **C** for honest algorithm cells. After brains exist:

| Gap | Notes |
|-----|--------|
| Privileged FOV 90° | Radius wired; FOV mode still open |
| Scale to ~50 seeds / cell | Smoke packages + resume/fresh proven; next is matrix scale |
| Thesis-aligned matrix | Include true greedy + tree VLM + graph VLM + choice VLM |

**Eval rule:** Paper coverage stays decoupled from perception `/grid_map` (privileged Habitat reveal).

**Package reminder (shipped):** each run under `sim/data/experiments/<exp>/<run_id>/` with `manifest.json`, metrics CSVs, side-by-side 10× MP4, event JSONL; viz via `python experiments/render_run.py <run_dir>/`. Elytra: **Resume incomplete** (YAML `experiment_id`) or **Fresh campaign** (`--fresh` + timestamp suffix).

---

### 2 — Runtime memory (after long-batch soak)

1. Bridge JPEG pool unbounded queue  
2. Coverage worker full-map churn  
3. `maprender_node.trajectory` unbounded  
4. Host/WSL Vmmem — cap via `~/.wslconfig` (`memory=6GB`); quit Docker when idle  

### 3 — Sim2real (deferred)

RTAB-Map / VO on the real robot — not required for sim ablations.

### Non-goals

- Concurrent multi-project Elytra / cloud multi-user  
- TARE/DSVP inside Habitat (trajectory replay later)  
- Full 3D frontier exploration  

---

## Suggested sequencing

| Step | Work | Gate |
|------|------|------|
| **1** ✓ | Artifact package + timelapse + event logs + `render_run.py` | One smoke run package ≤50 MB; pretty figures |
| **2** ✓ | Crash-safe resume + progress UI | Kill mid-batch; restart; only unfinished cells run |
| **2.1** ✓ | Fresh campaign UI + thrash unstick + tmux/return-home hardening | Smoke campaign can start reliably; no infinite return-home |
| **3** | **Swappable brains (Goal C)** — interface + lift tree DFS + true greedy | Ablation cell selects a brain file/plugin; greedy has no tree/VLM |
| **4** | Graph + numeric VLM brain; thesis choice+Dijkstra brain | Four-way smoke matrix green |
| **5** | FOV 90° + 50-seed paper matrix | Thesis-aligned campaign |
| **6** | Memory hardening | Multi-hour episode without OOM |
| **7** | Sim2real mapping | Hardware-ready |

---

## Discussion log

### 2026-09-07 — Swappable brains become co-top priority

Today’s “greedy” ablation is a mislabeled tree+VLM variant. Next architecture: pluggable `ExplorationBrain` with four implementations — (1) current tree DFS+scores, (2) true nearest-frontier greedy (no tree, no VLM), (3) Nav2-distance frontier graph + numeric VLM, (4) thesis Dijkstra/graph + VLM *choice* over labeled frontier images + map overview. Paper eval waits on honest algorithm cells.

### 2026-09-07 — B2.1 milestone closeout

Shipped campaign isolation (resume vs fresh), package layout cleanup, interrupt encode/cleanup, resilient tmux episode start, return-home guard abandon + near-goal short-circuit, and DiscreteMove thrash recovery (BACK/FWD alternating, growing steps, max 5) for occlusion traps.

### 2026-09-06 — Ablation packages + resume planned as next milestone

PART 1 (recording/viz) and PART 2 (crash-safe resume) before algorithm scale-up. Viz is a Python script, not a browser app.

### 2026-09-06 — Mapping/nav arc closeout

C++ mapper, async coverage, curvature ratio, wall-unstick, inflation 0.15 m; TEMP diags removed.

### 2026-08-31 — Goal B v1 shipped

CLI + SQLite + Elytra ablation entrypoints.
