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
        → explore_node (frontier DFS + VLM scores)
        → Nav2 NavigateToPose  →  /cmd_vel  →  DiscreteMove (Habitat)
        → on nav fail: explore thrash BACK/FWD DiscreteMove (×5), then mark frontier
```

| Piece | Location | Notes |
|-------|----------|--------|
| Mapping | C++ `known_pose_pc_mapper` (`use_pc_mapper:=true`) | Subsample 8; pose-gated integrate; Bresenham early-stop on OCCUPIED; grid inflate 0.05 m |
| Exploration | `explore_node` | Frontier tree DFS; VLM openness; return-home guard (abandon-safe); thrash unstick |
| Motion | `cmd_vel_to_discrete` | Drive vs turn via `|ang|/|lin|` ratio (default 1.0) |
| Nav2 | `nav2_params.yaml` | No-recovery BT; `allow_unknown: true`; costmap inflation **0.15 m** |
| Ablations | `experiments/` + Elytra Start/Stop Ablation | Packages + WAL resume; **Resume / Fresh** campaign UI |

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

### 1 — Paper-ready evaluation

| Gap | Notes |
|-----|--------|
| Privileged FOV 90° | Radius wired; FOV mode still open |
| True greedy-without-VLM | Confirm profile semantics |
| Scale to ~50 seeds / cell | Smoke packages + resume/fresh proven; next is matrix scale |

**Eval rule:** Paper coverage stays decoupled from perception `/grid_map` (privileged Habitat reveal).

**Package reminder (shipped):** each run under `sim/data/experiments/<exp>/<run_id>/` with `manifest.json`, metrics CSVs, side-by-side 10× MP4, event JSONL; viz via `python experiments/render_run.py <run_dir>/`. Elytra: **Resume incomplete** (YAML `experiment_id`) or **Fresh campaign** (`--fresh` + timestamp suffix).

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
| 3 | FOV 90° + true greedy + 50-seed matrix | Thesis-aligned campaign |
| 4 | Memory hardening | Multi-hour episode without OOM |
| 5 | Sim2real mapping | Hardware-ready |

---

## Discussion log

### 2026-09-07 — B2.1 milestone closeout

Shipped campaign isolation (resume vs fresh), package layout cleanup, interrupt encode/cleanup, resilient tmux episode start, return-home guard abandon + near-goal short-circuit, and DiscreteMove thrash recovery (BACK/FWD alternating, growing steps, max 5) for occlusion traps. Next: paper-scale eval matrix.

### 2026-09-06 — Ablation packages + resume planned as next milestone

PART 1 (recording/viz) and PART 2 (crash-safe resume) before algorithm scale-up. Viz is a Python script, not a browser app.

### 2026-09-06 — Mapping/nav arc closeout

C++ mapper, async coverage, curvature ratio, wall-unstick, inflation 0.15 m; TEMP diags removed.

### 2026-08-31 — Goal B v1 shipped

CLI + SQLite + Elytra ablation entrypoints.
