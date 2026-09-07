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

## Current baseline (as of 2026-09-06)

```text
/depth_data + /camera_info + /odom
        → known_pose_pc_mapper (C++, default)  →  /grid_map
        → explore_node (frontier DFS + VLM scores)
        → Nav2 NavigateToPose  →  /cmd_vel  →  DiscreteMove (Habitat)
```

| Piece | Location | Notes |
|-------|----------|--------|
| Mapping | C++ `known_pose_pc_mapper` (`use_pc_mapper:=true`) | Subsample 8; pose-gated integrate; Bresenham early-stop on OCCUPIED; grid inflate 0.05 m |
| Legacy laser map | `known_pose_mapper` | Available via `use_pc_mapper:=false` |
| Exploration | `explore_node` | Frontier tree DFS; VLM openness; return-home guard; wall-unstick |
| Motion | `cmd_vel_to_discrete` | Drive vs turn via `|ang|/|lin|` ratio (default 1.0) |
| Nav2 | `nav2_params.yaml` | No-recovery BT; `allow_unknown: true`; costmap inflation **0.15 m** |
| Coverage (live) | async IPC cache + `coverage_metrics_node` | Dashboard; not paper eval |
| Ablations | `experiments/` + Elytra Start/Stop Ablation | SQLite + smoke matrix CLI |

---

## Completed

| Goal | Shipped | Pointers |
|------|---------|----------|
| **A — PC → 2D occupancy** | 2026-08-26 … 2026-09-06 | C++ mapper, wall band 0.05–1.0 m, FREE ↛ overwrite OCCUPIED |
| **B — Ablation harness v1** | 2026-08-31 | YAML matrix, SQLite, aggregate tables, Elytra ablation buttons |
| **Nav robustness (partial)** | 2026-09-06 | Return-home guard; stuck policy 1 m / 60 s; wall-unstick; cmd_vel curvature ratio; inflation 0.15 m |

Historical discussion for A/B is in git history / JOURNAL; no need to keep the long design tables here.

---

## Open goals (next section of the project)

### 1 — Paper-ready evaluation (extend Goal B)

**Priority:** P1 for thesis tables.

| Gap | Status | Notes |
|-----|--------|--------|
| Privileged coverage FOV modes | Partial | Radius via `HABITAT_SENSOR_RANGE_M`; **90° FOV mode still open** (360° used today) |
| Coverage-vs-distance **plots** | Open | Tables done (`aggregate_results.py`); plot export deferred |
| True greedy-without-VLM baseline | Open | Profile exists; confirm it disables VLM ranking as intended |
| Scale to ~50 runs / cell | Open | Smoke matrix only so far |

**Eval rule:** Paper coverage must stay **decoupled from perception `/grid_map`** (privileged Habitat reveal), matching Aarush thesis Ch. 4.

### 2 — Runtime robustness & memory

**Priority:** P1 for long unattended ablations.

Known debt (2026-09-06 audit; not yet fixed):

1. Bridge JPEG `ThreadPoolExecutor` queue is **unbounded** under bind-mount latency → RAM growth.
2. Coverage worker reallocates full-map snapshots continuously → RSS pressure.
3. `maprender_node.trajectory` grows without bound for the episode.
4. Wall-unstick clearance gate (0.25 m) can miss NavFn `NO_VALID_PATH` when occupancy clearance is barely above threshold but inflated costmap blocks planning.

### 3 — Sim2real / perception pose (deferred)

- RTAB-Map / visual odometry as mapping authority on the real robot.
- Not required for sim ablations while privileged pose remains available.

### Non-goals (still)

- Concurrent multi-project Elytra / cloud multi-user.
- Running TARE/DSVP inside Habitat (thesis used trajectory replay — defer).
- Full 3D frontier exploration.

---

## Suggested sequencing

| Step | Work | Gate |
|------|------|------|
| 1 | Lock paper eval FOV (90 vs 360) + greedy baseline semantics | Matches thesis tables |
| 2 | Memory/robustness pass (JPEG coalesce, trajectory cap, coverage throttle, unstick gate) | Multi-hour episode without OOM / cascade stuck |
| 3 | Scale ablation matrix (~50 seeds) + plot export | Unattended batch green |
| 4 | Sim2real mapping (when hardware ready) | Real-robot `/grid_map` parity |

---

## Discussion log

### 2026-09-06 — Closeout before next project section

**Shipped this arc:** C++ PC mapper speed path; async coverage IPC; cmd_vel turn-over-drive ratio; wall-unstick + Nav2 error codes; costmap inflation 0.15 m; removed TEMP timing + TEMP wall-collision CSV instrumentation.

**Still open for next arc:** paper FOV/plots/greedy; memory leaks listed above; optional unstick on near-threshold `NO_VALID_PATH`.

### 2026-08-31 — Goal B v1 shipped

CLI + SQLite + Elytra ablation entrypoints. Remaining: FOV 90°, plots, greedy-without-VLM confirmation.

### 2026-08-28 — Return-home guard

After child nav failure, block sibling selection until return to scan node succeeds.

### 2026-08-26 — Goal A/B approach selection

A1 known-pose PC mapper selected; Goal B series orchestrator + SQLite + privileged eval.
