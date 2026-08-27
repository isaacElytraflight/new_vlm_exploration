# Future goals

Durable design backlog for **VLM-aided room exploration** (Habitat 3.0 + ROS 2 Jazzy + elytra-bridge).  
Day-to-day debugging stays in [JOURNAL.md](JOURNAL.md). Specs and decisions for not-yet-built work live here.

**How to use this file**

1. Pick **one** goal at a time.
2. Discuss and record decisions here **before** writing code.
3. When an approach is chosen, start a dedicated implementation session (TDD); do not silently implement from chat.
4. Append dated notes under [Discussion log](#discussion-log) as we converge.

**Suggested order:** Goal A (mapping) before Goal B (ablations). Mapping quality affects every coverage/frontier metric; the research advisor has already rejected the thin depth→laser strip as the primary map builder.

---

## Current baseline (what we build on)

```text
/depth_data  →  depth_to_laserscan  →  /scan (LaserScan)
                                           ↓
/odom (privileged)  +  known_pose_mapper  →  /grid_map (OccupancyGrid)
                                           ↓
                    explore_node frontiers + Nav2 costmaps
```

| Piece | Location | Notes |
|-------|----------|--------|
| Depth → laser | `explorer_bridge` (`depth_to_laserscan_node`, `scan_from_depth.py`) | Horizontal row band (`band_anchor: upper_third`); FOV-only |
| Scan → grid | `known_pose_mapper` + `scan_to_occupancy.py` | Privileged pose; **no** slam_toolbox in default launch |
| Frontiers | `explorer_mission` `explore_node` | Hard-subscribes `/grid_map` |
| Coverage (live) | `coverage_metrics_node` | `/exploration/debug/coverage_*`; **not persisted** |
| Episodes | Elytra / `start_sim.sh` | **Single-shot**; no N-run harness |

Contract reference: [habitat3-exploration/ros_workspace/design_doc.md](habitat3-exploration/ros_workspace/design_doc.md).

---

## Goal A — Point-cloud → 2D occupancy mapping

**Status:** v1 implemented (2026-08-26) — `known_pose_pc_mapper` behind `use_pc_mapper` launch flag.  
**Priority:** P1.

### Motivation

The depth→laser band pipeline is a poor fit for indoor RGB-D exploration:

- A thin row strip misses most wall geometry and still fought floor phantoms (see JOURNAL).
- LaserScan is a lossy projection of a dense depth image we already have.
- Advisor consensus: replace with a **point-cloud-like** representation, then **condense to a 2D grid** so frontier exploration stays 2D.

### Constraints

1. Exploration remains a **2D** problem: `OccupancyGrid` + frontier detection + Nav2.
2. Mapping authority must **not** be the thin laser band.
3. Prefer **existing ROS 2 (Jazzy) libraries** and current techniques where they fit.
4. Sim has **privileged pose** (Habitat GT / future T265) → loop-closure SLAM is optional for sim correctness.
5. Frontiers and `/grid_map` consumers should keep working with minimal API churn.

### What “point cloud → grid” must do

A raw cloud only supplies **occupied** hits. Frontiers and Nav2 need **free vs unknown** as well. Any serious replacement must:

1. Project depth → 3D points (`depth` frame → `map` via TF).
2. **Carve free space** (raycast / frustum) from the sensor origin through depth.
3. Mark **occupied** at endpoints (optionally height-filtered to reject floor/ceiling).
4. Publish `/grid_map` (and optionally a live `/points` for viz / Nav2 obstacle layers).

This is denser **RGB-D occupancy mapping**, not “dump points into cells.”

### Candidate approaches

| ID | Approach | Idea | Pros | Cons | Fit |
|----|----------|------|------|------|-----|
| **A1** | `depth_image_proc` + custom known-pose PC mapper | Depth → `PointCloud2`; replace `known_pose_mapper` with voxel/column integrate + per-pixel (or subsampled) ray carve in `map` | Keeps privileged-pose simplicity; full FOV density; mirrors today’s architecture; unit-testable pure math | Custom free-space code; tune height bands | **Selected** |
| **A2** | OctoMap (`octomap_server` / ROS 2 port) | Insert `PointCloud2` with probabilistic raycasting; project a 2D slice → OccupancyGrid | Mature package; free+occ from rays | 3D octree overhead; slice/height params; package glue in Docker | Spike / comparison if A1 free-space is painful |
| **A3** | RTAB-Map RGB-D | Aarush thesis stack: dense map + occupancy from RGB-D | Proven in thesis; loop closure for real robot | Heavy; redundant with GT pose in sim; different failure modes | Defer to **sim2real**, not sim ablations |
| **A4** | Nav2 STVL / PC obstacles only | Cloud for local costmap obstacles; separate global map | Good local avoidance | Does **not** replace global `/grid_map` / frontiers alone | Complementary later |
| **A5** | GPU TSDF (nvblox / similar) | Modern volumetric; slice to 2D | High-quality reconstruction | NVIDIA/Isaac deps; ops complexity | Out of scope unless GPU pipeline becomes a hard requirement |

### Selected approach: A1

**Working decision (2026-08-26):** Implement **A1** when Goal A is scheduled.

**Target data flow**

```text
/depth_data + /depth/camera_info
        → depth_image_proc (PointCloud2, camera frame)
        → height / voxel filter (optional node or in-mapper)
        → /points (and/or internal cloud)
        → known_pose point-cloud mapper (+ privileged TF)
        → /grid_map
        → explore_node frontiers (unchanged contract)
```

**Why A1 over A2/A3 now**

- Privileged pose already makes graph SLAM (A3) mostly dead weight in sim.
- We already own a known-pose integrator; extending it to RGB-D rays is the smallest conceptual jump.
- A2 remains a **fallback spike** if custom ray-carving becomes the bottleneck.
- Nav2 may still consume a derived `/scan` or PointCloud2 for obstacle layers, but **mapping authority** is the cloud→grid path.

### Open parameters (resolve at implementation kickoff)

| Question | Working default | Notes |
|----------|-----------------|--------|
| Height band | Wall band ~0.3–1.5 m above floor (tunable); reject near-floor hits for **occupied** | Free-space rays still use full depth |
| Free-space density | Subsample depth (e.g. every N px) for carve; denser for occupied endpoints | CPU budget in long episodes |
| Cloud persistence | Integrate-and-discard frames into the grid; optional short ring for viz | Avoid unbounded memory |
| `/scan` for Nav2 | Keep a thin derived scan **or** migrate costmap sources to PointCloud2 | Not the map authority |
| Floor phantoms | Occupied only from height-filtered points; saturated/far → UNKNOWN (same spirit as today’s `sat_eps`) | Carry lessons from JOURNAL |

### Implementation sketch (when we start — not now)

1. **RED:** pure-Python tests for ray-carve + height filter (positive: wall column marks occ + free along ray; negative: floor-only hits do not paint phantom walls).
2. Wire `depth_image_proc` (or equivalent in-process projection) in launch.
3. New mapper node (or evolve `known_pose_mapper`) consuming cloud + TF → `/grid_map`.
4. Feature-flag launch: old laser path vs PC path until parity.
5. Update `design_doc.md`; retire laser-as-authority after positive episode checks.
6. Optional: A2 OctoMap spike branch if A1 quality stalls.

### Non-goals for Goal A v1

- Full 3D frontier exploration.
- Replacing privileged pose with visual odometry in sim.
- Shipping RTAB-Map as the default sim mapper.

---

## Goal B — Multi-run ablation / comparison framework

**Status:** Schema and orchestration defaults selected (2026-08-26) — implement after Goal A (or after mapping is “good enough” for fair planner comparisons).  
**Priority:** P2.

### Motivation

The project end-state is **ablation studies**: many algorithms × ~**50** runs each × multiple scenes, running unattended for hours/days, with automatic metric collection, storage, and aggregation — press start, walk away, read tables/plots.

Today we only have interactive single episodes and **live** coverage topics.

### Thesis-aligned metrics (Aarush Aitha, Ch. 4)

Reference: local thesis PDF (`aarush_thesis.pdf`). Metrics to support:

| Metric | Thesis use | Our harness |
|--------|------------|-------------|
| **Final exploration %** | Privileged Habitat reveal (90° or 360°, ~5 m radius), **decoupled from perception** | Primary completeness metric; do **not** use perception `/grid_map` area for paper tables |
| **Total distance traveled** | Path length at end of run | From odom / trajectory log |
| **Consistency** | Mean ± std over repeats (thesis: 3; we: ~50) | Aggregate SQL/views |
| **Coverage vs distance** | Efficiency curves over the episode | Time series samples |
| **Path revisit histogram** | Looping / inefficiency | Grid cell visit counts along trajectory |
| **Trajectory overlays** | Qualitative figures | Optional PNG artifacts per run |
| Baselines (context) | Greedy, OpenCV+NBV, TARE, DSVP | Algorithm plug-ins / trajectory replay later |

**Important:** Live `coverage_metrics_node` (mapped/GT via IPC) is useful for dashboards, but **paper evaluation coverage** should follow Aarush’s privileged “perfect sight” reveal (or an equivalent documented GT policy), separate from Goal A’s perception map.

### Selected architecture (working defaults)

```text
experiment.yaml  →  batch orchestrator
                        ↓
              for each (algo, scene, seed) in matrix:
                  start_sim → wait complete/timeout → stop/cleanup
                        ↓
              per-run row → SQLite (+ optional JSONL mirror)
                        ↓
              aggregate CLI → mean/std tables, curves, revisit plots
```

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Orchestration | **Series first** (one `habitat3-sim`) | Matches one-GPU/desktop reality; parallel only if multi-container later |
| Lifecycle | Reuse `start_sim.sh` / `stop_sim.sh` + `cleanup_episode.sh` | Already battle-tested for orphan processes |
| Completion | Wait on `exploration/status` (`exploration_complete`) + hard timeout | Topic already exists |
| Storage | **SQLite** primary (`experiments/results.sqlite`); optional JSONL export | Queryable mean/std; portable file |
| UI | **CLI harness v1**; Elytra “Start ablation” button later | Avoid blocking on product UI |
| Config | YAML matrix: `algorithms[]` × `scenes[]` × `seeds` / `n_runs` | Human-editable sweeps |
| Algorithm plug-in | Named profiles (params / launch args / policy knobs) | Swap explorers without rewriting orchestrator |

### Per-run record schema (v1)

```text
runs
  run_id            TEXT PK
  experiment_id     TEXT
  algorithm_id      TEXT
  scene_id          TEXT
  seed              INTEGER
  started_at        TEXT (ISO)
  finished_at       TEXT
  status            TEXT  -- completed | timeout | error
  error_message     TEXT NULL
  final_coverage    REAL  -- privileged exploration %
  distance_m        REAL
  duration_s        REAL
  config_json       TEXT  -- frozen knobs for reproducibility
  artifact_dir      TEXT NULL

coverage_samples
  run_id, t_s, distance_m, coverage  -- curve points

revisit_bins
  run_id, revisit_count, cell_count  -- histogram
```

### Experiment config sketch

```yaml
experiment_id: vlm_vs_greedy_mp3d_2026q3
n_runs_per_cell: 50
timeout_s: 7200
eval:
  fov_deg: 360          # or 90 — match thesis FOV modes
  reveal_radius_m: 5.0  # privileged coverage
algorithms:
  - id: vlm_dfs
    profile: exploration_policy_vlm_default
  - id: greedy_nearest
    profile: exploration_policy_greedy
scenes:
  - JmbYfDe2QKZ
  # … Matterport IDs
seeds:
  mode: sequential      # or list
  start: 0
```

### Building blocks already in-repo

- Episode start/stop scripts and Elytra action hooks.
- `exploration/status` / `exploration_complete`.
- Habitat IPC `get_coverage_stats` (dashboard); extend or add privileged-reveal eval for thesis parity.
- Scene + exploration-policy knobs via Elytra / env.

### Gaps to build (when Goal B starts)

1. Batch orchestrator (matrix expand → loop → cleanup).
2. Waiter + timeout + failure classification.
3. SQLite writer + artifact directory layout (`experiments/<id>/<run_id>/`).
4. Privileged coverage evaluator aligned with thesis (FOV × radius).
5. Path-revisit histogram from logged poses.
6. Aggregate report script (tables + coverage-vs-distance plots).
7. Optional: Elytra button that shells the same CLI.

### Non-goals for Goal B v1

- Concurrent multi-project Elytra sessions / cloud multi-user.
- Running TARE/DSVP inside Habitat (thesis used AEDE trajectory replay — defer).
- Real-robot ablation matrix.

### Implementation sketch (when we start — not now)

1. Persist one completed episode to SQLite by hand (schema + single-run exporter).
2. Orchestrate N=2 smoke matrix (2 algos × 1 scene × 2 seeds).
3. Scale to N≈50; add aggregation CLI.
4. Add revisit + privileged FOV modes; then paper-oriented reports.

---

## Suggested sequencing

| Step | Work | Gate |
|------|------|------|
| 1 | Discuss / lock Goal A params (height band, `/scan` fate) | Done enough to implement |
| 2 | Implement Goal A (PC → `/grid_map`) behind feature flag | Episode map quality ≥ laser baseline |
| 3 | Lock Goal B eval definition (privileged FOV/radius) | Matches thesis tables we care about |
| 4 | Implement Goal B v1 (SQLite + series orchestrator) | Smoke matrix green |
| 5 | Scale ablations; optional Elytra entrypoint | 50-run cells unattended |

---

## Discussion log

### 2026-08-26 — Initial capture + approach selection

**Context:** Research advisor agreed depth→laser strip is the wrong primary pipeline. Need a future-goals doc and deep discussion before coding. Ablation harness required for ~50 runs/algorithm with thesis-like metrics.

**Goal A decision:** Proceed with **A1** (`depth_image_proc` + known-pose point-cloud mapper with free-space ray carving). Keep A2 (OctoMap) as a spike fallback. Defer A3 (RTAB-Map) to sim2real. A4/A5 are complementary or out of scope for v1.

**Goal B decision:** Series orchestrator + SQLite + YAML matrix; thesis metrics (final %, distance, mean±std, coverage-vs-distance, revisit); privileged eval decoupled from perception map; CLI before Elytra UI.

**Still soft (tune at kickoff, not blockers for doc):** exact wall height band numbers; whether Nav2 keeps `/scan` vs PointCloud2 sources; exact privileged reveal FOV default (90 vs 360) for the first ablation campaign.

**Next:** When ready to implement, open a dedicated session for Goal A only; append parameter lock notes here before coding.

### 2026-08-26/27 — Goal A implemented + Nav2 fail-fast; open cascade exhaustion

**Locked for Goal A v1:** wall band **0.05–1.0 m**; FREE does not overwrite OCCUPIED; `use_pc_mapper` default true.

**Nav2:** no-recovery BT + tolerance 1.0 m. Side effect: rapid multi-frontier exhaustion from a bad pose — next session: **return-to-parent before selecting another frontier** (see JOURNAL closeout).

---