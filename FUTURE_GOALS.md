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

## Current baseline (as of 2026-09-15)

```text
/depth_data + /camera_info + /odom
        → known_pose_pc_mapper (C++, default)  →  /grid_map
        → explore_node (thin orchestrator)
              → ExplorationBrain plugin (brain_id)
              → detect frontiers (inset into free) → brain → optional VLM → selectNextGoal
              → 360° scan only on first visit to a frontier (visited ≠ dead)
        → Nav2 NavigateToPose  →  /cmd_vel  →  DiscreteMove (Habitat)
        → on NEW-goal nav fail: nav_fail_policy
              inaccessible (prior OK + start clear + definitive NO_VALID_PATH/…) → mark dead
              else stuck → thrash → zero-inflation return → retry once;
              still wedged → termination_reason=stuck (do not burn remaining tree)
```

| Piece | Location | Notes |
|-------|----------|--------|
| Mapping | C++ `known_pose_pc_mapper` (`use_pc_mapper:=true`) | Subsample 8; pose-gated integrate; Bresenham early-stop on OCCUPIED; grid inflate 0.05 m (live-tunable) |
| Exploration | `explore_node` + `ExplorationBrain` | Pluggable brains via `brain_id`; detect/Nav2/recovery stay in node |
| Brains | `exploration_brain.hpp/.cpp` | `vlm_tree_dfs`, `greedy_nearest`, `vlm_frontier_graph`, `vlm_choice_dijkstra` |
| Visited vs dead | Tree + graph brains | `visited` = arrived (one-time 360°); `dead` / `fully_explored` = abandoned |
| Motion | `cmd_vel_to_discrete` | `turn_over_drive_ratio` default **0.5** + `drive_max_angular` **0.12** (corner arcs TURN, not 0.25 m FORWARD) |
| Nav2 | `nav2_params.yaml` | No-recovery BT; `allow_unknown: true`; costmap inflation **0.22 m** (≥ `robot_radius` 0.18) |
| Nav fail | `nav_fail_policy` | Inaccessible vs stuck; honest `termination_reason` in status / metrics |
| Ablations | `experiments/` + Elytra | Brain checkboxes; Fresh → `ablation_run_<ts>/`; cells `{algo}_seedN/` + `run_info.json` |
| Event JSONL | `experiment_event_logger.py` | status, tree, vlm/scores, **brain/decision**, **brain/graph_edges**, **vlm/choice** |

### Known serious issues (not resolved)

Ablation metrics remain **unreliable for claiming navigation quality**. Treat high coverage as suspicious until path follow + recovery are re-verified end-to-end.

1. **DiscreteMove path follow still imperfect.** Nav2 plans around corners; RPP→`cmd_vel`→0.25 m / 10° quantization still wedges the robot into walls. Ratio/heading gate (0.5 / 0.12) helps mild arcs; residual plow/thrash remains in smokes (`termination_reason=stuck` ~50–77% cov).
2. **“Advanced” recovery can look worse than naive mark-dead.** Ending `stuck` after one failed return avoids mass-blacklist lies, but also stops episodes that older thrash-forever runs sometimes recovered into higher coverage. Persistence ≠ correctness.
3. **Fake early “success” hazard.** Prior classifier bugs (prior still plannable while wedged; soft-fail score 0; apply-profile abort) produced `success` / `no live frontiers` in seconds at ~52% with almost no visits. Prefer `termination_reason`, visited counts, and trajectory media over status alone.
4. **Costmap / start clearance sensitivity.** Inflation vs `robot_radius`, frontier goals on free↔unknown edges, and Habitat `collided=True` during DiscreteMove interact; wedged starts make every NEW goal look like `NO_VALID_PATH`.
5. **Goal C debt unchanged:** graph edges are Euclidean kNN, not Nav2 path cost; choice brain is score-argmax stand-in, not true multi-image VLM choice.

**Open priority:** discrete lattice nav is now the default (`discrete-lattice-nav` branch). Re-verify corner follow visually; only then revisit recovery aggressiveness.

---

## Completed

| Goal | Shipped | Pointers |
|------|---------|----------|
| **A — PC → 2D occupancy** | 2026-08-26 … 2026-09-06 | C++ mapper, wall band 0.05–1.0 m |
| **B — Ablation harness v1** | 2026-08-31 | YAML matrix, SQLite, aggregate tables, Elytra ablation buttons |
| **Nav robustness (partial)** | 2026-09-06 | Return-home; stuck policy; wall-unstick; cmd_vel ratio; inflation 0.15 m |
| **B2 — Run packages + resume** | 2026-09-06 | Artifact package, media/events, `render_run.py`, WAL resume |
| **B2.1 — Campaign ops + thrash recovery** | 2026-09-07 | Fresh vs resume UI; per-run scratch; interrupt cleanup; tmux retries; return-home abandon; thrash BACK/FWD×growing (max 5) |
| **C — Swappable brains (code path)** | 2026-09-08 | Interface + 4 brains + thin explore_node + ablation `brain` wiring + event logging; **smoke campaign not yet run** |
| **C polish — ops / scan / recovery** | 2026-09-08 | Brain enable checkboxes; visited≠dead scan gate; zero-inflation last-ditch; `ablation_run_<ts>/{algo}_seedN` naming |

---

## Open goals (primary next steps)

### C — Swappable exploration brains — *smoke gate remaining*

**Shipped 2026-09-08 (implementation + ops polish).** Remaining exit criterion is the live campaign.

**Architecture (decision recorded 2026-09-07; implemented 2026-09-08)**

```text
shared stack:
  frontier detection, mapping, Nav2, DiscreteMove, thrash + zero-inflation recovery, packaging

swappable brain:
  ExplorationBrain interface in exploration_brain.hpp/.cpp
  ablation YAML → algorithms[].brain → /data/selected_brain.id → launch brain_id:=
  Elytra checkboxes filter algorithms[] before matrix expand
```

| Shared (orchestration) | Owned by each brain |
|------------------------|---------------------|
| Occupancy `/grid_map`, frontier *detection* geometry | Frontier *memory* (none / tree / graph) |
| Nav2 + thrash + zero-inflation recovery | Goal selection policy |
| Episode lifecycle, metrics, media, event publish hooks | Whether/when to call VLM |
| Ablation matrix wiring + short package names | Visited / dead / backtrack semantics |

**Algorithms**

| ID | Status | Notes |
|----|--------|-------|
| `vlm_tree_dfs` | Shipped | Tree DFS + numeric VLM (alias `vlm_dfs`) |
| `greedy_nearest` | Shipped | True Euclidean nearest; **no tree, no VLM** |
| `vlm_frontier_graph` | Shipped (v1) | kNN graph + numeric VLM; edges logged (Euclidean cost proxy) |
| `vlm_choice_dijkstra` | Shipped (v1) | Dijkstra neighborhood + choice JSONL; score-argmax stand-in for multi-image choice |

**Refactor steps**

1. ✓ `ExplorationBrain` API + factory  
2. ✓ Lift tree+VLM DFS → `vlm_tree_dfs`  
3. ✓ True `greedy_nearest`  
4. ✓ Thin `explore_node` orchestration  
5. ✓ `vlm_frontier_graph` (+ edge logging)  
6. ✓ `vlm_choice_dijkstra` (+ choice logging)  
7. ✓ Ablation `algorithms[].brain` + 4-cell smoke YAML  
8. ✓ Brain enable checkboxes + short package naming + visited≠dead scan + zero-inflation recovery  
9. ☐ Live smoke: 1 scene × 2 seeds × selected brains under resume/fresh  

**Exit criteria**

- [x] Matrix can run ≥2 brains without hand-knob recompiles  
- [x] `greedy_nearest` never builds a tree / never blocks on VLM  
- [x] Decision + visited in event JSONL (`exploration/brain/decision`)  
- [x] Graph brains emit edge costs (`exploration/brain/graph_edges`) — Euclidean v1  
- [x] Choice brain emits prompt/response + selected id (`exploration/vlm/choice`) — score-argmax v1  
- [x] Operator can disable brains in Elytra before Start Ablation  
- [x] Packages use `ablation_run_<ts>/{algo}_seedN` + `run_info.json`  
- [ ] Smoke: 1 scene × 2 seeds × 4 brains completes under resume/fresh  

**Follow-ups after smoke (still Goal C polish, not new goals)**

- Replace Euclidean kNN edge costs with Nav2 `ComputePathToPose` length  
- True multi-image VLM *choice* query (not score-argmax)  
- Confirm event JSONL fields in packaged runs  

**Non-goals for C**

- Hot-swapping brains mid-episode  
- Learning / RL policies  
- Changing mapper or Nav2 stack as part of the brain  

**Key paths**

- Brains: `ros_workspace/src/explorer_mission/include|src/.../exploration_brain.*`  
- Node: `explore_node.cpp` (`brain_id`, publish decision/edges/choice)  
- Ablation: `experiments/profiles.py`, `configs/smoke.yaml`, `orchestrator.py`, `package.py`  
- Launch: `start_sim.sh` reads `/data/selected_brain.id`  
- Events: `sim/scripts/experiment_event_logger.py`, msgs `BrainDecisionEvent` / `BrainGraphEdges` / `VlmChoiceEvent`  
- Tests: `test_exploration_brain.cpp`, `experiments/tests/test_config.py` + `test_event_log_shape.py`  

**Package layout (Fresh campaign)**

```text
sim/data/experiments/ablation_run_<YYYYMMDD_HHMMSS>/
  campaign_info.json
  greedy_nearest_seed0/
    run_info.json    # scene, brain, env, eval
    manifest.json
    metrics/ media/ logs/
  vlm_dfs_seed0/
  …
```

---

### 1 — Paper-ready evaluation *(co-top; unblocked once C smoke is green)*

| Gap | Notes |
|-----|--------|
| Privileged FOV 90° | Radius wired; FOV mode still open |
| Scale to ~50 seeds / cell | Smoke packages + resume/fresh proven; next is matrix scale |
| Thesis-aligned matrix | Four brains exist; need green smoke then scale |

**Eval rule:** Paper coverage stays decoupled from perception `/grid_map` (privileged Habitat reveal).

**Package reminder (shipped):** Fresh → `sim/data/experiments/ablation_run_<ts>/{algo}_seedN/` with `run_info.json`, `manifest.json`, metrics CSVs, side-by-side 10× MP4, event JSONL; viz via `python experiments/render_run.py <run_dir>/`. Elytra: brain checkboxes + **Resume incomplete** or **Fresh campaign**.

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
| **1** ✓ | Artifact package + timelapse + event logs + `render_run.py` | One smoke run package ≤50 MB |
| **2** ✓ | Crash-safe resume + progress UI | Kill mid-batch; only unfinished cells run |
| **2.1** ✓ | Fresh campaign UI + thrash unstick + tmux/return-home hardening | Reliable smoke start |
| **3** ✓ | Swappable brains code (Goal C) — 4 brains + wiring + logging | Unit tests green; smoke YAML ready |
| **3.0.1** ✓ | Brain checkboxes, visited≠dead scans, zero-inflation, short names | Ops polish before smoke |
| **3.1** | **Live multi-brain smoke** | Selected brains × seeds complete under Fresh/Resume; events in packages |
| **4** | Nav2 edge costs + true multi-image VLM choice | Thesis-faithful graph/choice |
| **5** | FOV 90° + 50-seed paper matrix | Thesis-aligned campaign |
| **6** | Memory hardening | Multi-hour episode without OOM |
| **7** | Sim2real mapping | Hardware-ready |

---

## Discussion log

### 2026-09-08 — Goal C ops polish (evening)

1. **Ablation brain checkboxes** — enable/disable algorithms before Start Ablation (`--algorithms` → `filter_algorithms`).  
2. **Visited ≠ dead** — 360° only on unvisited frontiers; backtrack to visited-alive skips rescan.  
3. **Zero-inflation last-ditch** — after thrash ×5, retry NavigateToPose with Nav2 + mapper inflation 0; then mark dead.  
4. **Package naming** — `ablation_run_<timestamp>/{algorithm_id}_seed{N}/` + `run_info.json` / `campaign_info.json`.

### 2026-09-08 — Goal C implementation session (full day)

Implemented Goal C end-to-end in code; **awaiting live smoke**.

**What shipped**

1. **`ExplorationBrain` API + factory** — `createExplorationBrain(id, config)`; aliases `vlm_dfs` → `vlm_tree_dfs`.  
2. **Four brains** in `exploration_brain.cpp`: tree DFS+VLM; true greedy; frontier graph + numeric VLM; Dijkstra neighborhood + choice logging.  
3. **Thin `explore_node`** — detect → brain → optional VLM → navigate/recover; param `brain_id`.  
4. **Ablation wiring** — `algorithms[].brain`; orchestrator writes `/data/selected_brain.id`; `start_sim.sh` passes launch arg; legacy `exploration_policy_greedy` → true greedy.  
5. **Event JSONL** — ROS msgs + logger for `brain/decision` (incl. visited/live), `brain/graph_edges`, `vlm/choice`.  
6. **Smoke config** — `experiments/configs/smoke.yaml`: 4 brains × 1 scene × 2 seeds (8 runs); Fresh folders are `ablation_run_<ts>`.

**Honest v1 limits:** Euclidean (not Nav2) graph costs; choice is score-argmax with logged prompt/response (not multi-image single-query VLM yet).

**Next:** run Fresh smoke in Elytra; verify packages + event fields; then paper-eval scale / Nav2+true-choice polish.

### 2026-09-07 — Swappable brains become co-top priority

Today’s “greedy” ablation was a mislabeled tree+VLM variant. Architecture: pluggable `ExplorationBrain` with four implementations. Paper eval waits on honest algorithm cells. *(Implemented 2026-09-08.)*

### 2026-09-07 — B2.1 milestone closeout

Campaign isolation (resume vs fresh), package layout, interrupt encode/cleanup, resilient tmux start, return-home abandon, DiscreteMove thrash recovery.

### 2026-09-06 — Ablation packages + resume planned as next milestone

PART 1 (recording/viz) and PART 2 (crash-safe resume) before algorithm scale-up.

### 2026-09-06 — Mapping/nav arc closeout

C++ mapper, async coverage, curvature ratio, wall-unstick, inflation 0.15 m.

### 2026-08-31 — Goal B v1 shipped

CLI + SQLite + Elytra ablation entrypoints.
