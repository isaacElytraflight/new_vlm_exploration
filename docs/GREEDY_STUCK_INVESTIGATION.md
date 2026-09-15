# Greedy Brain Stuck Investigation

**Date:** 2026-09-15  
**Status:** Investigation phase — do NOT implement speculative fixes yet.  
**Problem:** Robot gets stuck in corners when using Greedy brain during ablations.

---

## 1. System Map: Stuck-Related Control Flow

### 1.1 Component Overview

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           EXPLORATION STACK                                  │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌──────────────────┐                    ┌──────────────────┐              │
│  │  ExploreNode     │  selectNextGoal()  │ GreedyNearestBrain│              │
│  │  (explore_node.cpp)───────────────────▶  (exploration_brain.cpp)         │
│  │                  │                    │                  │              │
│  │  Main loop:      │  BrainDecision     │  - live_ vector  │              │
│  │  - detect        │  ◀─────────────────│  - visited_ set  │              │
│  │  - select        │                    │  - dead_ set     │              │
│  │  - navigate      │                    └──────────────────┘              │
│  │  - arrive/fail   │                                                      │
│  └────────┬─────────┘                                                      │
│           │                                                                 │
│           │ navigateNewGoalWithRecovery()                                  │
│           ▼                                                                 │
│  ┌──────────────────┐    NavigateToPose    ┌──────────────────┐            │
│  │  Nav2Navigator   │────────────────────▶ │  Nav2 Stack      │            │
│  │  (nav2_navigator.cpp)                   │  - bt_navigator  │            │
│  │                  │  ComputePathToPose   │  - planner_server│            │
│  │  - navigateToPose│◀────────────────────│  - controller    │            │
│  │  - computePathExists                    │  - costmaps      │            │
│  │  - lastErrorCode │                      └──────────────────┘            │
│  └────────┬─────────┘                                                      │
│           │                                                                 │
│           │ on failure                                                      │
│           ▼                                                                 │
│  ┌──────────────────┐                    ┌──────────────────┐              │
│  │ nav_fail_policy  │  classifyNewGoalNavFailure            │              │
│  │ (nav_fail_policy.cpp)─────────────────▶ kInaccessible    │              │
│  │                  │                    │ OR kStuck        │              │
│  │ - isDefinitiveGoalUnreachable         └──────────────────┘              │
│  │ - isStartClearanceOk                                                    │
│  └────────┬─────────┘                                                      │
│           │                                                                 │
│           │ if kStuck                                                       │
│           ▼                                                                 │
│  ┌──────────────────┐                                                      │
│  │  wall_unstick    │  runWallUnstick() → DiscreteMove BACK/FWD            │
│  │  (wall_unstick.cpp)                                                     │
│  │                  │  unstickThrashMotion(escalation_level)               │
│  │ - clearanceToOccupiedM                                                  │
│  │ - decideNavFailureRecovery                                              │
│  └──────────────────┘                                                      │
│                                                                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Key Files and Functions

| File | Key Functions | Role |
|------|--------------|------|
| `ros_workspace/src/explorer_mission/src/explore_node.cpp` | `startExploration()`, `navigateNewGoalWithRecovery()`, `detectAndOfferToBrain()` | Main orchestration loop |
| `ros_workspace/src/explorer_mission/src/exploration_brain.cpp` | `GreedyNearestBrain::selectNextGoal()`, `onNavFailed()`, `onArrived()` | Greedy frontier selection (lines 479-626) |
| `ros_workspace/src/explorer_mission/src/nav2_navigator.cpp` | `navigateToPose()`, `computePathExists()` | Nav2 action client |
| `ros_workspace/src/explorer_mission/src/nav_fail_policy.cpp` | `classifyNewGoalNavFailure()`, `isDefinitiveGoalUnreachable()` | Failure classification |
| `ros_workspace/src/explorer_mission/src/wall_unstick.cpp` | `unstickThrashMotion()`, `clearanceToOccupiedM()` | Recovery motions |
| `ros_workspace/src/explorer_mission/src/frontier_detection.cpp` | `findFrontierContoursMasked()`, `insetFrontierGoalWorld()` | Frontier detection and goal placement |

### 1.3 GreedyNearestBrain Algorithm (Detailed)

```cpp
// exploration_brain.cpp lines 479-626
class GreedyNearestBrain:
  
  // State
  live_: vector<FrontierCandidate>  // current unvisited frontiers
  visited_: set<uint32_t>           // arrived + scanned
  dead_: set<uint32_t>              // marked inaccessible by Nav2 failure
  poses_: map<id, Point2f>          // stored positions for viz
  
  // On frontier detection (called each time robot does 360° scan):
  onFrontiersDetected(frontiers):
    live_.clear()  // REPLACES all live frontiers with new detection
    for f in frontiers:
      if f.id not in visited_ and f.id not in dead_:
        live_.push_back(f)
        poses_[f.id] = f.position
    return accepted_ids
  
  // Goal selection:
  selectNextGoal(ctx):
    if live_.empty():
      return BrainAction::kComplete  // <-- TERMINATION TRIGGER
    
    best = argmin(euclidean_distance(f.position, ctx.robot_pose) for f in live_)
    return BrainAction::kNavigateTo, best.id, best.position
  
  // On successful arrival:
  onArrived(goal_id, pose):
    visited_.insert(goal_id)
    eraseLive(goal_id)
  
  // On navigation failure:
  onNavFailed(goal_id, mark_dead):
    if mark_dead:
      dead_.insert(goal_id)
      eraseLive(goal_id)
```

### 1.4 Navigation Failure Classification (Critical Path)

```cpp
// explore_node.cpp lines 937-1064: navigateNewGoalWithRecovery()

NavAttemptResult navigateNewGoalWithRecovery(goal_pos, look_at, prior_id):
  
  // 1. First navigation attempt
  if navigateToPosition(goal_pos):
    return {ok: true}
  
  // 2. Gather diagnostic info
  nav_error_code = nav2_navigator_->lastErrorCode()
  prior_plan_ok = theoreticalPlanTo(prior_pose)  // Can we return to where we came from?
  new_goal_plan_ok = computePathExists(goal_pos) // Can we plan to the goal?
  new_plan_code = nav2_navigator_->lastErrorCode()
  start_clearance = currentClearanceM()          // Distance to nearest obstacle
  
  // 3. Classify failure
  // nav_fail_policy.cpp lines 38-50
  if prior_known && prior_plan_ok && start_clearance_ok &&
     !new_goal_plan_ok && new_goal_unreachability_definitive:
    // Definitive codes: GOAL_OUTSIDE_MAP(204), GOAL_OCCUPIED(206), NO_VALID_PATH(208)
    return kInaccessible  // Mark frontier dead, continue exploration
  else:
    return kStuck  // Need recovery
  
  // 4. If kStuck: thrash recovery
  for i in 0..unstick_max_attempts:
    runWallUnstick(i)  // BACKWARD/FORWARD alternating
  
  // 5. Deflate costmap + return to prior
  applyCostmapInflation(0.0, 0.0)
  back_ok = navigateToPosition(prior_pose)
  // ... more recovery attempts if needed
  applyCostmapInflation(nav2_inflation_radius_m_, mapper_inflation_m_)
  
  // 6. Retry goal after recovery
  if navigateToPosition(goal_pos):
    return {ok: true}
  
  // 7. Final classification
  if !start_clearance_ok_after:
    // Still wedged → terminate episode as stuck (don't burn all frontiers)
    return {terminate_stuck: true}
  else:
    // Start is clear → this frontier is truly inaccessible
    return {mark_frontier_dead: true}
```

### 1.5 Termination Conditions

The exploration loop terminates when:

1. **Brain returns kComplete**: `live_.empty()` in Greedy (no more frontiers)
2. **Stuck recovery fails**: `terminate_stuck=true` (cannot return to prior node)
3. **ROS shutdown / timeout**: External intervention

```cpp
// explore_node.cpp line 276-280
if (decision.action == explorer_mission::BrainAction::kComplete) {
  publishPhase("complete", ..., terminationReasonCStr(TerminationReason::kSuccess));
  break;
}

// explore_node.cpp lines 309-316
if (nav.terminate_stuck) {
  publishPhase("complete", ..., terminationReasonCStr(TerminationReason::kStuck));
  break;
}
```

### 1.6 Grid Map and Costmap Flow

```
Habitat depth sensor
       │
       ▼
known_pose_pc_mapper_node (explorer_bridge)
  - /depth_data → /grid_map (OccupancyGrid)
  - obstacle_inflation_m = 0.05m
       │
       ▼
Nav2 global_costmap
  - static_layer: subscribes to /grid_map
  - obstacle_layer: /scan (LaserScan from depth)
  - inflation_layer: inflation_radius = 0.22m (robot_radius = 0.18m)
       │
       ▼
Nav2 planner_server (NavFn)
  - allow_unknown: true
  - tolerance: 1.0m
```

---

## 2. Labeled Hypothesis (Preliminary — May Be Discarded)

Based on code analysis, the stuck-in-corners failure may involve:

### H1: Greedy Frontier Placement in Corners (OUR LOGIC)
- `insetFrontierGoalWorld()` pulls goals 0.35m into free space from the free↔unknown edge
- In tight corners, inset may still land the goal near walls
- NavFn plans but RPP controller cannot follow tight arc → controller failure → blacklist

### H2: Costmap Inflation vs Corridor Width (NAV2 / GRID)
- `inflation_radius=0.22m` creates lethal zone around walls
- In narrow passages or corners, the entire corridor may be inflated
- Start becomes "occupied" → NO_VALID_PATH for ALL goals → mass blacklist if clearance check is borderline

### H3: Greedy's "All Frontiers Replaced" on Each Scan (OUR LOGIC)
- `onFrontiersDetected()` calls `live_.clear()` — complete replacement
- If detection misses a frontier (corner occlusion), it's gone from the live set
- Combined with aggressive dead-marking → premature "no live frontiers" termination

### H4: Start Clearance Check Borderline (NAV2 / OUR LOGIC)
- `unstick_min_clearance_m_=0.25m` threshold
- If robot is at 0.24m clearance, classified as "wedged" → kStuck
- But recovery thrash may not help if it's a corner geometry problem, not a temporary wedge

### H5: Prior Plan Always OK (False Positive) (OUR LOGIC)
- `theoreticalPlanTo(prior_pose)` may return true via `nearPose()` check
- If robot is still near prior, we skip the actual ComputePathToPose
- This can make "prior OK" always true, triggering inaccessible classification incorrectly

---

## 3. Reproduction Plan: Greedy Ablation Runs

### 3.1 Target Scene: Matterport3D `JmbYfDe2QKZ`

The target world is the **Matterport3D house `JmbYfDe2QKZ`** — that's the "long funny alphanumeric string" scene name.

| Property | Value |
|----------|-------|
| **Scene ID** | `JmbYfDe2QKZ` |
| **Container path** | `/data/scene_datasets/mp3d/JmbYfDe2QKZ/JmbYfDe2QKZ.glb` |
| **Host path** | `habitat3-exploration/sim/data/scene_datasets/mp3d/JmbYfDe2QKZ/` |
| **Config key** | `scenes: [JmbYfDe2QKZ]` in experiment YAML |
| **Elytra selector** | Scene dropdown → "Matterport JmbYfDe2QKZ" |

**Prerequisites:** The scene must be downloaded separately (Matterport ToS):
```bash
# On the researcher's machine, if not already present:
python habitat3-exploration/sim/scripts/download_mp3d_habitat_scene.py \
  --i-agree-to-mp-tos --scene JmbYfDe2QKZ

# Verify:
ls habitat3-exploration/sim/data/scene_datasets/mp3d/JmbYfDe2QKZ/JmbYfDe2QKZ.glb
```

### 3.2 Experiment Config

**File:** `experiments/configs/greedy_stress.yaml` (already created in this PR)

```yaml
experiment_id: greedy_stress_test
n_runs_per_cell: 5  # Multiple seeds for statistical significance
timeout_s: 1200     # 20 min per episode (enough to get stuck or succeed)
artifact_root: sim/data/experiments

eval:
  fov_deg: 360
  reveal_radius_m: 5.0

algorithms:
  - id: greedy_nearest
    brain: greedy_nearest

# Matterport3D scene: JmbYfDe2QKZ
# Resolves to: /data/scene_datasets/mp3d/JmbYfDe2QKZ/JmbYfDe2QKZ.glb
scenes:
  - JmbYfDe2QKZ

seeds:
  mode: sequential
  start: 0
```

### 3.3 Launch Commands (On Researcher's Machine)

**Option A: Via Elytra UI (Recommended)**

```bash
# 1. Start Elytra bridge
cd new_vlm_exploration/elytra-bridge/application
cp backend/.env.example backend/.env  # first time only
npm run dev
# Frontend: http://localhost:5173
# Backend: http://localhost:8787

# 2. Start Habitat sim container
cd new_vlm_exploration/habitat3-exploration/sim/docker
docker compose --env-file ../.env up -d sim
# noVNC: http://localhost:6080

# 3. In Elytra UI:
#    a. Select project: habitat3-exploration
#    b. Mode: sim
#    c. Click "Connect" (backend runs docker compose build/up)
#    d. Scene dropdown → "Matterport JmbYfDe2QKZ" (should be default)
#    e. Load experiment config: experiments/configs/greedy_stress.yaml
#    f. Click "Run Ablation"
```

**Option B: Via CLI Orchestrator**

```bash
cd new_vlm_exploration/habitat3-exploration

# Ensure container is running
cd sim/docker && docker compose --env-file ../.env up -d sim && cd ../..

# Run ablation
python -m experiments.orchestrator \
  --config experiments/configs/greedy_stress.yaml \
  --fresh
```

**Option C: Single Manual Episode (for debugging)**

```bash
# Inside container, set brain and scene:
docker exec habitat3-sim bash -lc '
  echo "greedy_nearest" > /data/selected_brain.id
  echo "/data/scene_datasets/mp3d/JmbYfDe2QKZ/JmbYfDe2QKZ.glb" > /data/selected_scene.path
'

# Start episode via tmux (from Elytra or manually):
docker exec habitat3-sim bash -lc '
  tmux kill-session -t habitat 2>/dev/null || true
  tmux new-session -d -s habitat "bash /workspace/scripts/start_sim.sh"
'

# Watch exploration status:
docker exec -it habitat3-sim bash -lc '
  source /opt/ros/jazzy/setup.bash
  source /opt/explorer_workspace/ros_workspace/install/setup.bash
  ros2 topic echo /exploration/status
'
```

### 3.4 Scene Selection Flow

The scene reaches Habitat through this chain:

```
Experiment YAML           Elytra UI dropdown         Environment variable
scenes: [JmbYfDe2QKZ] OR  "Matterport JmbYfDe2QKZ" OR HABITAT_SCENE=...
        │                         │                         │
        ▼                         ▼                         │
orchestrator._prepare_scene()     PUT /sim/habitat-scene    │
        │                         │                         │
        ▼                         ▼                         │
/data/selected_scene.path ◀───────┘                         │
        │                                                   │
        ▼                                                   │
start_sim.sh reads selected_scene.path ─────────────────────┘
        │
        ▼
export HABITAT_SCENE="/data/scene_datasets/mp3d/JmbYfDe2QKZ/JmbYfDe2QKZ.glb"
        │
        ▼
habitat_engine.py loads scene
```

### 3.5 Success/Fail Criteria

| Outcome | Criteria |
|---------|----------|
| **Success** | `final_coverage >= 0.89` AND `termination_reason=success` |
| **Stuck (Early Term)** | `termination_reason=stuck` at any coverage |
| **Pseudo-Success** | `termination_reason=success` but `final_coverage < 0.80` (false completion) |
| **Timeout** | `termination_reason=timeout` (episode hit 20min limit) |

### 3.6 Metrics to Record Per Run

Already collected by `experiment_collect.py` and `experiment_event_logger.py`:
- `final_coverage` (float 0-1)
- `distance_m` (total odometry)
- `duration_s`
- `termination_reason` (success/stuck/timeout/interrupted)
- `trajectory` (list of (x, y) poses)
- `coverage_samples` (time series of coverage %)
- `events.jsonl.gz`: exploration/status, brain/decision, frontier_tree

---

## 4. Instrumentation Plan

### 4.1 What to Record in OUR Pipeline (Low-Risk Additions)

These additions are **read-only / log-only** — they don't change behavior.

#### A. Enhanced Brain Decision Logging

**File:** `explore_node.cpp` in `publishBrainDecision()`

Add to `BrainDecisionEvent.msg` and logging:
- `robot_pose_x`, `robot_pose_y` (current robot position)
- `num_live_frontiers` (count of live_ before selection)
- `goal_distance_m` (Euclidean to selected goal)
- `nearest_frontier_id` (for verification it matches goal_id in greedy)

**Current state:** Already logs `visited_ids` and `live_ids`.

#### B. Frontier Detection Telemetry

**File:** `explore_node.cpp` in `detectAndOfferToBrain()`

Log (to ROS logger or diagnostic topic):
- Number of contours before/after filtering
- Number of contours rejected by dedupe
- Number of frontiers already in visited_/dead_ (rejected by brain)
- Inset distance for each frontier goal (was 0.35m applied? How much did it move?)

#### C. Nav Failure Diagnostics

**File:** `explore_node.cpp` in `navigateNewGoalWithRecovery()` (line ~983-992)

Already logs extensively. Ensure these are in events.jsonl:
- `nav_dt_s` (how long NavigateToPose ran)
- `nav_error_code` (from NavigateToPose)
- `new_plan_code` (from ComputePathToPose)
- `prior_plan_ok` (boolean)
- `start_clearance_m` (actual value, not just ok/not ok)
- `fail_class` (kInaccessible or kStuck)

#### D. Costmap State Snapshot (New)

Create optional `/exploration/debug/costmap_at_failure` topic (publish only on nav failure):
- `goal_x`, `goal_y`
- `goal_cost` (costmap value at goal cell)
- `start_cost` (costmap value at robot position)
- `path_length` (if ComputePath succeeded, how long was it?)
- `costmap_timestamp`

Implementation: Read from `/global_costmap/costmap` topic, sample cells.

### 4.2 Nav2-Side Instrumentation (External — Document Only)

These require no code changes, just log configuration:

#### A. BT Navigator Verbose Logging

Set log level for bt_navigator:
```bash
ros2 run rclcpp_py2_utils set_log_level /bt_navigator debug
```

Or in launch: add `--ros-args --log-level bt_navigator:=debug`

#### B. Planner Server Logging

Already logs plan failures with error codes. Can enable verbose:
```bash
ros2 run rclcpp_py2_utils set_log_level /planner_server debug
```

This will show:
- NavFn cell expansion details
- Whether `allow_unknown` paths are being used
- Tolerance calculations

#### C. Costmap Layer Debugging

Enable costmap debug topics in `nav2_params.yaml`:
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      always_send_full_costmap: true  # For debugging only
```

Topics to watch:
- `/global_costmap/costmap` (full costmap)
- `/global_costmap/costmap_updates` (incremental updates)
- `/local_costmap/costmap` (rolling window around robot)

#### D. Controller Server Logging

```bash
ros2 run rclcpp_py2_utils set_log_level /controller_server debug
```

Will show:
- RPP lookahead calculations
- Goal checker status
- Why controller aborted (collision, timeout, cannot follow path)

### 4.3 Data Collection Checklist for Ablation Runs

For each stuck episode, collect:

- [ ] `run_metrics.json` — final stats
- [ ] `events.jsonl.gz` — full event log
- [ ] `logs/media_recorder.log` — any recorder errors
- [ ] `logs/collector.log` — experiment_collect output
- [ ] `media/final_grid_map.png` — map at episode end
- [ ] `media/final_nav_plan.png` — last Nav2 plan overlay
- [ ] `media/map_timelapse_10x.mp4` — exploration progress video

From container (if available):
- [ ] ROS bag of `/global_costmap/costmap` during failure window
- [ ] Screenshot of RViz with costmap + frontier overlay at failure time

---

## 5. Questions / Blockers Requiring Local Environment

The following cannot be answered from code alone and require running the actual Elytra + Habitat stack with the **Matterport JmbYfDe2QKZ** scene:

1. **Which seeds trigger stuck on JmbYfDe2QKZ?**
   - Need to run `greedy_stress.yaml` ablation (5 seeds) and analyze results
   - Identify reproducible spawn positions that lead to corner traps

2. **What is the typical clearance value when stuck occurs?**
   - Need to capture `start_clearance_m` from live runs
   - Compare threshold (0.25m) vs actual clearances at failure points

3. **Are frontiers being placed in actually-navigable locations?**
   - Need to overlay inset frontier positions on costmap
   - Check if 0.35m inset is sufficient in JmbYfDe2QKZ corridors

4. **Does the costmap show the corner as occupied or free?**
   - Need to snapshot costmap at failure time
   - Compare Nav2 costmap vs Habitat ground-truth geometry

5. **Is RPP failing to follow a valid plan, or is the plan itself bad?**
   - Need to capture controller logs during failure
   - Determine if NavFn plans through JmbYfDe2QKZ corridors are followable

6. **Does inflation=0 recovery help in corners?**
   - Already implemented but need to measure success rate on JmbYfDe2QKZ
   - May need to adjust recovery strategy for this specific scene geometry

7. **What are the corridor widths in JmbYfDe2QKZ?**
   - Measure actual corridor widths vs inflation diameter (2×0.22m = 0.44m)
   - Identify choke points where inflation could block all paths

---

## 6. Safe Scaffolding (Optional — Docs/Debug Hooks Only)

If approved, these changes are strictly non-behavior-changing:

### 6.1 Documentation Files
- [x] This investigation document (`docs/GREEDY_STUCK_INVESTIGATION.md`)
- [ ] Add `greedy_stress.yaml` experiment config for reproducible ablations

### 6.2 Debug Logging (Optional PR)
Add ROS_INFO logs without changing control flow:
- Log frontier inset distance in `insetFrontierGoalWorld()`
- Log costmap cell values at goal position in `navigateNewGoalWithRecovery()`
- Log number of frontiers replaced vs retained in `onFrontiersDetected()`

These would be guarded by `publish_debug_topics_` parameter.

---

## Appendix A: Code References

### GreedyNearestBrain::selectNextGoal

```cpp
// exploration_brain.cpp lines 581-610
BrainDecision selectNextGoal(const BrainContext & ctx) override
{
  BrainDecision d;
  if (live_.empty()) {
    d.action = BrainAction::kComplete;
    d.detail = "no live frontiers";
    return d;
  }
  float best_dist2 = std::numeric_limits<float>::infinity();
  const FrontierCandidate * best = nullptr;
  for (const auto & f : live_) {
    const float dx = f.position.x - ctx.robot_pose.x;
    const float dy = f.position.y - ctx.robot_pose.y;
    const float dist2 = dx * dx + dy * dy;
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      best = &f;
    }
  }
  if (!best) {
    d.action = BrainAction::kComplete;
    d.detail = "no live frontiers";
    return d;
  }
  d.action = BrainAction::kNavigateTo;
  d.goal_id = best->id;
  d.goal = best->position;
  d.detail = "greedy nearest euclidean";
  return d;
}
```

### classifyNewGoalNavFailure

```cpp
// nav_fail_policy.cpp lines 38-50
NewGoalFailClass classifyNewGoalNavFailure(
  bool prior_pose_known,
  bool prior_plan_ok,
  bool new_goal_plan_ok,
  bool new_goal_unreachability_definitive,
  bool start_clearance_ok)
{
  if (prior_pose_known && prior_plan_ok && start_clearance_ok &&
    !new_goal_plan_ok && new_goal_unreachability_definitive)
  {
    return NewGoalFailClass::kInaccessible;
  }
  return NewGoalFailClass::kStuck;
}
```

### isDefinitiveGoalUnreachable

```cpp
// nav_fail_policy.cpp lines 8-17
bool isDefinitiveGoalUnreachable(uint16_t compute_path_error_code)
{
  switch (compute_path_error_code) {
    case kComputePathGoalOutsideMap:   // 204
    case kComputePathGoalOccupied:     // 206
    case kComputePathNoValidPath:      // 208
      return true;
    default:
      return false;
  }
}
```

---

## Appendix B: Matterport JmbYfDe2QKZ Scene Details

The target scene for Greedy stuck reproduction:

| Property | Value |
|----------|-------|
| Scene ID | `JmbYfDe2QKZ` |
| Type | Matterport3D house |
| Container path | `/data/scene_datasets/mp3d/JmbYfDe2QKZ/JmbYfDe2QKZ.glb` |
| Host path (bind-mounted) | `sim/data/scene_datasets/mp3d/JmbYfDe2QKZ/` |
| Navmesh | `JmbYfDe2QKZ.navmesh` |
| Spawn constraint | Ground floor (via floor_constraint.py, `agent_max_climb=0.15`) |

**Scene characteristics (from JOURNAL):**
- Multi-floor house with stairs
- Default spawn was upper floor; now constrained to ground floor
- Corridors and corners present (good for stuck testing)
- Navmesh rebaked to prevent stair climbing

**File layout on host:**
```
habitat3-exploration/sim/data/scene_datasets/mp3d/
├── mp3d.scene_dataset_config.json
└── JmbYfDe2QKZ/
    ├── JmbYfDe2QKZ.glb      # Main mesh
    ├── JmbYfDe2QKZ.navmesh  # Navigation mesh
    ├── JmbYfDe2QKZ.house    # Metadata
    └── JmbYfDe2QKZ_semantic.ply
```

---

## Appendix C: Related JOURNAL Entries

Key entries from `JOURNAL.md` documenting prior stuck debugging:

- **2026-09-15**: Checkpoint acknowledging "serious nav issues remain"
- **2026-09-09**: Multiple entries on:
  - DiscreteMove corner following (turn_over_drive_ratio fix)
  - Mass-blacklist from wedged start
  - Inflation radius < robot radius bug
  - Inaccessible classification requiring definitive codes
  - Prior-plan-always-OK bug

These fixes are already in `main`. The current investigation focuses on residual issues.
