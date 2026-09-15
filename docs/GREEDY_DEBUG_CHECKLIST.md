# Greedy Stuck Debug Checklist

Quick reference for collecting diagnostic data during Greedy ablation runs.

---

## Pre-Run Setup

### 1. Enable Debug Topics (Optional)

In `nav2_exploration.launch.py`, the explore node has `publish_debug_topics` parameter:

```bash
# Inside container, after stack starts:
ros2 param set /explore publish_debug_topics true
```

This enables:
- `/exploration/debug/events` — DiagnosticArray with phase changes
- `/exploration/debug/last_vlm_batch` — last frontier views (not useful for greedy)

### 2. Enable Nav2 Verbose Logging (Optional)

```bash
# Inside container:
ros2 run rclcpp_py2_utils set_log_level /bt_navigator debug
ros2 run rclcpp_py2_utils set_log_level /planner_server debug
ros2 run rclcpp_py2_utils set_log_level /controller_server debug
```

### 3. Verify Brain ID

```bash
# Inside container:
ros2 param get /explore brain_id
# Should return: String value is: greedy_nearest
```

---

## During-Run Monitoring

### Key Topics to Watch

```bash
# In separate terminals inside container:

# Exploration status (phase changes, termination)
ros2 topic echo /exploration/status

# Brain decisions (goal selection)
ros2 topic echo /exploration/brain/decision

# Frontier tree (flat nodes for greedy)
ros2 topic echo /exploration/frontier_tree

# Nav2 feedback
ros2 topic echo /navigate_to_pose/_action/feedback
```

### RViz Quick Setup

Open RViz in noVNC (http://localhost:6080) with:
- `/grid_map` → OccupancyGrid
- `/global_costmap/costmap` → Costmap2D
- `/plan` → Path (blue line)
- `/exploration/frontier_tree` → MarkerArray (if published)
- TF: map → base_link

---

## On Failure — Data Collection

### 1. Identify Failure Type

Check `run_metrics.json`:
```bash
cat sim/data/experiments/<experiment_id>/<run_id>/run_metrics.json | jq '.status, .termination_reason, .final_coverage'
```

| Field | Stuck Pattern |
|-------|---------------|
| `termination_reason` | `"stuck"` or `"success"` with low coverage |
| `final_coverage` | < 0.89 |
| `duration_s` | Very short (< 60s) = mass blacklist |

### 2. Analyze Event Log

```bash
# Decompress if gzipped
gunzip -k logs/events.jsonl.gz

# Find termination
grep '"exploration_complete":true' logs/events.jsonl

# Count frontier deaths
grep '"fully_explored":true' logs/events.jsonl | wc -l

# See decision sequence
grep 'brain/decision' logs/events.jsonl | jq '.action, .goal_id, .detail, .live_ids | length'
```

### 3. Key Questions to Answer

For each stuck episode, determine:

1. **How many frontiers were killed?**
   ```bash
   grep 'brain/decision' logs/events.jsonl | tail -1 | jq '.live_ids | length'
   # Should be 0 if "no live frontiers"
   ```

2. **What was the final robot position vs. frontiers?**
   - Check last `brain/decision` for `goal_x`, `goal_y`
   - Compare to robot pose in same event

3. **Was it truly stuck or false positive?**
   - Look at `media/final_grid_map.png`
   - Is there unexplored area visible?
   - Are there corridors/corners in the map?

4. **What Nav2 error codes appeared?**
   ```bash
   # In container logs or event logger output
   grep "nav_code\|nav_error" /path/to/logs/*.log
   ```

---

## Specific Failure Patterns

### Pattern A: Mass Blacklist (All Frontiers Dead in Seconds)

**Symptoms:**
- Episode ends in < 60s
- Many frontiers marked dead rapidly
- `termination_reason=success` but < 50% coverage

**Check:**
1. Start clearance at failure time
2. Costmap inflation vs robot position
3. Whether `prior_plan_ok` was true via `nearPose()` (false positive)

### Pattern B: Single Corner Trap

**Symptoms:**
- Robot navigates into corner
- Repeated `unsticking` phases
- Eventually `termination_reason=stuck`

**Check:**
1. Frontier positions relative to corner geometry
2. Whether inset (0.35m) was sufficient
3. RPP controller vs planner disagreement

### Pattern C: Frontier Detection Gap

**Symptoms:**
- Coverage plateaus at ~60-70%
- `no live frontiers` but unexplored area visible on map

**Check:**
1. `frontier_detection_radius` vs map extent
2. Whether occlusion hides frontiers from detection
3. Dedupe radius (1m) removing valid frontiers

---

## Container Commands Reference

```bash
# Shell into container
docker exec -it habitat3-sim bash -l

# Source ROS
source /opt/ros/jazzy/setup.bash
source /opt/explorer_workspace/ros_workspace/install/setup.bash

# Check explore node params
ros2 param list /explore

# Get specific param
ros2 param get /explore unstick_min_clearance_m
ros2 param get /explore unstick_max_attempts

# Check Nav2 inflation
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius

# Kill stuck episode
ros2 lifecycle set /bt_navigator shutdown
```

---

## Artifacts Location

After run completes, artifacts are in:
```
sim/data/experiments/<experiment_id>/<algorithm_id>_seed<N>/
├── run_metrics.json      # Final stats
├── run_info.json         # Run metadata
├── manifest.json         # Package manifest
├── logs/
│   ├── events.jsonl.gz   # Full event log
│   ├── collector.log     # experiment_collect output
│   ├── event_logger.log  # Logger status
│   └── media_recorder.log
└── media/
    ├── final_grid_map.png
    ├── final_nav_plan.png
    └── map_timelapse_10x.mp4
```
