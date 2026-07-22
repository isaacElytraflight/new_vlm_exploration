# Explorer ROS 2 Interface (Jazzy)

Shared contract for Habitat simulation and future physical robot. The same
`ros_workspace` runs in both modes; only `driver_backend` changes.

## Elytra-facing topics (hardware contract)

| Topic | Type | Encoding | Description |
|-------|------|----------|-------------|
| `/image_data` | `sensor_msgs/Image` | `rgb8` | Raw RGB camera frame |
| `/depth_data` | `sensor_msgs/Image` | `32FC1` | Raw depth in meters |

QoS: `sensor_data` profile (best effort, keep last).

## Elytra-facing actions

| Action | Type | Description |
|--------|------|-------------|
| `/movement/discrete_move` | `explorer_msgs/action/DiscreteMove` | Move forward/backward or rotate left/right |

### DiscreteMove directions

- `FORWARD` (0)
- `BACKWARD` (1)
- `TURN_LEFT` (2)
- `TURN_RIGHT` (3)

## Exploration stack (internal, sim + future real)

### SLAM / Nav2 / sensors (unchanged)

| Topic / action | Type | Description |
|----------------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | Synthetic scan from depth (`depth_to_laserscan`) |
| `/grid_map` | `nav_msgs/OccupancyGrid` | Known-pose occupancy from `/scan` + privileged TF |
| `/odom` | `nav_msgs/Odometry` | Privileged pose (Habitat GT / T265): `odom` → `base_link` TF |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 controller output → discrete bridge |
| `/plan`, `/local_plan` | `nav_msgs/Path` | Nav2 global/local plans (overlaid in map renderer) |
| `rotate_360` | `explorer_msgs/Rotate360` | 360° scan with image capture |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | Nav2 frontier navigation |

### Removed topics (legacy)

| Topic | Replaced by |
|-------|-------------|
| `frontiers/frontiers`, `frontiers/filtered_frontiers` | On-demand detection inside `explore_node` |
| `frontier_blacklist` | Frontier decision tree + exclusion mask |
| `chosen_frontier` | Tree DFS selection (lowest VLM openness) |
| `/graph_node/graph`, `/graph_node/graph_markers`, `/graph_node/backtrack_path` | `exploration/frontier_tree` parent-return |
| `vlm/query` (`FrontierViewsProcess`) | `vlm/rate_frontiers` (`RateFrontierOpenness`) |
| `/map_renderer/map_img_raw` | Removed (unused) |

### Tier 1 — operational (always on)

| Topic | Type | QoS | Description |
|-------|------|-----|-------------|
| `exploration/frontier_tree` | `explorer_msgs/FrontierTree` | transient_local | Full decision tree (positions, scores, parent/child, `current_node_id`) |
| `exploration/status` | `explorer_msgs/ExplorationStatus` | transient_local | Phase transitions: `scanning`, `detecting`, `awaiting_vlm`, `selecting`, `navigating`, `backtracking`, `complete` |
| `/map_renderer/map_img` | `sensor_msgs/CompressedImage` | depth 1 | Bird's-eye viz for Elytra (tree nodes labeled with VLM score 0–5) |

### Tier 2 — internal pipeline

| Topic | Type | Description |
|-------|------|-------------|
| `exploration/vlm/views` | `explorer_msgs/FrontierViews` | Batch of frontier images + tree node IDs for VLM rating |
| `exploration/vlm/scores` | `explorer_msgs/FrontierOpennessScores` | Parallel openness scores (0–5) keyed by node ID |
| `vlm/rate_frontiers` | `explorer_msgs/RateFrontierOpenness` | Action: rate each frontier image independently |

### Tier 3 — optional debug (`publish_debug_topics:=true` on `explore_node`)

| Topic | Type | Description |
|-------|------|-------------|
| `exploration/debug/events` | `diagnostic_msgs/DiagnosticArray` | Timestamped phase/event log |
| `exploration/debug/last_vlm_batch` | `explorer_msgs/FrontierViews` | Last VLM batch republish |

### Debug cheat sheet

| Symptom | Echo |
|---------|------|
| Stuck waiting on VLM | `exploration/status`, `exploration/vlm/scores` |
| Wrong navigation target | `exploration/frontier_tree`, `/plan` |
| Tree not updating | `exploration/frontier_tree` |
| Episode finished? | `exploration/status` (`exploration_complete`) |

### Mapping pipeline (sim-to-real parity)

Default launch (`nav2_exploration.launch.py`) mirrors the real robot (T265 pose + depth mapping):

1. Privileged pose: Habitat GT (sim) or T265 (real) → `/odom` + `odom`→`base_link`; `map`→`odom` identity
2. `/depth_data` → `depth_to_laserscan` → `/scan`
3. `/scan` + matching `/odom` stamp → `known_pose_mapper` → `/grid_map` (no slam_toolbox)
4. `explore_node` reads `/grid_map` on demand at tree leaf nodes

Debug-only: `use_privileged_map:=true` restores `habitat_map_node` (Habitat pathfinder / `get_map` IPC).
Dashboard: **Depth Debug** view colorizes `/depth_data` (NaN / zero / sat / valid).

#### Depth → scan (what actually fixed phantom walls)

The Habitat depth camera sits ~0.1 m above the floor. A **center** (or bottom)
row band intermittently samples the floor and paints linear occupied “walls”
a few meters ahead.

**Solution:** `band_anchor: upper_third` — a 24-row band centered at row
`H/3` (480 → rows 148–172), looking slightly upward at wall geometry. Keep
`scan_height: 24`, FOV-only bins (`full_360: false`), uncovered / saturated
bearings as NaN.

Supporting (not the phantom-wall fix, but keep):

| Setting | Value | Why |
|---------|-------|-----|
| Habitat `depth.far` | 50 m | Room walls get true depth; voids saturate near far |
| `range_max` | 10 m | Mapping horizon |
| Near-`sensor_far` depth | NaN | Skip ray → UNKNOWN (do not invent free arcs or fake far walls) |

**What did not fix phantoms:** clipping everything within ~2.5 m of `range_max`
to “free” (`free_near_eps`). That hid real far walls and did not stop floor hits.

### Navigation

- **Default:** `explore_node` with `navigation_mode:=nav2` sends `NavigateToPose` goals; Nav2 plans on costmaps; `/cmd_vel` is converted to discrete Habitat steps via `cmd_vel_to_discrete_node`.
- **Fallback:** `navigation_mode:=discrete` uses straight-line `discrete_navigator` + `/movement/discrete_move` (no obstacle planning).

### Exploration loop (frontier tree)

1. Rotate 360° and cache directional images.
2. At leaf nodes only: detect frontier boundaries within `frontier_detection_radius`, excluding circles around existing tree nodes; rate each child via VLM (0–5 openness).
3. DFS: pick lowest-score unexplored child; navigate; on failure mark child explored and return to parent; terminate in place when no unexplored nodes remain (no mandatory return to root).

## Backends

| Parameter | Values | Sim | Real |
|-----------|--------|-----|------|
| `driver_backend` | `habitat`, `hardware`, `mock` | `habitat` | `hardware` |

- **habitat**: IPC to `habitat_engine.py` (`get_obs`, `step`, `get_pose`, `get_map`, `reset`, `shutdown`)
- **hardware**: stub until `real/` is wired
- **mock**: fixed arrays for unit tests

## Process layout (sim)

1. `habitat_engine.py` — conda Python, owns `habitat_sim`
2. `explorer_bridge_node` — sensors, discrete move action, `odom` → `base_link` TF
3. `depth_to_laserscan` + `known_pose_mapper` — sensor `/grid_map` with privileged pose
4. Nav2 stack — planner, controller, costmaps
5. `cmd_vel_to_discrete_node` — `/cmd_vel` → `/movement/discrete_move`
6. `explorer_mission` nodes — actions, explore, VLM client, VLM server, map renderer
7. `live_viewer.py` — noVNC frame display
