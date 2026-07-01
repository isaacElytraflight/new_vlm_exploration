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

| Topic / service / action | Type | Description |
|--------------------------|------|-------------|
| `/grid_map` | `nav_msgs/OccupancyGrid` | Habitat pathfinder top-down map |
| `/odom` | `nav_msgs/Odometry` | Robot pose in `map` frame |
| `frontiers/frontiers` | `explorer_msgs/FrontierArray` | Raw detected frontiers |
| `frontiers/filtered_frontiers` | `explorer_msgs/FrontierArray` | Distance/blacklist filtered |
| `frontiers/frontier_views` | `explorer_msgs/FrontierViews` | Per-frontier camera views |
| `/graph_node/graph` | `explorer_msgs/Graph` | Topological exploration graph |
| `vlm/query` | `explorer_msgs/FrontierViewsProcess` | VLM frontier selection |
| `rotate_360` | `explorer_msgs/Rotate360` | 360° scan with image capture |
| `chosen_frontier` | `std_msgs/Int8` | VLM-selected frontier index |

Navigation in sim uses `DiscreteMove` sequences (replaces ROS 1 `move_base`). Real robot will use Nav2 adapter (stub).

## Backends

| Parameter | Values | Sim | Real |
|-----------|--------|-----|------|
| `driver_backend` | `habitat`, `hardware`, `mock` | `habitat` | `hardware` |

- **habitat**: IPC to `habitat_engine.py` (`get_obs`, `step`, `get_pose`, `get_map`, `reset`, `shutdown`)
- **hardware**: stub until `real/` is wired
- **mock**: fixed arrays for unit tests

## Process layout (sim)

1. `habitat_engine.py` — conda Python, owns `habitat_sim`
2. `explorer_bridge_node` — ROS Jazzy, sensors + movement action + odom/TF
3. `habitat_map_node` — publishes `/grid_map` from pathfinder
4. `explorer_mission` nodes — frontiers, graph, actions, explore, VLM pipeline
5. `live_viewer.py` — noVNC frame display

## Elytra paths

- Sim ROS setup: `/opt/explorer_workspace/ros_workspace/install/setup.bash`
- Launch: `ros2 launch explorer_mission exploration.launch.py`
- VLM defaults to **local Ollama** (`VLM_BACKEND=local`) via `http://host.docker.internal:11434`. Pull `qwen2.5vl:3b` on the host before running episodes. Optional Gemini cloud: `VLM_BACKEND=gemini` + `GEMINI_API_KEY` in `sim/.env` or Elytra backend `.env`.
- Tune local latency with `VLM_LOCAL_MAX_EDGE` (default 512) or run `python sim/scripts/benchmark_vlm.py` on the host with Ollama running.
