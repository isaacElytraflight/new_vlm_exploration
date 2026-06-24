# Explorer ROS 2 Interface (Jazzy)

Shared contract for Habitat simulation and future physical robot. The same
`ros_workspace` runs in both modes; only `driver_backend` changes.

## Topics

| Topic | Type | Encoding | Description |
|-------|------|----------|-------------|
| `/image_data` | `sensor_msgs/Image` | `rgb8` | Raw RGB camera frame |
| `/depth_data` | `sensor_msgs/Image` | `32FC1` | Raw depth in meters (lidar-style depth image) |

QoS: `sensor_data` profile (best effort, keep last).

## Actions

| Action | Type | Description |
|--------|------|-------------|
| `/movement/discrete_move` | `explorer_msgs/action/DiscreteMove` | Move forward/backward or rotate left/right |

### DiscreteMove directions

- `FORWARD` (0)
- `BACKWARD` (1)
- `TURN_LEFT` (2)
- `TURN_RIGHT` (3)

## Backends

| Parameter | Values | Sim | Real |
|-----------|--------|-----|------|
| `driver_backend` | `habitat`, `hardware`, `mock` | `habitat` | `hardware` |

- **habitat**: IPC to `habitat_engine.py` (conda, Unix socket `/tmp/habitat_engine.sock`)
- **hardware**: stub until `real/` is wired
- **mock**: fixed arrays for unit tests (no GPU)

## Process layout (sim)

1. `habitat_engine.py` — conda Python, owns `habitat_sim`
2. `explorer_bridge_node` — ROS Jazzy, publishes topics + action server
3. `live_viewer.py` — noVNC frame display from `/tmp/habitat_live/frame.jpg`

## Elytra paths

- Sim ROS setup: `/opt/explorer_workspace/ros_workspace/install/setup.bash`
- Set in `project.yaml` → `sim.rosInstallSetupPath`
