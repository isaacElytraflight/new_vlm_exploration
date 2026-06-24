# real/ — physical robot compartment (stub)

No physical robot is wired to this project yet. This compartment exists because
the Elytra project-folder contract requires both `real/` and `sim/` to be
present.

When a robot joins the project:

1. Fill in `host`, `user`, key paths, and script paths in the `real:` block of
   `../project.yaml`.
2. Copy `.env.example` to `.env` and add secrets/overrides (loaded only when
   connecting in physical mode).
3. Add the on-robot start/recording scripts referenced by `startScriptPath` /
   `recordingScriptPath`.
4. Deploy the same `../ros_workspace/` to the robot and build with colcon.
5. Launch `explorer_bridge_node` with `driver_backend:=hardware` after
   implementing `HardwareDriver` in `explorer_bridge/hardware_driver.py`.

The ROS interface (sim and real) is documented in
`../ros_workspace/design_doc.md`:

- `/image_data` — raw RGB
- `/depth_data` — raw depth (lidar-style depth image)
- `/movement/discrete_move` — forward/backward/turn actions

Set `real.rosInstallSetupPath` in `project.yaml` to the on-robot colcon
install `setup.bash` (mirrors drone-2026 physical mode).
