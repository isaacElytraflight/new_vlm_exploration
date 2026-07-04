# Project journal

Running log of work on **VLM-aided room exploration** (Habitat 3.0 + elytra-bridge). Written for future debugging: what broke, what fixed it, and what *didn't*.

Add a new dated section at the top when you work on this repo.

---

## 2026-07-03 — Nav2 + SLAM sim-to-real mapping, frontier decision tree restructure

### Goals
1. Replace privileged Habitat pathfinder map with **sensor-driven SLAM** (`depth → /scan → slam_toolbox → /grid_map`) and **Nav2** obstacle-aware navigation (port of ROS 1 `move_base` stack).
2. Replace continuous frontier filtering + VLM frontier **selection** + graph/Dijkstra backtrack with a **global frontier decision tree**: on-demand OpenCV detection at leaf nodes, parallel VLM **openness** ratings (0–5), DFS that prefers **lowest** score first, in-place termination when the tree is exhausted.

### What we built

**Nav2 + SLAM pipeline**
- Default launch: `nav2_exploration.launch.py` (includes `exploration.launch.py` + Nav2 + slam_toolbox + depthimage_to_laserscan).
- `explorer_bridge`: `odom → base_link` TF (SLAM publishes `map → odom`); optional privileged `habitat_map_node` gated by `use_privileged_map:=false` default.
- `cmd_vel_to_discrete_node.py` — `/cmd_vel` → `/movement/discrete_move` (ROS 1 `cmd_vel_to_actions` port).
- `Nav2Navigator` + `explore_node` `navigation_mode:=nav2|discrete`.
- Docker: Nav2 + slam_toolbox packages; `config/nav2_params.yaml`, `config/slam_toolbox.yaml`.
- Elytra: real-time motion toggle, Nav2 plan overlay view (`nav-plan-map`).

**Frontier decision tree (replaces old frontier/graph stack)**
- **`FrontierTree` library** — `createRoot`, `addChild`, `selectNextChild` (lowest openness, random tie), `hasUnexploredNodesExcluding` (in-place termination), `markFullyExplored` (score 0 → explored immediately).
- **`explore_node` rewrite** — loop: rotate360 → detect at leaf only (exclusion mask around all tree nodes + radius filter) → parallel VLM rate → navigate lowest-score child or backtrack to parent.
- **Removed:** `frontiers_node`, `graph_node`, `graph_logic`, blacklist, `chosen_frontier`, `FrontierViewsProcess` / `vlm/query`.
- **VLM:** `vlm/rate_frontiers` (`RateFrontierOpenness` action); parallel Ollama HTTP per image; JSON `parse_openness_score()`; no grid map in prompt.
- **ROS topic contract (tiered):**
  - Tier 1: `exploration/frontier_tree`, `exploration/status` (latched)
  - Tier 2: `exploration/vlm/views`, `exploration/vlm/scores`
  - Tier 3 (optional): `exploration/debug/events`, `exploration/debug/last_vlm_batch` via `publish_debug_topics:=true`
- **`maprender_node`:** subscribes `/exploration/frontier_tree`; labels nodes with VLM score 0–5; dropped unused `map_img_raw`.

**New messages**
- `FrontierTree`, `FrontierTreeNode`, `FrontierOpennessScores`, `ExplorationStatus`, `RateFrontierOpenness.action`
- `FrontierViews.frontier_ids` is now `uint32[]` (tree node IDs)

### Problems & fixes

| Problem | Root cause | Fix |
|--------|------------|-----|
| `selectNextChild` picked wrong child | `addChild` held stale `parent*` after `vector` reallocation | Re-`find(parent_id)` after `push_back` |
| colcon build failed on `nav2_msgs` | Missing `find_package(nav2_msgs)` + Nav2 not in container image | Added CMake deps; `apt install ros-jazzy-navigation2` in dev container |
| `Nav2Navigator` incomplete type errors | Forward-declared `NavigateToPose` in header | Full `#include <nav2_msgs/action/navigate_to_pose.hpp>` |

### Verified
- **colcon test** `explorer_mission`: 24/24 gtests pass (frontier detection mask, frontier tree, harness).
- **pytest** `test_py/`: 31/31 pass (including `parse_openness_score`).
- **Elytra** backend: 18/18 Node tests pass.
- Container `colcon build --packages-select explorer_msgs explorer_mission` succeeds after Nav2 packages installed.

### Key files
- `explorer_mission/src/explore_node.cpp` — tree exploration loop
- `explorer_mission/src/frontier_tree.cpp`, `include/explorer_mission/frontier_tree.hpp`
- `explorer_mission/src/frontier_detection.cpp` — `buildExclusionMask`, `filterContoursNearRobot`
- `explorer_mission/explorer_mission/vlm/vlm_node.py` — parallel openness rating
- `explorer_mission/launch/nav2_exploration.launch.py`, `launch/exploration.launch.py`
- `explorer_bridge/cmd_vel_to_discrete_node.py`, `depth_camera_info_node.py`
- `ros_workspace/design_doc.md` — tiered ROS topic contract

### Ops notes
- Rebuild in container after pull: `colcon build --packages-select explorer_msgs explorer_mission`
- Launch episode via Elytra Connect → Run Exploration Episode (`start_sim.sh` uses `nav2_exploration.launch.py`).
- Debug: `ros2 topic echo /exploration/status`, `/exploration/frontier_tree`
- Params: `frontier_detection_radius` (default 5.0 m), `publish_debug_topics` (default false)

---

## 2026-07-01 — Local Ollama VLM, multi-panel sim views, frontier pipeline fixes

### Goal
Replace Gemini-only VLM with **local Ollama** (default) to avoid rate limits; add Elytra
multi-panel simulation dashboard; fix frontier detection/filtering/VLM index bugs so
exploration covers more of the map and VLM choices match map labels.

### What we built

**Local VLM backend (`habitat3-exploration`)**
- Pluggable `vlm/backends/`: `OllamaBackend` (default) + `GeminiBackend` (opt-in).
- Env: `VLM_BACKEND=local`, `VLM_OLLAMA_URL`, `VLM_LOCAL_MODEL`, `VLM_LOCAL_MAX_EDGE`, etc.
- Wired through `docker-compose.yml`, `sim/.env.example`, Elytra Settings `envFields`.
- `qwen3-vl:4b-instruct` on host Ollama (instruct variant — thinking model returns empty at low `num_predict`).
- `think: false` in Ollama `/api/chat`; image downscale before inference; warmup on startup.
- `sim/scripts/benchmark_vlm.py` for manual latency tuning.
- **21** VLM pytest tests (mocked HTTP).

**Elytra multi-panel simulation dashboard (`elytra-bridge`)**
- `project.yaml` `views:` contract — third-person (`/birdseye_data`), RGB (`/image_data`), grid map (`/map_renderer/map_img`).
- `SimulationDashboard.jsx` + `rosViewBridge.js` + `viewConfig.js`; proxy routes `/sim/views`, `/sim/views/:id/frame`.
- `scripts/ros-view-server/elytra_view_server.py` on port **8090** in container.
- Birdseye chase camera in `habitat_engine.py`; `/birdseye_data` in `explorer_bridge_node`.
- **14/14** `simViews.test.js` pass.

**Frontier + exploration fixes**
- Renumber filtered frontier IDs to **0..K-1** (map labels, VLM captions, validation aligned).
- Fix `maprender_node` contour drawing (pixel coords, not world).
- Widen filter: **0.5–15 m**, max **8** frontiers; compute raw frontier midpoints.
- VLM index validation against candidate count (not raw contour count) — fixes spurious choice of `4` when only 2 options.
- `validate_frontier_choice()` in `parsing.py`; skip VLM for single-frontier batches.
- Stronger explore fallback when filtered set empty; stop-reason logging.
- Maprender re-renders on frontier update; `maprender` rate 1 Hz.

**Earlier same session (carried in)**
- Gemini API key fail-fast (`gemini_auth.py`).
- Map image race: `frontier_vlm_client` pending retry + faster maprender.
- `frontier_vlm_client` map batch queue when map not ready yet.

### Problems & fixes

| Problem | Symptom | Root cause | Fix that worked |
|--------|---------|------------|-----------------|
| Gemini rate limits | VLM failures mid-episode | Cloud quota | Local Ollama default; `VLM_BACKEND=gemini` fallback |
| `qwen3-vl:4b` empty response | VLM returns blank / explore fails | Thinking model burns `num_predict` on internal reasoning | Use **`qwen3-vl:4b-instruct`**; `"think": false` in API |
| Compose ignores `sim/.env` | Container still had old `VLM_*` defaults | `${VAR}` resolved from compose cwd, not `env_file` | `docker compose --env-file ../.env up` |
| Frontier labels in white space | Numbers not on boundary | Contour `points` are grid pixels; maprender used `world_to_flipped_pixel` | `grid_pixel_to_flipped_pixel()` |
| VLM picks frontier 4 with 2 options | Wrong navigation target | Validation vs `raw_frontiers_size`, not filtered IDs | Validate `0..N-1`; lookup by array index |
| Few frontiers (count=1–2) | Large gray areas unexplored | 1–5 m filter, max 5, aggressive blacklist | 0.5–15 m, max 8, raw fallback |
| "Freeze" after ~3 steps | Sim runs but no exploration | `explore_node` exited cleanly; orphan VLM query still in flight | Single-frontier: no VLM publish; fallback frontiers; stop logging |
| View server 404 on rgb/bird | Panels empty | QoS mismatch on `sensor_msgs/Image` | `qos_profile_sensor_data` in view server |
| No map at first VLM batch | "No map image yet" | maprender 0.1 Hz + one-shot client | 1 Hz maprender; pending batch retry |

### Verified end state
- Ollama **0.31.1** on Windows host; `qwen3-vl:4b-instruct` pulled; reachable from container (`host.docker.internal:11434`).
- VLM tests **21/21** pytest; explorer_mission gtests pass.
- Multi-panel proxies return HTTP 200 after connect + Run Episode.
- Exploration runs multiple VLM decision cycles with 2–3 frontiers (improved from count=1); still stops early if all candidates blacklisted — fallback logic added today.

### Key files
- `habitat3-exploration/ros_workspace/src/explorer_mission/explorer_mission/vlm/backends/` — Ollama + Gemini
- `habitat3-exploration/ros_workspace/src/explorer_mission/src/explore_node.cpp` — VLM index + fallback
- `habitat3-exploration/ros_workspace/src/explorer_mission/src/frontiers_node.cpp` — filter params + renumber
- `habitat3-exploration/ros_workspace/src/explorer_mission/explorer_mission/maprender_node.py` — contour fix
- `habitat3-exploration/sim/.env.example` — Ollama setup
- `elytra-bridge/application/frontend/src/SimulationDashboard.jsx`
- `elytra-bridge/application/backend/src/rosViewBridge.js`
- `elytra-bridge/scripts/ros-view-server/elytra_view_server.py`

### Ops notes
- **Ollama setup:** install on Windows, `ollama pull qwen3-vl:4b-instruct`, keep running before episodes.
- **Compose env:** `cd sim/docker && docker compose --env-file ../.env up -d sim`
- **Rebuild C++ after changes:** `colcon build --packages-select explorer_mission` in container (or rebuild image for Shutdown persistence).

### Still open
- Exploration may still terminate when all nearby frontiers are blacklisted despite large unknown regions — monitor `Stopping exploration: raw_frontiers=…` log; if `raw_frontiers=0`, trace `habitat_map_node` / incremental reveal.
- VLM latency ~3–12 s per multi-image query on laptop GPU — tune `VLM_LOCAL_MAX_EDGE` (384) or use `benchmark_vlm.py`.
- Birdseye / third-person panel depends on habitat engine chase camera — verify orientation per scene.

---

## 2026-06-25 — Exploration episode E2E: blank screen, zero frontiers, 120 s delay

### Goal
Get **Run Exploration Episode** working end-to-end in Elytra: noVNC shows the
scene, frontier detection finds valid targets in skokloster-castle, and the
explore loop proceeds in seconds (not minutes).

### What we fixed
- **Launch crash → blank noVNC**: `exploration.launch.py` referenced
  `maprender_node` and `vlm_node`, but hybrid `ament_cmake` +
  `ament_cmake_python` never installed Python `console_scripts`. Added
  `install(PROGRAMS … RENAME …)` in `CMakeLists.txt`; shebang on `vlm_node.py`.
- **Habitat IPC**: `get_pose` failed on numpy-quaternion → explicit
  `mn.Quaternion(Vector3(xyz), w)`; `get_map` needed `height` arg for
  `get_topdown_view(mpp, height)`.
- **Incremental explored map**: new `explored_map.py` — navmesh flood-fill from
  agent (4 m sensor range) reveals FREE/OCCUPIED; unobserved cells stay
  UNKNOWN(-1). Wired into `habitat_engine.get_map()`.
- **~120 s action timeout**: `explore_node` blocked on action/service futures
  without spinning → every `wait_for(120s)` hit full timeout. Background
  `MultiThreadedExecutor` + thread-safe frontier state.
- **`frontier_vlm_client_node` crash**: nested `spin_some` inside callback while
  `main()` already spins → FATAL "already added to executor". Made
  `viewsCb` event-driven; timeout via wall timer.
- **`vlm_node` logger**: rclpy rejects printf-style `info("…%s", x)` → f-strings.
- **Docker image** rebuilt twice so C++ fixes survive `compose down` / Shutdown.

### Problems & fixes

| Problem | Symptom | Root cause | Fix that worked |
|--------|---------|------------|-----------------|
| Blank noVNC after Run Episode | tmux session dies; only stale `frame.jpg` | `ros2 launch` aborts: `maprender_node` not in `lib/explorer_mission/` | `install(PROGRAMS)` for Python nodes in `CMakeLists.txt` |
| No `map` TF / no `/grid_map` | `TF lookup failed: "map" … does not exist` | `get_pose` IPC error (quaternion ctor); swallowed at debug level | Explicit magnum quaternion from `(x,y,z,w)` components |
| `/grid_map` empty or errors | `get_map failed: get_topdown_view() … height` | Habitat API needs vertical slice height | Pass agent floor `position[1]` |
| Exploration completes instantly | `Received filtered frontiers (count=0)`; no navigation | Full navmesh published as FREE/OCCUPIED only — **zero UNKNOWN** cells; frontier = free∩unknown boundary | `explored_map.py` incremental reveal (`HABITAT_SENSOR_RANGE_M=4`) |
| ~120 s between scan and frontiers | Captured images at t+0; frontiers at t+120 | `explore_node` `future.wait_for` without executor spin | `MultiThreadedExecutor` on background thread |
| VLM client dies on first decision | `already been added to an executor` | `spin_some` in `viewsCb` while node is in `rclcpp::spin` | Async VLM goal + `checkTimeout` timer; no nested spin |
| Fixes lost after Shutdown | Old bugs return | `compose down` recreates container; `install/` baked in image, not mounted | Rebuild `habitat3-exploration:latest` after C++ changes |

### Verified end state (castle scene)
- Filtered frontiers: **2** (was 0).
- Time to first filtered frontiers: **~0.7 s** (was ~120 s).
- Time to decision point: **~1.2 s**.
- Map cells (example): free ~19k, unknown ~160k (incremental reveal working).
- Tests: `test_explored_map.py` **7/7**; `test_frontier_detection` gtest **6/6**
  (includes positive control + fully-known-map negative control).

### Tests added
- **`sim/scripts/test_explored_map.py`** — partial reveal → frontiers; full reveal →
  no frontiers (negative control); wall blocking; doorway reach; mask accumulation.
- **`test_frontier_detection.cpp`** — `RevealedDiscHasFrontier`,
  `FullyKnownMapHasNoFrontier_NegativeControl`, `FilteredFrontierWithinRange`.

### What did *not* work
- Publishing the **raw Habitat navmesh** as the occupancy grid — mathematically
  frontier-free (no unknown cells).
- **`spin_some` inside action wait loops** — futures never complete; use a
  background executor instead.
- **`ament_python_install_package` alone** for hybrid packages — does not install
  `setup.py` `console_scripts`; need explicit `install(PROGRAMS)`.
- **In-container `colcon build` without image rebuild** — survives Stop/Reset
  only; **Shutdown** (`compose down`) recreates from stale image.

### Elytra lifecycle note
| Action | Container | Fixes persist? |
|--------|-----------|----------------|
| Stop Episode | Kept | Yes |
| Reset Simulation | `compose restart` (same container) | Yes |
| Shutdown → start + connect | `compose down` → recreate from image | Only if image rebuilt |

`scripts/` (`habitat_engine.py`, `explored_map.py`) are bind-mounted — Python
changes are live immediately. C++ binaries require image rebuild or in-container
`colcon build --packages-select explorer_mission`.

### Key files
- `habitat3-exploration/sim/scripts/explored_map.py` — incremental reveal logic
- `habitat3-exploration/sim/scripts/habitat_engine.py` — `get_pose`, revealed `get_map`
- `habitat3-exploration/sim/scripts/test_explored_map.py` — reveal unit tests
- `habitat3-exploration/ros_workspace/src/explorer_mission/CMakeLists.txt` — Python node install
- `habitat3-exploration/ros_workspace/src/explorer_mission/src/explore_node.cpp` — background executor
- `habitat3-exploration/ros_workspace/src/explorer_mission/src/frontier_vlm_client_node.cpp` — no nested spin
- `habitat3-exploration/ros_workspace/src/explorer_mission/explorer_mission/vlm/vlm_node.py` — f-string logging
- `habitat3-exploration/ros_workspace/src/explorer_mission/test/test_frontier_detection.cpp` — regression tests

### Harmless noise (still present)
```
SSD Load Failure! ... skokloster-castle.scn exists but failed to load
```
Mesh-only GLB; `load_semantic_mesh=False`. Ignore.

### Still open
- VLM frontier **selection** needs `GEMINI_API_KEY` in `sim/.env`; without it the
  stack finds frontiers and reaches a decision point but cannot query Gemini.
- `frontier_vlm_client` may log "No map image yet" on the first batch until
  `maprender_node` publishes — retries on the next explore loop.

---

## 2026-06-24 (late) — VLM exploration stack migration (ROS 1 → ROS 2 Jazzy)

### Goal
Port the full `rtabmap_docker` VLM exploration pipeline into `habitat3-exploration`:
frontiers, graph, actions, explore orchestrator, VLM client/server, map renderer.
Replace `move_base` with discrete-move navigation for Habitat sim.

### What we built
- **`explorer_msgs`** extended with Frontier/Graph msgs, AddEdge/RunDijkstra srvs,
  FrontierViewsProcess/Rotate360/PerceiveAndCapture actions.
- **`explorer_mission`** (ament_cmake + Python): C++ nodes `frontiers`, `graph_node`,
  `actions`, `frontier_vlm_client`, `explore`; Python `vlm_node` (gemini-3.5-flash),
  `maprender_node`. Libs: `frontier_detection`, `graph_logic`, `discrete_navigator`.
- **Habitat IPC** `get_pose` / `get_map`; `habitat_map_node` publishes `/grid_map`;
  bridge publishes `/odom` + TF `map`→`base_link`.
- **`exploration.launch.py`** brings up bridge + map + full mission stack.
- **`start_sim.sh`** now launches `explorer_mission exploration.launch.py`.
- **Tests**: 4 gtest + 16 bridge pytest + 8 mission pytest = **28/28 pass**.

### move_base replacement
`explore_node` uses `DiscreteNavigator` (0.25 m steps, 10° turns) via
`/movement/discrete_move` action client — no Nav2 in sim image.

### Key files
- `habitat3-exploration/ros_workspace/src/explorer_mission/`
- `habitat3-exploration/ros_workspace/design_doc.md` (updated exploration contract)
- `habitat3-exploration/sim/scripts/habitat_engine.py` (get_pose, get_map)
- `habitat3-exploration/sim/scripts/run_ros_tests.sh` (gtest + pytest)

---

## 2026-06-24 — ROS 2 Jazzy shim for Habitat (topics + movement action)

### Goal
Expose Habitat over ROS 2 the way the real robot will be wired: publish raw RGB
(`/image_data`) and depth (`/depth_data`), and serve a movement action
(`/movement/discrete_move`) for forward/backward/turn — matching the elytra
`ros_workspace` contract used by drone-2026 (same API, swappable backend).

### What we built
- **`habitat3-exploration/ros_workspace/`** — new colcon workspace:
  - `explorer_msgs` (ament_cmake): `action/DiscreteMove.action`
    (`FORWARD/BACKWARD/TURN_LEFT/TURN_RIGHT`, `steps`).
  - `explorer_bridge` (ament_python): `explorer_bridge_node` publishes
    `/image_data` (rgb8) + `/depth_data` (32FC1) and serves
    `/movement/discrete_move`. Driver dispatch: `habitat | hardware | mock`.
  - `image_utils.py` (pure numpy↔Image), `habitat_ipc.py`, `habitat_driver.py`,
    `hardware_driver.py` (stub), `mock_driver.py` (tests).
- **Two-process split** — `habitat_sim` runs in **conda py3.9**
  (`sim/scripts/habitat_engine.py`); `rclpy` runs in **system py3.12**. They talk
  over a **Unix-domain socket** (`/tmp/habitat_engine.sock`) with a JSON-line
  protocol (`get_obs`/`step`/`reset`/`shutdown`). Engine adds a `DEPTH` sensor and
  explicit `ActuationSpec` (incl. `move_backward`).
- **Docker** bumped to **Ubuntu 24.04 + ROS 2 Jazzy** (`ros-base`,
  `rosidl-default-generators`, colcon); conda/habitat/EGL stack preserved.
  Build context moved to project root; `ros_workspace/src` bind-mounted; added
  `.dockerignore`.
- **Tests** (separate files, with controls) under
  `ros_workspace/src/explorer_bridge/test/` — harness positive/negative controls,
  pure image-utils, mock-engine positive integration, negative integration.
  **12/12 pass**; GPU smoke test ~145 FPS.
- **Movement Demo button** — `project.yaml` button + `run_movement_demo.sh` sends
  forward 1, left 90° (9×10°), forward 2, right 90°, back 2. Added **`oneshot`**
  button support to elytra (`runOneShotScript` in sim/ssh targets) so it runs via
  `docker exec` **without** killing the episode's tmux session.

### Problems & fixes

| Problem | Symptom | Root cause | Fix that worked |
|--------|---------|------------|-----------------|
| Episode crash-loops, noVNC black | start script dies instantly | `ros2 launch` couldn't find node: installed to `bin/` not `lib/explorer_bridge/` | Add **`setup.cfg`** with `install_scripts=$base/lib/explorer_bridge` (matches drone pkg) |
| Script aborts on ROS source | `AMENT_TRACE_SETUP_FILES: unbound variable` | `set -u` + ROS `setup.bash` references optional vars | Drop `-u` (use `set -eo pipefail`); isolate conda procs in subshells |
| noVNC black but topics fine | `/image_data` streams, `frame.jpg` never updates | system py3.12 (rclpy) has **no `imageio`**; write swallowed by try/except | `write_jpeg_frame` uses **Pillow** (present), imageio fallback |
| Smoke test import error | `_ARRAY_API not found` / numpy.core.multiarray fail | habitat-sim needs **numpy<2** | Pin `"numpy<2"` in conda pip install |
| Build fails (apt) | pinned `ros-jazzy-*` versions "not found" | exact deb versions not in repo | Unpin ROS packages |
| Build fails (colcon) | `ModuleNotFoundError: No module named 'em'` | colcon used conda python for rosidl | Force system python on PATH + `-DPython3_EXECUTABLE=/usr/bin/python3`; add `python3-empy` |
| Build context error | `failed to checksum ... ros_workspace/log/latest` | colcon symlinks copied into build context | `.dockerignore` excludes `build/ log/ install/`; copy only `src/` |
| Movement button kills episode | clicking demo stops the sim | normal buttons replace tmux session | `oneshot: true` → `runOneShotScript` (exec, no tmux), stays enabled while `inFlight` |

### What did *not* work
- Pinning ROS deb versions from drone-2026 — those exact builds aren't in the
  Jazzy apt repo for this image; unpinning is required.
- Relying on `imageio` in the ROS process — it lives only in the conda env.
- Bind-mounting the whole `ros_workspace` over the image's built `install/` —
  shadows the colcon build; mount `src/` only and rebuild in-container when needed.

### Robot autonomy note
The old `explore_episode.py` random-walker is **gone from the live path**. The
engine only steps on an action goal, so the agent stays put until a client (the
Movement Demo button, a manual `ros2 action send_goal`, or a future VLM policy)
drives `/movement/discrete_move`. This is intentional — movement is external,
exactly as it will be on the real robot.

### Key files
- `habitat3-exploration/ros_workspace/` — `explorer_msgs`, `explorer_bridge`, `design_doc.md`
- `habitat3-exploration/sim/scripts/habitat_engine.py` — conda engine + IPC
- `habitat3-exploration/sim/scripts/start_sim.sh` — engine + viewer + ROS bridge
- `habitat3-exploration/sim/scripts/run_movement_demo.sh` — demo action sequence
- `habitat3-exploration/sim/scripts/run_ros_tests.sh` — colcon build + pytest
- `habitat3-exploration/sim/docker/Dockerfile` — Noble + Jazzy + conda/habitat
- `habitat3-exploration/project.yaml` — `rosInstallSetupPath`, Movement Demo button
- elytra-bridge `simTarget.js` / `sshTarget.js` / `server.js` / `projectStore.js` / `App.jsx` — `oneshot` action support

### Harmless noise (still present)
```
SSD Load Failure! ... skokloster-castle.scn exists but failed to load
```
Mesh-only GLB; `load_semantic_mesh=False`. Ignore.

---

## 2026-06-11 — Live view pipeline (noVNC felt ~1 FPS)

### Goal
Make the noVNC live viewer match the ~15 FPS frame write rate logged by `explore_episode.py`.

### What we did
- Fixed JPEG temp-file crash (`frame.jpg.tmp` → `frame.tmp.jpg`).
- Replaced **feh** with **`live_viewer.py`** (OpenCV `cv2.imshow` on Xvfb).
- Tuned **x11vnc** in `ensure_display.sh`: `-wait 10 -defer 10 -threads`.
- `start_sim.sh` now always kills stale viewers and starts a fresh `live_viewer.py`.

### Problems & fixes

| Problem | Symptom | Root cause | Fix that worked |
|--------|---------|------------|-----------------|
| JPEG write crash | Episode stops after 1 step; `Could not find a backend to open ... frame.jpg.tmp` | `imageio` picks format from the **last** extension; `.tmp` is not JPEG | Write to `frame.tmp.jpg`, then `os.replace()` → `frame.jpg` |
| noVNC ~1 FPS despite ~14.5 FPS in logs | Log says `~14.5 FPS` view frames; browser updates ~1/sec | **feh `--reload`** reloads the file but often **does not trigger X11 damage**; x11vnc only pushes occasional full refreshes | **`live_viewer.py`** with `cv2.imshow` — direct framebuffer updates |
| Zombie feh | `pgrep` showed `[feh] <defunct>`; new viewer skipped | `start_sim.sh` used `if ! pgrep feh` — zombie PID blocked restart | Always `pkill` feh/live_viewer before starting; don't skip on pgrep alone |
| Misleading diagnosis | Assumed JPEG plugin missing | `imageio` test with `.tmp.jpg` worked fine | Check **filename extension**, not just format/quality args |

### What did *not* work (or was insufficient)
- **Aligning feh `--reload` with `HABITAT_VIEW_FPS`** — helped in theory, did not fix noVNC; damage events were the real issue.
- **Writing JPEG faster (PNG → JPEG, rate-limited encode)** — fixed sim CPU/disk load and log FPS, but noVNC still ~1 FPS until viewer was replaced.
- **Assuming sim render FPS = viewer FPS** — smoke test ~250+ sim steps/s; viewer is a separate pipeline (encode → disk → X11 → VNC → browser).

### Harmless noise (ignore unless you need semantics)
```
SSD Load Failure! ... skokloster-castle.scn exists but failed to load
```
Mesh-only GLB scene; no semantic `.scn` needed. Set `load_semantic_mesh=False` in sim config.

### Key files
- `habitat3-exploration/sim/scripts/explore_episode.py` — writes `frame.jpg` at `HABITAT_VIEW_FPS`
- `habitat3-exploration/sim/scripts/live_viewer.py` — displays frames on `:1`
- `habitat3-exploration/sim/scripts/start_sim.sh` / `stop_sim.sh`
- `habitat3-exploration/sim/scripts/ensure_display.sh` — Xvfb + x11vnc + websockify

---

## 2026-06-10 (evening) — Reset simulation, display stack, elytra stop behavior

### Goal
Recover noVNC after **Reset Simulation**; stop conflating “stop episode” with “restart container”.

### What we did
- Added **`ensure_display.sh`** — (re)starts Xvfb, x11vnc, websockify if missing after container restart.
- `start_sim.sh` calls `ensure_display.sh` before launching viewer/episode.
- Elytra: **Stop Episode** no longer calls `resetSimulation()` unless `resetOnStop: true` (drone-2026 end-mission only; Habitat `stop-sim` does not reset).
- `simTarget.runScript` exports `DISPLAY=:1` explicitly.
- `simTarget.reset()` runs `ensure_display.sh` after compose restart.

### Problems & fixes

| Problem | Symptom | Root cause | Fix that worked |
|--------|---------|------------|-----------------|
| noVNC dead after reset | `feh ERROR: Can't open X display` | `docker compose restart` kills **Xvfb**; entrypoint does not re-run; websockify may survive without a display | `ensure_display.sh` before feh/viewer; call from `reset()` and `start_sim.sh` |
| Stop killed whole sim | Stop button restarted container | Frontend called `resetSimulation()` on every `stopAction` | Only reset when `resetOnStop: true` in `project.yaml` |
| DISPLAY wrong in tmux | Intermittent X failures | `process.env.DISPLAY` on Windows host leaked into `docker exec` env | Hardcode `DISPLAY=:1` in `simTarget.runScript` |

### What did *not* work
- Expecting **entrypoint alone** to fix display after reset — entrypoint runs once at container create, not on `compose restart`.
- Relying on **feh** without checking X is up — feh fails loudly; episode may still run headless.

---

## 2026-06-10 (afternoon) — Elytra-bridge bugs & local clone

### Goal
Debug Habitat + elytra together; fix sim mode assuming drone-2026.

### What we did
- Cloned **elytra-bridge** into `elytra-bridge/` (local fixes; push to GitHub when ready).
- Commits on `main` (local):
  - `e8d134c` — sim mode project-agnostic (`composeService`, `sim.user`, ROS env only when configured; hotswap `relPath` crash).
  - `99792b7` — UI sync: `connectedProject` in `/drone/status`; adopt session at boot.
  - `ccd6768` — persist session in `backend/.elytra/session.json`; re-adopt running sim after backend restart.
  - `3e67fef` — fix project UI desync on reload.
- Tmux log: retain last output when session ends; reconcile `inFlight` when tmux session gone.
- Smoke test: suppress harmless semantic SSD noise; `load_semantic_mesh=False`.

### Problems & fixes

| Problem | Symptom | Root cause | Fix that worked |
|--------|---------|------------|-----------------|
| Connect fails | `docker compose ... build sim` exit 1 | Compose service named `habitat` but elytra runs `build **sim**` | Rename service to **`sim`** in `docker-compose.yml` (name is load-bearing) |
| Build fails after rename | `parent snapshot ... does not exist` | Corrupted Docker BuildKit cache | `docker builder prune -af` |
| Habitat banner, drone buttons | Status says Habitat; buttons/mission are drone-2026 | Frontend: status from API vs buttons from **local default project** | Adopt `connectedProject` from `/drone/status` when connected |
| “Random” UI on refresh | Sometimes correct, sometimes drone default | Backend session **in-memory only**; `node --watch` restarts wipe it while container keeps running | Persist `session.json`; re-adopt on startup |
| Start button stuck disabled | `inFlight` true after script exits | Tmux session ended but session state not cleared | Reconcile on `/drone/tmux-log` when `!hasSession` |
| Tmux log goes empty | Output disappears when session ends | `capture-pane` only works while session exists | Cache `lastTmuxLog` on backend |
| Sim exec as wrong user | Habitat runs as root; drone expects `sim` user | Hardcoded `sim` user in `runScript` | `sim.user` in project.yaml; empty = container default |
| Multiple elytra instances | Port 5173/8787 confusion | Old dev servers still running | Kill by port; use clone on 8788/5174 for agent testing |

### What did *not* work
- **Only fixing the frontend default project** — without backend session persistence, refresh after backend restart still looked “random”.
- **Assuming one elytra instance** — always check `netstat`/task manager for stale node/vite on 5173 and 8787.

### Ops notes
- Clear stale elytra: stop extra `node`/`vite` on 5173/8787; optional `docker compose down` for habitat project.
- Dev clone ports: backend **8788**, frontend **5174** (avoid collision with primary instance).

---

## 2026-06-10 (morning) — Habitat 3.0 + elytra project bootstrap

### Goal
Stand up Habitat 3.0 (habitat-sim/lab **v0.3.1**) as an elytra-bridge project package; install agent-skills as Cursor rules; GPU rendering on WSL2 + Docker Desktop.

### What we built
- **`habitat3-exploration/`** — `project.yaml`, `sim/` (Docker, scripts), stub `real/`, missions mount.
- **Docker image** — Ubuntu 22.04 + Miniforge + habitat-sim 0.3.1 headless + habitat-lab v0.3.1 + Xvfb/x11vnc/noVNC/tmux.
- **Scripts** — `smoke_test.py`, `explore_episode.py`, `start_sim.sh`, `stop_sim.sh`, `download_data.sh`.
- **`.cursor/rules/`** — 28 agent-requested rules from agent-skills pack.
- **`%UserProfile%\.wslconfig`** — memory cap, `autoMemoryReclaim=gradual`, `sparseVhd=true`.

### Problems & fixes (Habitat / GPU / WSL2)

| Problem | Symptom | Root cause | Fix that worked |
|--------|---------|------------|-----------------|
| Dataset download fails | git LFS errors | `git-lfs` not in image | Add `git-lfs` to Dockerfile; `git lfs install` |
| Git Bash path mangling | `docker exec` breaks paths | MSYS converts `/data/...` | Prefix with **`MSYS_NO_PATHCONV=1`** |
| No EGL / CUDA device 0 | `unable to find EGL device for CUDA device 0` | WSL2 has no native NVIDIA EGL; GL goes through Mesa **d3d12** | Compose env: `LD_LIBRARY_PATH=/usr/lib/wsl/lib`, `GALLIUM_DRIVER=d3d12`, `MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA`, mount `/usr/lib/wsl`, device `/dev/dxg` |
| CUDA↔EGL matching | Habitat can't match CUDA device to EGL | d3d12 EGL lacks `EGL_CUDA_DEVICE_NV` | **`HABITAT_GPU_DEVICE_ID=-1`** in compose + sim scripts |
| Segfault on renderer init | Crash when creating RGB sensor | Conda **Mesa/GL** libs shadow system glvnd | Remove conda `libGL*` / `libOpenGL*`; use system **`libopengl0`** + glvnd |
| `sim.previous_step_collided` | AttributeError in explore episode | API change in habitat-sim 0.3.1 | `collided = bool(obs.get("collided", False))` |
| Slow noVNC (first pass) | ~1 FPS | PNG every sim step + feh reload cap | JPEG + rate-limited `HABITAT_VIEW_FPS` (see 2026-06-11 for viewer fix) |

### What did *not* work
- **Default NVIDIA EGL vendor** (`10_nvidia.json`) on WSL2 Docker — no EGL devices; needed Mesa d3d12 path.
- **`gpu_device_id=0`** without bypass — CUDA/EGL enumeration fails on WSL2.
- **Keeping conda OpenGL libs** — segfault in Magnum renderer creation.
- **Large `nvidia/cudagl` base image** — avoided; plain Ubuntu + runtime NVIDIA toolkit injection is enough with correct WSL env.

### Verified end state (2026-06-10)
- Smoke test: D3D12 NVIDIA renderer, ~100–137 FPS stepping.
- Episode via tmux + elytra buttons; noVNC on `http://localhost:6080`.
- Scene data bind-mounted at `habitat3-exploration/sim/data/` → `/data`.

### Key paths
| Path | Purpose |
|------|---------|
| `habitat3-exploration/project.yaml` | Elytra project descriptor |
| `habitat3-exploration/sim/docker/docker-compose.yml` | **Service must be named `sim`** |
| `habitat3-exploration/sim/scripts/` | Runtime scripts (bind-mounted live) |
| `elytra-bridge/` | Local middleware clone |
| `.cursor/rules/` | Agent skills as Cursor rules |

### Environment cheatsheet (WSL2 + Habitat Docker)
```yaml
# docker-compose.yml — WSL2 GPU rendering
LD_LIBRARY_PATH: /usr/lib/wsl/lib
__EGL_VENDOR_LIBRARY_FILENAMES: /usr/share/glvnd/egl_vendor.d/50_mesa.json
GALLIUM_DRIVER: d3d12
MESA_D3D12_DEFAULT_ADAPTER_NAME: NVIDIA
HABITAT_GPU_DEVICE_ID: "-1"
devices:
  - /dev/dxg
volumes:
  - /usr/lib/wsl:/usr/lib/wsl:ro
```

```bash
# Windows Git Bash — avoid path conversion
MSYS_NO_PATHCONV=1 docker exec habitat3-sim ...
```

### Still TODO (as of 2026-06-11)
- Push elytra-bridge commits to GitHub when ready.
- Habitat → ROS bridge (Habitat is not ROS-native).
- VLM policy in `choose_action()` in `explore_episode.py`.
- Populate `real/` when physical robot exists.
