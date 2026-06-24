# Project journal

Running log of work on **VLM-aided room exploration** (Habitat 3.0 + elytra-bridge). Written for future debugging: what broke, what fixed it, and what *didn't*.

Add a new dated section at the top when you work on this repo.

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
