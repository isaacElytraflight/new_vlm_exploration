# habitat3-exploration

Elytra Bridge project package wrapping **Meta Habitat 3.0** (habitat-lab/habitat-sim
v0.3.x) for VLM-aided room-exploration research.

The package follows the [Elytra project-folder contract](https://github.com/isaacElytraflight/elytra-bridge/blob/main/docs/project-folder-contract.md):

```
habitat3-exploration/
├── project.yaml             # descriptor read by the elytra-bridge backend
├── real/                    # stub compartment (no physical robot yet)
├── sim/
│   ├── .env.example         # sim-mode env overrides (copy to sim/.env)
│   ├── docker/              # Dockerfile + docker-compose.yml
│   ├── scripts/             # start/stop/smoke-test scripts (run inside container)
│   └── data/                # scene datasets (bind-mounted, gitignored)
└── missions/                # mission saves (bind-mounted into the container)
```

## Prerequisites

- Windows with WSL2 + Docker Desktop (WSL2 backend)
- NVIDIA GPU with a current driver (GPU passthrough into containers is provided
  by Docker Desktop's bundled NVIDIA container toolkit)
- Recommended: cap WSL2 memory via `%UserProfile%\.wslconfig` (see repo root README)

## First-time setup

```bash
cd habitat3-exploration/sim/docker

# 1. Build the image (~10-15 min the first time)
docker compose build

# 2. Start the container (also starts Xvfb + noVNC on http://localhost:6080)
docker compose up -d

# 3. One-time: download free test scenes into the bind-mounted data/ folder
docker exec habitat3-sim bash /workspace/scripts/download_data.sh

# 4. (Recommended) Default research scene: Matterport3D JmbYfDe2QKZ
#    See "Matterport3D default scene" below — requires Matterport ToS access.
#    Not checked into git and not baked into the Docker image.

# 5. Verify GPU rendering end to end
docker exec habitat3-sim bash /workspace/scripts/run_smoke_test.sh
```

The smoke test prints the active GL renderer (should contain `NVIDIA`), steps a
random agent through `skokloster-castle.glb`, reports FPS, and writes sample RGB
frames to `sim/data/smoke_test_output/`.

### Matterport3D default scene (`JmbYfDe2QKZ`)

Elytra defaults to this house for exploration episodes:

`/data/scene_datasets/mp3d/JmbYfDe2QKZ/JmbYfDe2QKZ.glb`

**Why it is not in the repo or image:** Matterport’s terms forbid redistributing
the mesh. Each machine must download it after you personally have access. Data
lives only under host `sim/data/` (gitignored), bind-mounted to `/data`.

#### One-time access (per person / lab)

1. Open https://niessner.github.io/Matterport/
2. Sign the Terms of Use and email `matterport3d@googlegroups.com` to request
   access.
3. Matterport sends you `download_mp.py` (keep it; you need it as proof of
   access / for the official full-archive path). You do **not** need to run the
   full 1.3 TB scan dump for Habitat.

Official Habitat notes:
https://github.com/facebookresearch/habitat-sim/blob/main/DATASETS.md#matterport3d-mp3d-dataset

#### Download only `JmbYfDe2QKZ` on a new device (~200 MB, not the 16 GB zip)

From the **host** (repo root), after the container exists so `sim/data` is the
same tree the container mounts:

```bash
# Host Python 3 is fine — writes into habitat3-exploration/sim/data/...
python habitat3-exploration/sim/scripts/download_mp3d_habitat_scene.py \
  --i-agree-to-mp-tos \
  --scene JmbYfDe2QKZ
```

`--i-agree-to-mp-tos` means you already agreed to the Matterport ToS (same
agreement that got you `download_mp.py`). The script pulls **only** that house’s
byte ranges from the public Habitat task archive (`mp3d_habitat.zip`), then
writes:

```
sim/data/scene_datasets/mp3d/
  mp3d.scene_dataset_config.json
  JmbYfDe2QKZ/
    JmbYfDe2QKZ.glb
    JmbYfDe2QKZ.navmesh
    JmbYfDe2QKZ.house
    JmbYfDe2QKZ_semantic.ply
```

Confirm inside the container:

```bash
docker exec habitat3-sim ls -lah /data/scene_datasets/mp3d/JmbYfDe2QKZ/
```

Then in Elytra: Scene dropdown → **Matterport JmbYfDe2QKZ** (default) →
**Run Exploration Episode**.

#### Alternate: official `download_mp.py` (full habitat task zip ~16 GB)

If you prefer Matterport’s script (Python 2 era; use a Py2 env or adapt):

```bash
# Downloads v1/tasks/mp3d_habitat.zip under -o, then unpack and copy one house:
python download_mp.py -o /tmp/mp3d_dl --task_data habitat
# Answer the ToS prompt, then CTRL-C before it starts the full scan dump
# if the script continues past task_data.
unzip /tmp/mp3d_dl/v1/tasks/mp3d_habitat.zip -d /tmp/mp3d_habitat
# Copy JmbYfDe2QKZ/ into habitat3-exploration/sim/data/scene_datasets/mp3d/
```

Or use the wrapper that expects `download_mp.py` on disk:

```bash
docker exec habitat3-sim bash /workspace/scripts/download_mp3d_scene.sh \
  /path/to/download_mp.py JmbYfDe2QKZ
```

#### Checklist for a new laptop / CI machine

- [ ] `docker compose build && docker compose up -d` under `sim/docker`
- [ ] `docker exec habitat3-sim bash /workspace/scripts/download_data.sh` (free test scenes)
- [ ] Matterport access + `download_mp3d_habitat_scene.py --i-agree-to-mp-tos`
- [ ] `ls sim/data/scene_datasets/mp3d/JmbYfDe2QKZ/JmbYfDe2QKZ.glb` on the host
- [ ] Elytra Connect (sim) → Run Exploration Episode

### EGL troubleshooting (WSL2)

If the smoke test fails with `no EGL devices found` / `unable to find EGL device
for CUDA device 0`, try in order (set in `sim/.env` or compose `environment:`):

1. Ensure the host NVIDIA driver is current, then `docker compose up -d --force-recreate`.
2. `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json`
   (already set by the image; confirm it was not overridden).
3. `HSIM_DISABLE_CUDA_DEVICE=1` — bypasses CUDA device enumeration, which is the
   known WSL2 limitation, and falls back to the D3D12/dxg path.

## Using with elytra-bridge

Open this folder in elytra-bridge via **File → Open Project** and connect in
**simulation** mode. The backend will:

- bring up `sim/docker/docker-compose.yml` (project `habitat3-exploration`,
  container `habitat3-sim`)
- show noVNC at `http://localhost:6080` in the viewer iframe
- run the **Run Exploration Episode** button through
  `/workspace/scripts/start_sim.sh` inside the `habitat` tmux session

Copy `sim/.env.example` to `sim/.env` for local overrides (loaded only when
connecting in sim mode). VLM frontier selection defaults to **local Ollama** on
the host (`VLM_BACKEND=local`, model `qwen2.5vl:3b`). Install
[Ollama](https://ollama.com), run `ollama pull qwen2.5vl:3b`, then start an
episode. Optional cloud fallback: set `VLM_BACKEND=gemini` and `GEMINI_API_KEY`
in `sim/.env` (see `sim/.env.example`); values are injected into the container
via docker-compose `env_file`. Tune local latency with `VLM_LOCAL_MAX_EDGE` or
run `python sim/scripts/benchmark_vlm.py` on the host (requires Ollama + Pillow).

## Notes

- Scene datasets live in `sim/data/` on the host (bind-mounted to `/data` in the
  container) so image rebuilds never re-download them. That folder is
  **gitignored** — clone the repo on a new device, then re-run
  `download_data.sh` plus the Matterport scene steps above.
- The `real/` compartment is intentionally minimal; populate it when a physical
  robot joins the project.
- Habitat is not ROS-based. ROS topic bridging (for parity with other elytra
  projects) is a planned follow-up; `ros.distro` in `project.yaml` is a
  placeholder until then.
