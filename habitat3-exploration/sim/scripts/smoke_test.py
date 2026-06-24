"""GPU smoke test: load a test scene, step a random agent, report FPS.

Run via run_smoke_test.sh (it activates the conda env and un-quiets Magnum logs
so the active GL renderer line — which should contain "NVIDIA" — is printed).
"""

import os
import random
import time

import imageio.v2 as imageio
import habitat_sim

SCENE = "/data/scene_datasets/habitat-test-scenes/skokloster-castle.glb"
OUTPUT_DIR = "/data/smoke_test_output"
NUM_STEPS = 200


def make_sim() -> habitat_sim.Simulator:
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = SCENE
    sim_cfg.enable_physics = True
    # Test scenes are GLB-only (no .scn / info_semantic.json). Without this,
    # habitat-sim logs scary-looking semantic load failures that are harmless.
    sim_cfg.load_semantic_mesh = False
    # -1 skips CUDA<->EGL device matching, which always fails on WSL2 where GL
    # goes through Mesa d3d12 (no EGL_CUDA_DEVICE_NV attribute). Magnum then
    # uses the first EGL device, which is still the hardware GPU.
    sim_cfg.gpu_device_id = int(os.environ.get("HABITAT_GPU_DEVICE_ID", "0"))

    rgb = habitat_sim.CameraSensorSpec()
    rgb.uuid = "rgb"
    rgb.sensor_type = habitat_sim.SensorType.COLOR
    rgb.resolution = [480, 640]
    rgb.position = [0.0, 1.5, 0.0]

    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [rgb]

    return habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg]))


def main() -> None:
    if not os.path.exists(SCENE):
        raise SystemExit(
            f"Scene not found: {SCENE}\n"
            "Run download_data.sh first (docker exec habitat3-sim bash "
            "/workspace/scripts/download_data.sh)."
        )

    os.makedirs(OUTPUT_DIR, exist_ok=True)
    print(
        "Loading skokloster-castle (mesh-only test scene; no semantic .scn). "
        "SemanticScene warnings in the log are harmless."
    )
    sim = make_sim()
    actions = ["move_forward", "turn_left", "turn_right"]

    start = time.perf_counter()
    for step in range(NUM_STEPS):
        obs = sim.step(random.choice(actions))
        if step % 50 == 0:
            frame_path = os.path.join(OUTPUT_DIR, f"frame_{step:04d}.png")
            imageio.imwrite(frame_path, obs["rgb"][..., :3])
    elapsed = time.perf_counter() - start

    sim.close()
    print(f"OK: stepped {NUM_STEPS} steps in {elapsed:.2f}s "
          f"({NUM_STEPS / elapsed:.1f} FPS at 640x480)")
    print(f"Sample frames written to {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
