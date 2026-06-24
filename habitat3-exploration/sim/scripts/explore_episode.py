"""Continuous exploration episode for the elytra-bridge "start" button.

DEPRECATED: start_sim.sh now launches habitat_engine.py + the ROS bridge.
This script remains as a standalone random-walk fallback without ROS.
"""

import os
import random
import signal
import time

import imageio.v2 as imageio
import habitat_sim

SCENE = os.environ.get(
    "HABITAT_SCENE",
    "/data/scene_datasets/habitat-test-scenes/skokloster-castle.glb",
)
LIVE_DIR = "/tmp/habitat_live"
# JPEG encodes much faster than PNG — important when updating every frame.
FRAME = os.path.join(LIVE_DIR, "frame.jpg")
# Target noVNC refresh rate (sim can step faster; we only encode when due).
VIEW_FPS = float(os.environ.get("HABITAT_VIEW_FPS", "15"))

_running = True


def _handle_stop(signum, frame):
    global _running
    _running = False


def make_sim() -> habitat_sim.Simulator:
    sim_cfg = habitat_sim.SimulatorConfiguration()
    sim_cfg.scene_id = SCENE
    sim_cfg.enable_physics = True
    sim_cfg.load_semantic_mesh = False
    # -1 on WSL2: skip CUDA<->EGL matching (see smoke_test.py for details)
    sim_cfg.gpu_device_id = int(os.environ.get("HABITAT_GPU_DEVICE_ID", "0"))

    rgb = habitat_sim.CameraSensorSpec()
    rgb.uuid = "rgb"
    rgb.sensor_type = habitat_sim.SensorType.COLOR
    rgb.resolution = [480, 640]
    rgb.position = [0.0, 1.5, 0.0]

    agent_cfg = habitat_sim.agent.AgentConfiguration()
    agent_cfg.sensor_specifications = [rgb]

    return habitat_sim.Simulator(habitat_sim.Configuration(sim_cfg, [agent_cfg]))


def choose_action(collided: bool) -> str:
    """Placeholder policy: walk forward, turn away from collisions."""
    if collided:
        return random.choice(["turn_left", "turn_right"])
    return random.choices(
        ["move_forward", "turn_left", "turn_right"], weights=[0.7, 0.15, 0.15]
    )[0]


def main() -> None:
    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    if not os.path.exists(SCENE):
        raise SystemExit(f"Scene not found: {SCENE} — run download_data.sh first.")

    os.makedirs(LIVE_DIR, exist_ok=True)
    sim = make_sim()
    print(f"Exploring {SCENE} — live frame at {FRAME} (view via noVNC :6080)")

    collided = False
    step = 0
    frames_written = 0
    frame_interval = 1.0 / max(VIEW_FPS, 1.0)
    loop_start = time.perf_counter()
    next_frame_at = loop_start
    print(f"Live view target: {VIEW_FPS:.0f} FPS (set HABITAT_VIEW_FPS to change)")

    try:
        while _running:
            obs = sim.step(choose_action(collided))
            collided = bool(obs.get("collided", False))
            step += 1
            now = time.perf_counter()
            if now >= next_frame_at:
                # Must end in .jpg so imageio picks the JPEG backend (not .jpg.tmp)
                tmp = os.path.join(LIVE_DIR, "frame.tmp.jpg")
                imageio.imwrite(tmp, obs["rgb"][..., :3], quality=85)
                os.replace(tmp, FRAME)
                frames_written += 1
                next_frame_at = now + frame_interval
            if step % 500 == 0:
                elapsed = now - loop_start
                print(
                    f"step {step} | ~{step / elapsed:.0f} sim steps/s | "
                    f"{frames_written} view frames (~{frames_written / elapsed:.1f} FPS)"
                )
    finally:
        sim.close()
        print(f"Episode stopped after {step} steps.")


if __name__ == "__main__":
    main()
