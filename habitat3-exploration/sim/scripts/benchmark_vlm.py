#!/usr/bin/env python3
"""Manual latency benchmark for the local Ollama VLM backend.

Usage (on host, with Ollama running):
  python sim/scripts/benchmark_vlm.py

Optional env: VLM_OLLAMA_URL, VLM_LOCAL_MODEL, VLM_LOCAL_MAX_EDGE
"""

from __future__ import annotations

import io
import os
import sys
import time

try:
    from PIL import Image
except ImportError:
    print("Install Pillow to run this benchmark.", file=sys.stderr)
    raise SystemExit(1)

# Allow running from repo root without ROS install.
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
MISSION_SRC = os.path.join(ROOT, "ros_workspace", "src", "explorer_mission")
if MISSION_SRC not in sys.path:
    sys.path.insert(0, MISSION_SRC)

from sensor_msgs.msg import CompressedImage  # noqa: E402

from explorer_mission.vlm.backends.ollama import OllamaBackend  # noqa: E402


def _make_msg(width: int, height: int, color: tuple[int, int, int]) -> CompressedImage:
    buf = io.BytesIO()
    Image.new("RGB", (width, height), color=color).save(buf, format="JPEG")
    msg = CompressedImage()
    msg.format = "jpeg"
    msg.data = list(buf.getvalue())
    return msg


def main() -> None:
    backend = OllamaBackend(warmup=True)
    backend.validate()

    images = [
        ("Current occupancy map.", _make_msg(640, 480, (40, 40, 40))),
        ("Image for Frontier 0.", _make_msg(640, 480, (80, 120, 200))),
        ("Image for Frontier 1.", _make_msg(640, 480, (200, 120, 80))),
    ]
    prompt = (
        "Choose the best frontier. Reply with only the frontier number on the first line."
    )

    started = time.perf_counter()
    response = backend.query(prompt, images)
    elapsed = time.perf_counter() - started
    print(f"Model: {backend.model_label}")
    print(f"Elapsed: {elapsed:.2f}s")
    print(f"Response:\n{response}")


if __name__ == "__main__":
    main()
