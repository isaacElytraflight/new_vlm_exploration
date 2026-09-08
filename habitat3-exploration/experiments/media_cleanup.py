"""Helpers for cleaning up run media packages (frames → mp4 or delete)."""

from __future__ import annotations

import shlex
import shutil
import subprocess
from pathlib import Path
from typing import Optional


def cleanup_run_media(
    artifact_dir: Path | str,
    *,
    container_name: str = "habitat3-sim",
    container_run_dir: Optional[str] = None,
    try_encode: bool = True,
) -> dict:
    """Encode leftover frames to MP4 if needed, then delete frames/.

    Returns a small status dict for logging/tests.
    """
    artifact_dir = Path(artifact_dir)
    media = artifact_dir / "media"
    frames = media / "frames"
    mp4 = media / "map_timelapse_10x.mp4"
    result = {
        "had_frames": frames.is_dir(),
        "encoded": False,
        "deleted_frames": False,
        "mp4_bytes": mp4.stat().st_size if mp4.is_file() else 0,
    }
    if not frames.is_dir():
        return result

    frame_count = len(list(frames.glob("frame_*.jpg")))
    result["frame_count"] = frame_count
    need_encode = try_encode and frame_count > 0 and (
        not mp4.is_file() or mp4.stat().st_size < 64
    )

    if need_encode and container_run_dir:
        frames_c = f"{container_run_dir}/media/frames"
        mp4_c = f"{container_run_dir}/media/map_timelapse_10x.mp4"
        log_c = f"{container_run_dir}/logs/ffmpeg_encode.log"
        py = (
            "import sys; sys.path.insert(0, '/workspace/scripts'); "
            "from pathlib import Path; "
            "from experiment_media_recorder import encode_frame_dir_to_mp4; "
            f"frames = Path({frames_c!r}); mp4 = Path({mp4_c!r}); "
            f"log = Path({log_c!r}); log.parent.mkdir(parents=True, exist_ok=True); "
            "ok = encode_frame_dir_to_mp4(frames, mp4, fps=10, log_path=log); "
            "raise SystemExit(0 if ok else 1)"
        )
        proc = subprocess.run(
            [
                "docker",
                "exec",
                container_name,
                "bash",
                "-lc",
                f"/usr/bin/python3 -c {shlex.quote(py)}",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        result["encoded"] = proc.returncode == 0
        result["encode_rc"] = proc.returncode

    shutil.rmtree(frames, ignore_errors=True)
    result["deleted_frames"] = not frames.exists()
    result["mp4_bytes"] = mp4.stat().st_size if mp4.is_file() else 0
    return result
