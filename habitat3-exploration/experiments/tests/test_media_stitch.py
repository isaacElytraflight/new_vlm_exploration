"""Unit tests for media stitch helpers and ffmpeg encode (no ROS required)."""

from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

import pytest

_SCRIPTS = Path(__file__).resolve().parents[2] / "sim" / "scripts"
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))


np = pytest.importorskip("numpy")
cv2 = pytest.importorskip("cv2")

from experiment_media_recorder import (  # noqa: E402
    encode_frame_dir_to_mp4,
    ensure_even_dims,
    stitch_side_by_side,
)


def test_harness_positive_control():
    assert 1 + 1 == 2


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_stitch_side_by_side_positive():
    left = np.zeros((40, 60, 3), dtype=np.uint8)
    right = np.ones((40, 60, 3), dtype=np.uint8) * 255
    out = stitch_side_by_side(left, right, max_width=800, grid_only=False)
    assert out is not None
    assert out.shape[1] >= left.shape[1]
    assert out.shape[0] % 2 == 0
    assert out.shape[1] % 2 == 0


def test_stitch_odd_height_padded_positive():
    """Regression: odd map heights previously broke libx264 yuv420p."""
    left = np.zeros((41, 61, 3), dtype=np.uint8)
    right = np.ones((39, 59, 3), dtype=np.uint8) * 255
    out = stitch_side_by_side(left, right, max_width=800, grid_only=False)
    assert out is not None
    assert out.shape[0] % 2 == 0
    assert out.shape[1] % 2 == 0


def test_ensure_even_dims_negative_input_odd():
    img = np.zeros((41, 60, 3), dtype=np.uint8)
    out = ensure_even_dims(img)
    assert out.shape == (42, 60, 3)


def test_stitch_grid_only_when_plan_missing_negative():
    left = np.zeros((40, 60, 3), dtype=np.uint8)
    out = stitch_side_by_side(left, None, max_width=800, grid_only=False)
    assert out is not None
    assert out.shape[0] % 2 == 0
    assert out.shape[1] % 2 == 0


def test_stitch_both_missing_negative():
    assert stitch_side_by_side(None, None, max_width=800, grid_only=False) is None


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg not installed")
def test_encode_frame_dir_to_mp4_positive(tmp_path: Path):
    frame_dir = tmp_path / "frames"
    frame_dir.mkdir()
    frame = np.zeros((48, 64, 3), dtype=np.uint8)
    frame[:, :] = (40, 120, 200)
    for i in range(1, 6):
        ok, enc = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        assert ok
        (frame_dir / f"frame_{i:06d}.jpg").write_bytes(enc.tobytes())

    out_mp4 = tmp_path / "map_timelapse_10x.mp4"
    log = tmp_path / "ffmpeg.log"
    assert encode_frame_dir_to_mp4(frame_dir, out_mp4, fps=10, log_path=log) is True
    assert out_mp4.is_file()
    assert out_mp4.stat().st_size > 64
    # Probe when ffprobe available (container/host).
    if shutil.which("ffprobe"):
        proc = subprocess.run(
            [
                "ffprobe",
                "-v",
                "error",
                "-show_entries",
                "format=duration",
                "-of",
                "default=nw=1",
                str(out_mp4),
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        assert proc.returncode == 0
        assert "duration=" in proc.stdout


@pytest.mark.skipif(shutil.which("ffmpeg") is None, reason="ffmpeg not installed")
def test_encode_empty_frame_dir_negative(tmp_path: Path):
    frame_dir = tmp_path / "frames"
    frame_dir.mkdir()
    out_mp4 = tmp_path / "out.mp4"
    assert encode_frame_dir_to_mp4(frame_dir, out_mp4, fps=10) is False
    assert not out_mp4.exists()
