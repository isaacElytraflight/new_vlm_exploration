#!/usr/bin/env python3
"""Record side-by-side grid|plan map timelapse at 1 Hz → 10× MP4.

Subscribes to CompressedImage topics from maprender_node, stitches frames,
encodes with ffmpeg at 10 fps (10× wall-clock), and writes final PNGs.

Encoding strategy: write numbered JPEGs during the episode, then run a single
ffmpeg pass at finalize. This avoids truncated/empty MP4s from a killed pipe
(missing moov atom) and keeps libx264 happy via even frame dimensions.
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np

try:
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None  # type: ignore


def decode_jpeg(data: bytes):
    if cv2 is None:
        return None
    arr = np.frombuffer(data, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    return img


def downscale(img: np.ndarray, max_width: int) -> np.ndarray:
    h, w = img.shape[:2]
    if w <= max_width:
        return img
    scale = max_width / float(w)
    return cv2.resize(img, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)


def ensure_even_dims(img: np.ndarray) -> np.ndarray:
    """libx264 yuv420p requires even width and height."""
    h, w = img.shape[:2]
    pad_h = h % 2
    pad_w = w % 2
    if pad_h == 0 and pad_w == 0:
        return img
    return cv2.copyMakeBorder(
        img, 0, pad_h, 0, pad_w, cv2.BORDER_CONSTANT, value=(20, 20, 20)
    )


def stitch_side_by_side(
    left: Optional[np.ndarray],
    right: Optional[np.ndarray],
    *,
    max_width: int,
    grid_only: bool,
) -> Optional[np.ndarray]:
    if left is None and right is None:
        return None
    if grid_only or right is None:
        if left is None:
            return None
        return ensure_even_dims(downscale(left, max_width))
    if left is None:
        return ensure_even_dims(downscale(right, max_width))

    left = downscale(left, max_width // 2)
    right = downscale(right, max_width // 2)
    h = max(left.shape[0], right.shape[0])

    def pad(img: np.ndarray) -> np.ndarray:
        if img.shape[0] == h:
            return img
        pad_h = h - img.shape[0]
        return cv2.copyMakeBorder(img, 0, pad_h, 0, 0, cv2.BORDER_CONSTANT, value=(20, 20, 20))

    return ensure_even_dims(np.hstack([pad(left), pad(right)]))


def encode_frame_dir_to_mp4(
    frame_dir: Path,
    out_mp4: Path,
    *,
    fps: int = 10,
    log_path: Optional[Path] = None,
) -> bool:
    """Encode frame_000001.jpg… into a playable H.264 MP4. Returns True on success."""
    frame_dir = Path(frame_dir)
    out_mp4 = Path(out_mp4)
    frames = sorted(frame_dir.glob("frame_*.jpg"))
    if not frames:
        return False
    if shutil.which("ffmpeg") is None:
        return False

    out_mp4.parent.mkdir(parents=True, exist_ok=True)
    # Must keep a .mp4 suffix — ffmpeg rejects ".mp4.tmp" as unknown muxer.
    tmp = out_mp4.with_name(out_mp4.stem + ".encoding.mp4")
    if tmp.exists():
        tmp.unlink()

    pattern = str(frame_dir / "frame_%06d.jpg")
    cmd = [
        "ffmpeg",
        "-y",
        "-framerate",
        str(int(fps)),
        "-i",
        pattern,
        "-c:v",
        "libx264",
        "-pix_fmt",
        "yuv420p",
        "-preset",
        "veryfast",
        "-crf",
        "28",
        "-movflags",
        "+faststart",
        "-an",
        str(tmp),
    ]
    log_fh = None
    try:
        if log_path is not None:
            log_path.parent.mkdir(parents=True, exist_ok=True)
            log_fh = log_path.open("ab")
            log_fh.write(f"\n# encode {len(frames)} frames → {out_mp4}\n".encode())
            log_fh.flush()
        proc = subprocess.run(
            cmd,
            stdout=log_fh or subprocess.DEVNULL,
            stderr=log_fh or subprocess.PIPE,
            check=False,
        )
        if proc.returncode != 0:
            if log_fh is None and proc.stderr:
                sys.stderr.write(proc.stderr.decode("utf-8", errors="replace"))
            if tmp.exists():
                tmp.unlink(missing_ok=True)
            return False
        if not tmp.is_file() or tmp.stat().st_size < 64:
            tmp.unlink(missing_ok=True)
            return False
        os.replace(tmp, out_mp4)
        return True
    finally:
        if log_fh is not None:
            log_fh.close()


def _build_media_recorder_cls(Node, CompressedImage):
    class MediaRecorderImpl(Node):
        def __init__(
            self,
            *,
            out_dir: Path,
            capture_hz: float,
            max_width: int,
            jpeg_quality: int,
            grid_only: bool,
            stop_flag_path: Path | None,
        ) -> None:
            super().__init__("experiment_media_recorder")
            self._out_dir = Path(out_dir)
            self._out_dir.mkdir(parents=True, exist_ok=True)
            self._frames_dir = self._out_dir / "frames"
            self._frames_dir.mkdir(parents=True, exist_ok=True)
            self._period = 1.0 / max(0.1, float(capture_hz))
            self._max_width = int(max_width)
            self._jpeg_quality = int(jpeg_quality)
            self._grid_only = bool(grid_only)
            self._stop_flag_path = Path(stop_flag_path) if stop_flag_path else None

            self._latest_grid: Optional[bytes] = None
            self._latest_plan: Optional[bytes] = None
            self._lock = threading.Lock()
            self._last_frame: Optional[np.ndarray] = None
            self._last_grid_png: Optional[np.ndarray] = None
            self._last_plan_png: Optional[np.ndarray] = None
            self._frame_count = 0
            self._stopping = False

            self.create_subscription(
                CompressedImage, "/map_renderer/grid_img", self._grid_cb, 1
            )
            self.create_subscription(
                CompressedImage, "/map_renderer/nav_plan_img", self._plan_cb, 1
            )
            self.create_timer(self._period, self._tick)

        def _grid_cb(self, msg) -> None:
            with self._lock:
                self._latest_grid = bytes(msg.data)

        def _plan_cb(self, msg) -> None:
            with self._lock:
                self._latest_plan = bytes(msg.data)

        def _tick(self) -> None:
            if self._stopping:
                return
            if self._stop_flag_path and self._stop_flag_path.is_file():
                self._stopping = True
                raise SystemExit(0)

            with self._lock:
                grid_bytes = self._latest_grid
                plan_bytes = self._latest_plan

            grid = decode_jpeg(grid_bytes) if grid_bytes else None
            plan = decode_jpeg(plan_bytes) if plan_bytes else None
            if grid is not None:
                self._last_grid_png = grid
            if plan is not None:
                self._last_plan_png = plan

            frame = stitch_side_by_side(
                grid, plan, max_width=self._max_width, grid_only=self._grid_only
            )
            if frame is None or cv2 is None:
                return

            self._last_frame = frame
            ok, enc = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
            )
            if not ok:
                return

            self._frame_count += 1
            path = self._frames_dir / f"frame_{self._frame_count:06d}.jpg"
            path.write_bytes(enc.tobytes())

        def finalize(self) -> None:
            final_grid = self._out_dir / "final_grid_map.png"
            final_plan = self._out_dir / "final_nav_plan.png"
            mp4_out = self._out_dir / "map_timelapse_10x.mp4"
            ffmpeg_log = self._out_dir.parent / "logs" / "ffmpeg_encode.log"
            alt_log = self._out_dir / "ffmpeg_encode.log"
            log_path = ffmpeg_log if (self._out_dir.parent / "logs").is_dir() else alt_log

            if self._last_grid_png is not None and cv2 is not None:
                cv2.imwrite(str(final_grid), self._last_grid_png)
            if self._last_plan_png is not None and cv2 is not None:
                cv2.imwrite(str(final_plan), self._last_plan_png)
            elif self._last_frame is not None and cv2 is not None and not final_plan.exists():
                cv2.imwrite(str(final_plan), self._last_frame)

            ok = encode_frame_dir_to_mp4(
                self._frames_dir,
                mp4_out,
                fps=10,
                log_path=log_path,
            )
            if ok:
                shutil.rmtree(self._frames_dir, ignore_errors=True)
                size = mp4_out.stat().st_size if mp4_out.is_file() else 0
                self.get_logger().info(
                    f"media recorder done frames={self._frame_count} mp4_bytes={size} out={self._out_dir}"
                )
            else:
                if mp4_out.exists() and mp4_out.stat().st_size < 64:
                    mp4_out.unlink(missing_ok=True)
                self.get_logger().error(
                    f"media recorder encode FAILED frames={self._frame_count} "
                    f"kept_frames_dir={self._frames_dir} log={log_path}"
                )

    return MediaRecorderImpl


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Record map timelapse media for a run.")
    parser.add_argument("--out-dir", type=Path, required=True)
    parser.add_argument("--capture-hz", type=float, default=1.0)
    parser.add_argument("--max-width", type=int, default=800)
    parser.add_argument("--jpeg-quality", type=int, default=60)
    parser.add_argument(
        "--grid-only",
        action="store_true",
        help="Fallback: encode grid strip only (smaller package)",
    )
    parser.add_argument(
        "--stop-flag",
        type=Path,
        default=None,
        help="When this file appears, finalize and exit",
    )
    parser.add_argument(
        "--max-runtime-s",
        type=float,
        default=0.0,
        help="Optional hard runtime limit (0 = unlimited)",
    )
    args = parser.parse_args(argv)

    if cv2 is None:
        print("opencv-python required", file=sys.stderr)
        return 2

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CompressedImage

    MediaRecorderCls = _build_media_recorder_cls(Node, CompressedImage)

    rclpy.init()
    node = MediaRecorderCls(
        out_dir=args.out_dir,
        capture_hz=args.capture_hz,
        max_width=args.max_width,
        jpeg_quality=args.jpeg_quality,
        grid_only=args.grid_only,
        stop_flag_path=args.stop_flag,
    )
    started = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if args.stop_flag and Path(args.stop_flag).is_file():
                break
            if args.max_runtime_s > 0 and (time.monotonic() - started) >= args.max_runtime_s:
                break
    except SystemExit:
        pass
    finally:
        node.finalize()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
