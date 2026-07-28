"""Pure helpers for coverage-vs-distance metrics (no ROS / Habitat deps)."""

from __future__ import annotations

import io
import math
from collections import deque
from typing import Deque, List, Optional, Sequence, Tuple

import numpy as np

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:  # pragma: no cover
    Image = None  # type: ignore
    ImageDraw = None  # type: ignore
    ImageFont = None  # type: ignore


def integrate_path_meters(
    meters_so_far: float,
    prev_xy: Optional[Tuple[float, float]],
    x: float,
    y: float,
) -> Tuple[float, Tuple[float, float]]:
    """Accumulate planar path length via hypot(dx, dy). Yaw-only does not add."""
    if prev_xy is None:
        return float(meters_so_far), (float(x), float(y))
    dx = float(x) - prev_xy[0]
    dy = float(y) - prev_xy[1]
    return float(meters_so_far) + math.hypot(dx, dy), (float(x), float(y))


def free_area_m2(grid: np.ndarray, resolution: float) -> float:
    """Explored free area only: count(grid == 0) * resolution²."""
    if grid is None or getattr(grid, "size", 0) == 0:
        return 0.0
    res = float(resolution)
    if res <= 0.0:
        return 0.0
    free_cells = int(np.count_nonzero(np.asarray(grid) == 0))
    return free_cells * (res * res)


def mapped_area_m2(grid: np.ndarray, resolution: float) -> float:
    """Known mapped area: free (0) + occupied/walls (100), excluding unknown (-1).

    Including walls keeps coverage from dropping when free cells are later
    reclassified as occupied.
    """
    if grid is None or getattr(grid, "size", 0) == 0:
        return 0.0
    res = float(resolution)
    if res <= 0.0:
        return 0.0
    arr = np.asarray(grid)
    known = int(np.count_nonzero((arr == 0) | (arr == 100)))
    return known * (res * res)


def coverage_ratio(mapped_m2: float, gt_m2: float) -> float:
    """mapped / GT mappable; 0 when gt <= 0. Clamped to [0, 1]."""
    if float(gt_m2) <= 0.0:
        return 0.0
    return max(0.0, min(1.0, float(mapped_m2) / float(gt_m2)))


def adjacent_wall_mask(navigable: np.ndarray) -> np.ndarray:
    """Non-navigable cells in the 8-neighbourhood of any navigable cell (walls)."""
    nav = np.asarray(navigable, dtype=bool)
    h, w = nav.shape
    dilated = np.zeros((h, w), dtype=bool)
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            r0, r1 = max(0, dr), h + min(0, dr)
            c0, c1 = max(0, dc), w + min(0, dc)
            dilated[r0:r1, c0:c1] |= nav[r0 - dr : r1 - dr, c0 - dc : c1 - dc]
    return dilated & (~nav)


def floor_area_m2_from_navigable(navigable: np.ndarray, mpp: float) -> float:
    """GT floor-only area: navigable cell count * meters_per_pixel²."""
    if navigable is None or getattr(navigable, "size", 0) == 0:
        return 0.0
    m = float(mpp)
    if m <= 0.0:
        return 0.0
    return float(np.count_nonzero(np.asarray(navigable, dtype=bool))) * (m * m)


def gt_mappable_area_m2(navigable: np.ndarray, mpp: float) -> float:
    """GT mappable area: navigable floor + adjacent wall cells × mpp²."""
    if navigable is None or getattr(navigable, "size", 0) == 0:
        return 0.0
    m = float(mpp)
    if m <= 0.0:
        return 0.0
    nav = np.asarray(navigable, dtype=bool)
    walls = adjacent_wall_mask(nav)
    return float(np.count_nonzero(nav | walls)) * (m * m)


class CoverageSampleRing:
    """Fixed-capacity ring buffer of (meters, coverage) samples."""

    def __init__(self, maxlen: int = 2048) -> None:
        if maxlen <= 0:
            raise ValueError("maxlen must be positive")
        self._buf: Deque[Tuple[float, float]] = deque(maxlen=int(maxlen))

    def append(self, meters: float, coverage: float) -> None:
        self._buf.append((float(meters), float(coverage)))

    def as_series(self) -> Tuple[List[float], List[float]]:
        if not self._buf:
            return [], []
        xs, ys = zip(*self._buf)
        return list(xs), list(ys)


def _draw_axis_label_vertical(
    img: "Image.Image",
    text: str,
    *,
    x: int,
    y_center: int,
    fill: Tuple[int, int, int],
    font,
) -> None:
    """Draw rotated text for the Y-axis label."""
    if Image is None or ImageDraw is None:
        return
    # Measure on a throwaway image, then rotate.
    probe = Image.new("RGBA", (8, 8), (0, 0, 0, 0))
    probe_draw = ImageDraw.Draw(probe)
    bbox = probe_draw.textbbox((0, 0), text, font=font)
    tw = max(1, bbox[2] - bbox[0] + 4)
    th = max(1, bbox[3] - bbox[1] + 4)
    label = Image.new("RGBA", (tw, th), (0, 0, 0, 0))
    ImageDraw.Draw(label).text((2, 2), text, fill=fill + (255,), font=font)
    rotated = label.rotate(90, expand=True)
    px = max(0, x - rotated.width // 2)
    py = max(0, y_center - rotated.height // 2)
    img.paste(rotated, (px, py), rotated)


def render_coverage_chart_jpeg(
    meters: Sequence[float],
    coverage: Sequence[float],
    width: int = 640,
    height: int = 360,
) -> bytes:
    """Draw coverage (y) vs distance (x) as a JPEG via PIL."""
    if Image is None or ImageDraw is None:
        raise RuntimeError("Pillow is required to render coverage charts")

    img = Image.new("RGB", (int(width), int(height)), (24, 26, 30))
    draw = ImageDraw.Draw(img)
    # Extra left margin for rotated Y label; bottom for X label + ticks.
    margin_l, margin_r, margin_t, margin_b = 72, 20, 32, 52
    plot_w = max(1, width - margin_l - margin_r)
    plot_h = max(1, height - margin_t - margin_b)
    draw.rectangle(
        [margin_l, margin_t, margin_l + plot_w, margin_t + plot_h],
        outline=(90, 96, 110),
        width=1,
    )
    try:
        font = ImageFont.load_default()
    except Exception:  # pragma: no cover
        font = None

    title = "Coverage vs Distance"
    draw.text((margin_l, 8), title, fill=(220, 220, 230), font=font)

    x_label = "Distance traveled (m)"
    y_label = "Coverage (mapped / GT)"
    # Center X label under the plot.
    xb = draw.textbbox((0, 0), x_label, font=font) if hasattr(draw, "textbbox") else (0, 0, len(x_label) * 6, 10)
    x_label_w = xb[2] - xb[0]
    draw.text(
        (margin_l + max(0, (plot_w - x_label_w) // 2), height - 22),
        x_label,
        fill=(200, 205, 220),
        font=font,
    )
    _draw_axis_label_vertical(
        img,
        y_label,
        x=18,
        y_center=margin_t + plot_h // 2,
        fill=(200, 205, 220),
        font=font,
    )
    # Re-acquire draw after paste (same image).
    draw = ImageDraw.Draw(img)

    if len(meters) >= 2 and len(coverage) >= 2 and len(meters) == len(coverage):
        xs = [float(v) for v in meters]
        ys = [float(v) for v in coverage]
        x0, x1 = min(xs), max(xs)
        y0, y1 = 0.0, max(1e-9, max(ys) if max(ys) > 0 else 1.0)
        if y1 < 1.0:
            y1 = 1.0
        span_x = (x1 - x0) if x1 > x0 else 1.0

        def to_px(mx: float, cy: float) -> Tuple[int, int]:
            px = margin_l + int(round(((mx - x0) / span_x) * plot_w))
            py = margin_t + plot_h - int(round(((cy - y0) / (y1 - y0)) * plot_h))
            return px, py

        # Tick labels
        draw.text((margin_l - 2, margin_t + plot_h + 4), f"{x0:.1f}", fill=(150, 155, 170), font=font)
        x1_label = f"{x1:.1f}"
        x1b = draw.textbbox((0, 0), x1_label, font=font)
        draw.text(
            (margin_l + plot_w - (x1b[2] - x1b[0]), margin_t + plot_h + 4),
            x1_label,
            fill=(150, 155, 170),
            font=font,
        )
        draw.text((margin_l - 28, margin_t - 2), f"{y1:.2f}", fill=(150, 155, 170), font=font)
        draw.text((margin_l - 20, margin_t + plot_h - 10), "0", fill=(150, 155, 170), font=font)

        pts = [to_px(mx, cy) for mx, cy in zip(xs, ys)]
        draw.line(pts, fill=(80, 180, 255), width=2)
        last = pts[-1]
        draw.ellipse([last[0] - 3, last[1] - 3, last[0] + 3, last[1] + 3], fill=(255, 200, 80))
        label = f"{ys[-1]:.2f} @ {xs[-1]:.1f}m"
        draw.text((margin_l + 4, margin_t + 4), label, fill=(200, 205, 220), font=font)
    else:
        draw.text(
            (margin_l + 12, margin_t + plot_h // 2 - 6),
            "waiting for samples…",
            fill=(140, 145, 160),
            font=font,
        )

    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()
