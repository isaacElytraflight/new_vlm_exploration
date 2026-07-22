"""Colorize float depth for debugging: flag NaN, zero, saturation, and odd values.

BGR output suitable for OpenCV JPEG encode. Pure NumPy/cv2 — no ROS import.
"""

from __future__ import annotations

from typing import Tuple

import cv2
import numpy as np

# BGR overlays (diagnostic, high-contrast — not a pretty aesthetic).
COLOR_NAN = (255, 0, 255)  # magenta
COLOR_NEG = (0, 140, 255)  # orange
COLOR_ZERO = (0, 0, 255)  # red
COLOR_NEAR = (0, 0, 120)  # dark red — (0, range_min)
COLOR_BEYOND_HORIZON = (0, 255, 255)  # yellow — past mapping horizon, below sat
COLOR_SAT = (255, 255, 255)  # white — near sensor far

DEFAULT_RANGE_MIN = 0.1
DEFAULT_RANGE_MAX = 10.0
DEFAULT_SENSOR_FAR = 50.0
DEFAULT_SAT_EPS = 0.5
LEGEND_HEIGHT = 28


def classify_depth_masks(
    depth: np.ndarray,
    *,
    range_min: float = DEFAULT_RANGE_MIN,
    range_max: float = DEFAULT_RANGE_MAX,
    sensor_far: float = DEFAULT_SENSOR_FAR,
    sat_eps: float = DEFAULT_SAT_EPS,
) -> dict[str, np.ndarray]:
    """Boolean masks for each diagnostic class (mutually exclusive priority order)."""
    d = np.asarray(depth, dtype=np.float32)
    finite = np.isfinite(d)
    sat_lo = float(sensor_far) - max(0.0, float(sat_eps))

    is_nan = ~finite
    is_neg = finite & (d < 0.0)
    is_zero = finite & (d == 0.0)
    is_near = finite & (d > 0.0) & (d < range_min)
    is_sat = finite & (d >= sat_lo)
    is_beyond = finite & (d > range_max) & (d < sat_lo)
    is_valid = finite & (d >= range_min) & (d <= range_max)

    return {
        "nan": is_nan,
        "neg": is_neg,
        "zero": is_zero,
        "near": is_near,
        "valid": is_valid,
        "beyond": is_beyond,
        "sat": is_sat,
    }


def depth_to_debug_bgr(
    depth: np.ndarray,
    *,
    range_min: float = DEFAULT_RANGE_MIN,
    range_max: float = DEFAULT_RANGE_MAX,
    sensor_far: float = DEFAULT_SENSOR_FAR,
    sat_eps: float = DEFAULT_SAT_EPS,
    with_legend: bool = True,
) -> np.ndarray:
    """Convert HxW float depth to HxW (or H+legend) BGR uint8 debug image."""
    if depth.ndim != 2:
        raise ValueError("depth must be 2-D float image")
    h, w = depth.shape
    masks = classify_depth_masks(
        depth,
        range_min=range_min,
        range_max=range_max,
        sensor_far=sensor_far,
        sat_eps=sat_eps,
    )

    # Base: TURBO on valid meters scaled to [range_min, range_max].
    out = np.zeros((h, w, 3), dtype=np.uint8)
    valid = masks["valid"]
    if np.any(valid):
        span = max(float(range_max) - float(range_min), 1e-6)
        norm = np.zeros((h, w), dtype=np.float32)
        norm[valid] = (depth[valid] - range_min) / span
        norm = np.clip(norm, 0.0, 1.0)
        u8 = (norm * 255.0).astype(np.uint8)
        turbo = cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)
        out[valid] = turbo[valid]

    out[masks["beyond"]] = COLOR_BEYOND_HORIZON
    out[masks["sat"]] = COLOR_SAT
    out[masks["near"]] = COLOR_NEAR
    out[masks["zero"]] = COLOR_ZERO
    out[masks["neg"]] = COLOR_NEG
    out[masks["nan"]] = COLOR_NAN

    if not with_legend:
        return out
    return _append_legend(out, range_min=range_min, range_max=range_max, sensor_far=sensor_far)


def _append_legend(
    bgr: np.ndarray,
    *,
    range_min: float,
    range_max: float,
    sensor_far: float,
) -> np.ndarray:
    h, w, _ = bgr.shape
    bar = np.zeros((LEGEND_HEIGHT, w, 3), dtype=np.uint8)
    swatches: list[Tuple[tuple[int, int, int], str]] = [
        (COLOR_NAN, "NaN"),
        (COLOR_ZERO, "0"),
        (COLOR_NEAR, f"<{range_min:g}"),
        ((180, 180, 180), f"{range_min:g}-{range_max:g}m turbo"),
        (COLOR_BEYOND_HORIZON, f">{range_max:g}"),
        (COLOR_SAT, f"sat~{sensor_far:g}"),
        (COLOR_NEG, "neg"),
    ]
    n = len(swatches)
    for i, (color, label) in enumerate(swatches):
        x0 = int(i * w / n)
        x1 = int((i + 1) * w / n)
        bar[:, x0:x1] = color
        # Dark text on light swatches, light text on dark.
        luminance = 0.114 * color[0] + 0.587 * color[1] + 0.299 * color[2]
        text_color = (0, 0, 0) if luminance > 140 else (255, 255, 255)
        cv2.putText(
            bar,
            label,
            (x0 + 4, LEGEND_HEIGHT - 8),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.35,
            text_color,
            1,
            cv2.LINE_AA,
        )
    return np.vstack([bgr, bar])


def depth_debug_counts(
    depth: np.ndarray,
    *,
    range_min: float = DEFAULT_RANGE_MIN,
    range_max: float = DEFAULT_RANGE_MAX,
    sensor_far: float = DEFAULT_SENSOR_FAR,
    sat_eps: float = DEFAULT_SAT_EPS,
) -> dict[str, int]:
    masks = classify_depth_masks(
        depth,
        range_min=range_min,
        range_max=range_max,
        sensor_far=sensor_far,
        sat_eps=sat_eps,
    )
    return {k: int(np.count_nonzero(v)) for k, v in masks.items()}
