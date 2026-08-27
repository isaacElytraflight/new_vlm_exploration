"""Render a live exploration state-machine HUD as JPEG (no ROS imports)."""

from __future__ import annotations

from dataclasses import dataclass
from io import BytesIO
from typing import Sequence

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:  # pragma: no cover
    Image = None  # type: ignore
    ImageDraw = None  # type: ignore
    ImageFont = None  # type: ignore

PHASE_COLORS = {
    "scanning": (80, 200, 220),
    "detecting": (230, 200, 80),
    "awaiting_vlm": (210, 120, 220),
    "selecting": (180, 180, 100),
    "navigating": (90, 210, 120),
    "backtracking": (230, 150, 70),
    "complete": (160, 165, 175),
    "idle": (140, 150, 160),
}


@dataclass(frozen=True)
class StatusEvent:
    t_sec: float
    phase: str
    detail: str
    current_node_id: int
    target_node_id: int
    complete: bool


def append_status_event(
    log: list[StatusEvent],
    event: StatusEvent,
    *,
    maxlen: int = 12,
) -> None:
    phase = str(event.phase).strip()
    if not phase:
        return
    log.append(event)
    overflow = len(log) - max(1, int(maxlen))
    if overflow > 0:
        del log[:overflow]


def dwell_seconds(log: Sequence[StatusEvent], *, now_sec: float) -> float:
    if not log:
        return 0.0
    return max(0.0, float(now_sec) - float(log[-1].t_sec))


def spin_hint(phase: str) -> str:
    """One-line discriminator for in-place 360° loops."""
    p = str(phase).strip().lower()
    if p == "scanning":
        return "Spin here is explore-node rotate_360 (state machine scan)."
    if p in ("navigating", "backtracking"):
        return "Spin here is Nav2 (path / recovery), not a new SM scan."
    return ""


def render_status_hud_jpeg(
    log: Sequence[StatusEvent],
    *,
    now_sec: float,
    width: int = 640,
    height: int = 360,
) -> bytes:
    if Image is None or ImageDraw is None:
        raise RuntimeError("Pillow is required to render the status HUD")

    img = Image.new("RGB", (int(width), int(height)), (18, 20, 24))
    draw = ImageDraw.Draw(img)
    try:
        font = ImageFont.load_default()
    except Exception:  # pragma: no cover
        font = None

    draw.text((16, 10), "Exploration state", fill=(200, 205, 220), font=font)

    if not log:
        draw.text((16, 48), "Waiting for /exploration/status …", fill=(160, 165, 175), font=font)
        draw.text(
            (16, 72),
            "No phase yet — stack still starting.",
            fill=(120, 125, 140),
            font=font,
        )
        return _to_jpeg(img)

    last = log[-1]
    color = PHASE_COLORS.get(last.phase, (220, 220, 230))
    draw.text((16, 36), last.phase.upper(), fill=color, font=font)
    dwell = dwell_seconds(log, now_sec=now_sec)
    draw.text((16, 56), f"in this phase  {dwell:0.1f} s", fill=(180, 185, 200), font=font)
    draw.text(
        (16, 74),
        f"node {last.current_node_id}  →  target {last.target_node_id}"
        + ("  COMPLETE" if last.complete else ""),
        fill=(170, 175, 190),
        font=font,
    )
    detail = last.detail.strip() or "(no detail)"
    draw.text((16, 94), detail[:80], fill=(210, 210, 220), font=font)

    hint = spin_hint(last.phase)
    if hint:
        draw.text((16, 114), hint, fill=(140, 190, 210), font=font)

    draw.text((16, 142), "Recent phases (oldest → newest)", fill=(130, 135, 150), font=font)
    y = 160
    for ev in log:
        line = (
            f"+{ev.t_sec:6.1f}s  {ev.phase:<12}  {ev.detail[:48]}"
        )
        fill = PHASE_COLORS.get(ev.phase, (190, 195, 205))
        draw.text((16, y), line, fill=fill, font=font)
        y += 14
        if y > height - 20:
            break

    return _to_jpeg(img)


def _to_jpeg(img: Image.Image) -> bytes:
    buf = BytesIO()
    img.save(buf, format="JPEG", quality=85)
    return buf.getvalue()
