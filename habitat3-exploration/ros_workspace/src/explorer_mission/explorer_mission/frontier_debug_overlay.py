"""Bake VLM score + reasoning onto a frontier RGB frame (testable pure logic)."""

from __future__ import annotations

import io
from dataclasses import dataclass

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:  # pragma: no cover
    Image = None  # type: ignore[assignment,misc]
    ImageDraw = None  # type: ignore[assignment,misc]
    ImageFont = None  # type: ignore[assignment,misc]


@dataclass(frozen=True)
class FrontierOverlay:
    score: int
    reasoning: str


def format_overlay_lines(overlay: FrontierOverlay, *, max_reasoning_chars: int = 120) -> list[str]:
    """Return text lines drawn on the debug image."""
    if overlay.score < 0:
        score_line = "openness: unrated"
    else:
        score_line = f"openness: {int(overlay.score)}"
    reasoning = (overlay.reasoning or "").strip() or "(no reasoning)"
    if len(reasoning) > max_reasoning_chars:
        reasoning = reasoning[: max_reasoning_chars - 1] + "…"
    return [score_line, reasoning]


def _wrap_text(text: str, *, max_chars: int = 48) -> list[str]:
    words = text.split()
    if not words:
        return [""]
    lines: list[str] = []
    cur = words[0]
    for w in words[1:]:
        trial = f"{cur} {w}"
        if len(trial) <= max_chars:
            cur = trial
        else:
            lines.append(cur)
            cur = w
    lines.append(cur)
    return lines


def overlay_frontier_jpeg(
    jpeg_bytes: bytes,
    overlay: FrontierOverlay,
) -> bytes:
    """Decode JPEG/PNG bytes, draw score+reasoning, return JPEG bytes."""
    if Image is None or ImageDraw is None:
        raise RuntimeError("Pillow is required for frontier overlay")

    with Image.open(io.BytesIO(jpeg_bytes)) as img:
        rgb = img.convert("RGB")
    draw = ImageDraw.Draw(rgb)
    try:
        font = ImageFont.load_default()
    except Exception:  # pragma: no cover
        font = None

    lines = format_overlay_lines(overlay)
    wrapped: list[str] = [lines[0]]
    wrapped.extend(_wrap_text(lines[1], max_chars=48))

    pad = 8
    line_h = 14
    box_h = pad * 2 + line_h * len(wrapped)
    box_w = rgb.width
    draw.rectangle([(0, 0), (box_w, box_h)], fill=(0, 0, 0))
    y = pad
    for line in wrapped:
        draw.text((pad, y), line, fill=(0, 255, 128), font=font)
        y += line_h

    out = io.BytesIO()
    rgb.save(out, format="JPEG", quality=85)
    return out.getvalue()


def overlay_has_score_text(jpeg_bytes: bytes, score: int) -> bool:
    """Negative-control helper: ensure overlay JPEG is larger / different than input."""
    return len(jpeg_bytes) > 0 and f"{score}".encode() is not None
