"""VLM response parsing helpers."""

from __future__ import annotations

import base64
import re

from sensor_msgs.msg import CompressedImage


def parse_leading_int(s: str) -> int:
    """Return the first integer found in a VLM response string."""
    match = re.search(r"(-?\d+)", s)
    if not match:
        raise ValueError(f"No integer found in VLM response: {s!r}")
    return int(match.group(1))


def compressed_to_part(msg: CompressedImage, caption: str) -> tuple[dict, dict]:
    """Convert CompressedImage to Gemini inline_data + caption parts."""
    fmt = msg.format.lower().strip()
    if fmt in ("png", "png; png"):
        mime = "image/png"
    elif fmt in ("jpeg", "jpg", "jpeg; jpeg", "jpg; jpg"):
        mime = "image/jpeg"
    else:
        mime = "image/jpeg"
    raw = bytes(msg.data)
    b64 = base64.b64encode(raw).decode("ascii")
    return {"inline_data": {"mime_type": mime, "data": b64}}, {"text": caption}
