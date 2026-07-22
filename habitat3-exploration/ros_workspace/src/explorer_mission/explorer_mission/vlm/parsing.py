"""VLM response parsing and image helpers."""

from __future__ import annotations

import base64
import io
import json
import re
from dataclasses import dataclass

from sensor_msgs.msg import CompressedImage

try:
    from PIL import Image
except ImportError:  # pragma: no cover - optional in minimal test envs
    Image = None  # type: ignore[assignment,misc]


def parse_leading_int(s: str) -> int:
    """Return the first integer found in a VLM response string."""
    match = re.search(r"(-?\d+)", s)
    if not match:
        raise ValueError(f"No integer found in VLM response: {s!r}")
    return int(match.group(1))


def _clamp_openness(score: int) -> int:
    return max(0, min(5, int(score)))


def _score_from_json_obj(payload: object) -> int | None:
    if isinstance(payload, dict) and "score" in payload:
        return _clamp_openness(int(payload["score"]))
    return None


def parse_openness_score(s: str) -> int:
    """Parse a 0-5 openness score from JSON or plain text VLM output.

    Tolerates truncated JSON (common when the local VLM hits ``num_predict``)
    by extracting a ``"score": N`` field with a regex before falling back to
    the first integer in the string.
    """
    return parse_openness_result(s).score


def parse_openness_reasoning(s: str) -> str:
    """Extract reasoning text from a VLM openness reply (may be empty)."""
    return parse_openness_result(s).reasoning


@dataclass(frozen=True)
class OpennessResult:
    score: int
    reasoning: str = ""


def _reasoning_from_json_obj(payload: object) -> str:
    if isinstance(payload, dict) and "reasoning" in payload:
        return str(payload["reasoning"]).strip()
    return ""


def parse_openness_result(s: str) -> OpennessResult:
    """Parse score + optional reasoning from JSON or plain text VLM output."""
    text = s.strip()
    if not text:
        raise ValueError("Empty VLM response")

    score: int | None = None
    reasoning = ""

    try:
        payload = json.loads(text)
        score = _score_from_json_obj(payload)
        reasoning = _reasoning_from_json_obj(payload)
    except (json.JSONDecodeError, TypeError, ValueError):
        payload = None

    if score is None:
        match = re.search(r'"score"\s*:\s*(-?\d+)', text, flags=re.IGNORECASE)
        if match:
            score = _clamp_openness(int(match.group(1)))
        else:
            score = _clamp_openness(parse_leading_int(text))

    if not reasoning:
        # Truncated JSON: "reasoning": "partial sentence...
        rm = re.search(
            r'"reasoning"\s*:\s*"((?:\\.|[^"\\])*)',
            text,
            flags=re.IGNORECASE,
        )
        if rm:
            reasoning = rm.group(1).replace('\\"', '"').strip()

    return OpennessResult(score=int(score), reasoning=reasoning)


def validate_frontier_choice(chosen: int, candidate_ids: list[int]) -> int:
    """Return a valid frontier id from VLM output and the candidate id list."""
    if not candidate_ids:
        raise ValueError("No frontier candidates provided")
    valid = {int(fid) for fid in candidate_ids}
    if chosen in valid:
        return chosen
    if 0 <= chosen < len(candidate_ids):
        return int(candidate_ids[chosen])
    raise ValueError(f"VLM chose frontier {chosen}, not in candidates {candidate_ids}")


def _mime_from_format(fmt: str) -> str:
    normalized = fmt.lower().strip()
    if "png" in normalized:
        return "image/png"
    return "image/jpeg"


def decode_compressed_image(msg: CompressedImage) -> bytes:
    return bytes(msg.data)


def downscale_image_bytes(raw: bytes, max_edge: int) -> bytes:
    """Downscale image bytes so the longest edge is at most max_edge."""
    if max_edge <= 0 or Image is None:
        return raw

    with Image.open(io.BytesIO(raw)) as img:
        img = img.convert("RGB")
        width, height = img.size
        longest = max(width, height)
        if longest <= max_edge:
            out = io.BytesIO()
            img.save(out, format="JPEG", quality=85)
            return out.getvalue()

        scale = max_edge / float(longest)
        resized = img.resize(
            (max(1, int(round(width * scale))), max(1, int(round(height * scale)))),
            Image.Resampling.LANCZOS,
        )
        out = io.BytesIO()
        resized.save(out, format="JPEG", quality=85)
        return out.getvalue()


def compressed_to_base64_jpeg(msg: CompressedImage, max_edge: int = 512) -> str:
    """Decode CompressedImage, optionally downscale, return base64 JPEG."""
    raw = decode_compressed_image(msg)
    jpeg = downscale_image_bytes(raw, max_edge)
    return base64.b64encode(jpeg).decode("ascii")


def compressed_to_part(msg: CompressedImage, caption: str) -> tuple[dict, dict]:
    """Convert CompressedImage to Gemini inline_data + caption parts."""
    mime = _mime_from_format(msg.format)
    raw = bytes(msg.data)
    b64 = base64.b64encode(raw).decode("ascii")
    return {"inline_data": {"mime_type": mime, "data": b64}}, {"text": caption}
