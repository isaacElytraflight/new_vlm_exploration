"""Tests for frontier VLM debug overlay."""

from __future__ import annotations

import io

import pytest

from explorer_mission.frontier_debug_overlay import (
    FrontierOverlay,
    format_overlay_lines,
    overlay_frontier_jpeg,
)

try:
    from PIL import Image

    HAS_PIL = True
except ImportError:
    HAS_PIL = False


def _jpeg(color=(40, 80, 120)) -> bytes:
    if not HAS_PIL:
        pytest.skip("Pillow required")
    buf = io.BytesIO()
    Image.new("RGB", (320, 240), color=color).save(buf, format="JPEG")
    return buf.getvalue()


def test_harness_positive():
    assert 1 + 1 == 2


def test_harness_negative():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_format_overlay_lines_positive():
    lines = format_overlay_lines(FrontierOverlay(score=4, reasoning="open atrium"))
    assert lines[0] == "openness: 4"
    assert "atrium" in lines[1]


def test_format_overlay_empty_reasoning_negative():
    lines = format_overlay_lines(FrontierOverlay(score=0, reasoning=""))
    assert lines[0] == "openness: 0"
    assert lines[1] == "(no reasoning)"


def test_overlay_frontier_jpeg_changes_bytes_positive():
    raw = _jpeg()
    out = overlay_frontier_jpeg(raw, FrontierOverlay(score=3, reasoning="hallway turn"))
    assert isinstance(out, (bytes, bytearray))
    assert len(out) > 0
    assert out != raw


def test_overlay_frontier_jpeg_bad_input_negative():
    with pytest.raises(Exception):
        overlay_frontier_jpeg(b"not-an-image", FrontierOverlay(score=1, reasoning="x"))
