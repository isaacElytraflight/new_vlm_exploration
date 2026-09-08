"""Tests for run media cleanup (frames → encode-or-delete)."""

from __future__ import annotations

from pathlib import Path

import pytest

from experiments.media_cleanup import cleanup_run_media


def test_harness_positive_control():
    assert 1 + 1 == 2


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_cleanup_deletes_frames_without_encode_when_mp4_ok_positive(tmp_path: Path):
    media = tmp_path / "media"
    frames = media / "frames"
    frames.mkdir(parents=True)
    (frames / "frame_000001.jpg").write_bytes(b"fake")
    mp4 = media / "map_timelapse_10x.mp4"
    mp4.write_bytes(b"x" * 128)

    result = cleanup_run_media(tmp_path, try_encode=True, container_run_dir=None)
    assert result["had_frames"] is True
    assert result["deleted_frames"] is True
    assert not frames.exists()
    assert mp4.is_file()


def test_cleanup_no_frames_is_noop_negative(tmp_path: Path):
    media = tmp_path / "media"
    media.mkdir()
    result = cleanup_run_media(tmp_path, try_encode=True)
    assert result["had_frames"] is False
    assert result["deleted_frames"] is False


def test_cleanup_try_encode_false_still_deletes_frames_positive(tmp_path: Path):
    frames = tmp_path / "media" / "frames"
    frames.mkdir(parents=True)
    (frames / "frame_000001.jpg").write_bytes(b"fake")
    result = cleanup_run_media(tmp_path, try_encode=False, container_run_dir=None)
    assert result["encoded"] is False
    assert result["deleted_frames"] is True
    assert not frames.exists()
