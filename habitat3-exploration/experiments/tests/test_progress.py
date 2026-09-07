from __future__ import annotations

import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from experiments.progress import ProgressWriter  # noqa: E402


def test_progress_writer_roundtrip_positive(tmp_path: Path):
    path = tmp_path / ".ablation_progress.json"
    writer = ProgressWriter(path)
    writer.update(
        experiment_id="exp",
        count=2,
        total=4,
        step="collecting_metrics",
        detail="Run 2/4",
        run_id="run2",
    )
    data = ProgressWriter(path).read()
    assert data["count"] == 2
    assert data["total"] == 4
    assert data["percent"] == 50
    assert data["step"] == "collecting_metrics"


def test_cancel_requested_positive(tmp_path: Path):
    writer = ProgressWriter(tmp_path / "p.json")
    assert writer.cancel_requested() is False
    writer.request_cancel()
    assert writer.cancel_requested() is True


def test_progress_missing_file_negative(tmp_path: Path):
    writer = ProgressWriter(tmp_path / "missing.json")
    data = writer.read()
    assert data["step"] == "Idle"
    assert data["complete"] is False


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
