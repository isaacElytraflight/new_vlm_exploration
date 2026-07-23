"""Guards: DFS / nearest-parent are Elytra exploration-policy toggles (not dual buttons)."""

from __future__ import annotations

from pathlib import Path

import pytest

PROJECT_YAML = Path(__file__).resolve().parents[2] / "project.yaml"
SCRIPT = Path(__file__).resolve().parent / "set_dfs_order.sh"


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


@pytest.mark.skipif(not PROJECT_YAML.is_file(), reason="project.yaml not available")
def test_no_dual_dfs_oneshot_buttons_negative():
    yaml = pytest.importorskip("yaml")
    data = yaml.safe_load(PROJECT_YAML.read_text(encoding="utf-8"))
    ids = {b["id"] for b in data["buttons"]}
    assert "dfs-highest" not in ids
    assert "dfs-lowest" not in ids


def test_set_dfs_order_script_exists_positive():
    assert SCRIPT.is_file()
    text = SCRIPT.read_text(encoding="utf-8")
    assert "dfs_prefer_highest_openness" in text
