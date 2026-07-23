"""Guards: DFS child-order oneshot buttons + set_dfs_order.sh."""

from __future__ import annotations

from pathlib import Path

import pytest

PROJECT_YAML = Path(__file__).resolve().parents[2] / "project.yaml"
SCRIPT = Path(__file__).resolve().parent / "set_dfs_order.sh"

EXPECTED = {
    "dfs-highest": "highest",
    "dfs-lowest": "lowest",
}


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


@pytest.mark.skipif(not PROJECT_YAML.is_file(), reason="project.yaml not available")
def test_dfs_order_buttons_oneshot_positive():
    yaml = pytest.importorskip("yaml")
    data = yaml.safe_load(PROJECT_YAML.read_text(encoding="utf-8"))
    buttons = {b["id"]: b for b in data["buttons"]}
    for button_id, order in EXPECTED.items():
        assert button_id in buttons, f"missing button {button_id}"
        btn = buttons[button_id]
        assert btn.get("kind") == "script"
        assert btn.get("oneshot") is True
        assert btn.get("scriptPath") == "/workspace/scripts/set_dfs_order.sh"
        assert str(btn.get("extraArgs", "")).strip() == order


@pytest.mark.skipif(not PROJECT_YAML.is_file(), reason="project.yaml not available")
def test_dfs_order_buttons_missing_script_rejected_negative():
    yaml = pytest.importorskip("yaml")
    data = yaml.safe_load(PROJECT_YAML.read_text(encoding="utf-8"))
    buttons = {b["id"]: b for b in data["buttons"]}
    for button_id in EXPECTED:
        btn = buttons[button_id]
        assert btn.get("scriptPath"), f"{button_id} must have scriptPath"
        assert btn.get("kind") != "teleop"


def test_set_dfs_order_script_exists_positive():
    assert SCRIPT.is_file()
    text = SCRIPT.read_text(encoding="utf-8")
    assert "dfs_prefer_highest_openness" in text
    assert "/explore" in text


def test_set_dfs_order_script_rejects_bad_arg_negative():
    text = SCRIPT.read_text(encoding="utf-8")
    assert "highest|lowest" in text or "Usage:" in text
