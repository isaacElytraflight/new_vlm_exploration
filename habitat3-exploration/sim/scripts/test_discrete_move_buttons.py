"""Guards: discrete teleop buttons use kind=teleop (fast Elytra path)."""

from __future__ import annotations

from pathlib import Path

import pytest

PROJECT_YAML = Path(__file__).resolve().parents[2] / "project.yaml"

EXPECTED = {
    "step-forward": "forward",
    "turn-left": "turn_left",
    "turn-right": "turn_right",
    "step-backward": "backward",
}


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2


@pytest.mark.skipif(not PROJECT_YAML.is_file(), reason="project.yaml not available in this environment")
def test_project_buttons_are_teleop_kind_positive():
    yaml = pytest.importorskip("yaml")
    data = yaml.safe_load(PROJECT_YAML.read_text(encoding="utf-8"))
    buttons = {b["id"]: b for b in data["buttons"]}
    for button_id, direction in EXPECTED.items():
        assert button_id in buttons, f"missing button {button_id}"
        btn = buttons[button_id]
        assert btn.get("kind") == "teleop"
        assert btn.get("teleopDirection") == direction
        assert "scriptPath" not in btn or not btn.get("oneshot")


def test_teleop_step_script_still_exists_positive():
    """Socket helper remains for CLI/debug even if UI uses kind=teleop."""
    script = Path(__file__).resolve().parent / "teleop_step.sh"
    assert script.is_file()
    assert "elytra_teleop.sock" in script.read_text(encoding="utf-8")
