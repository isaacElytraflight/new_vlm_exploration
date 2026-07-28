"""Guard: launch Python nodes must be installed via CMake PROGRAMS."""

from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
CMAKE = ROOT / "CMakeLists.txt"
LAUNCH = ROOT / "launch" / "exploration.launch.py"
NAV2_LAUNCH = ROOT / "launch" / "nav2_exploration.launch.py"


def test_harness_positive():
    assert 1 + 1 == 2


def test_harness_negative():
    with pytest.raises(AssertionError):
        assert 1 == 2


def test_current_frontier_view_installed_in_cmake_positive():
    text = CMAKE.read_text(encoding="utf-8")
    assert "current_frontier_view_node.py" in text
    assert "RENAME current_frontier_view_node" in text


def test_coverage_metrics_installed_in_cmake_positive():
    text = CMAKE.read_text(encoding="utf-8")
    assert "coverage_metrics_node.py" in text
    assert "RENAME coverage_metrics_node" in text


def test_launch_references_installed_python_nodes_positive():
    launch = LAUNCH.read_text(encoding="utf-8")
    cmake = CMAKE.read_text(encoding="utf-8")
    for exe in ("maprender_node", "vlm_node", "current_frontier_view_node"):
        assert f'executable="{exe}"' in launch
        assert f"RENAME {exe}" in cmake


def test_nav2_launch_wires_coverage_metrics_positive():
    launch = NAV2_LAUNCH.read_text(encoding="utf-8")
    cmake = CMAKE.read_text(encoding="utf-8")
    assert 'executable="coverage_metrics_node"' in launch
    assert "RENAME coverage_metrics_node" in cmake


def test_missing_rename_would_break_launch_negative():
    """Negative: setup.py console_scripts alone are not enough for ament_cmake."""
    cmake = CMAKE.read_text(encoding="utf-8")
    # Must not rely solely on setup.py for the ROS executable path.
    assert "install(PROGRAMS" in cmake
    assert "current_frontier_view_node.py" in cmake
    assert "coverage_metrics_node.py" in cmake
