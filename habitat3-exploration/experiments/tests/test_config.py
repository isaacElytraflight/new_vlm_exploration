from __future__ import annotations

import sys
from pathlib import Path

import pytest

_ROOT = Path(__file__).resolve().parents[2]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from experiments.config import (  # noqa: E402
    ExperimentConfig,
    expand_matrix,
    expand_seeds,
    resolve_scene_path,
    run_id_for,
)
from experiments.profiles import get_profile


def test_resolve_scene_path_positive():
    assert resolve_scene_path("JmbYfDe2QKZ").endswith("JmbYfDe2QKZ/JmbYfDe2QKZ.glb")


def test_resolve_scene_path_absolute_glb_positive():
    path = resolve_scene_path("/data/foo/bar.glb")
    assert path == "/data/foo/bar.glb"


def test_expand_seeds_sequential_positive():
    seeds = expand_seeds({"mode": "sequential", "start": 3}, n_runs_per_cell=2)
    assert seeds == [3, 4]


def test_expand_seeds_list_positive():
    seeds = expand_seeds({"mode": "list", "list": [10, 20]}, n_runs_per_cell=99)
    assert seeds == [10, 20]


def test_expand_seeds_empty_list_negative():
    with pytest.raises(ValueError, match="non-empty"):
        expand_seeds({"mode": "list", "list": []}, n_runs_per_cell=1)


def test_expand_matrix_smoke_config_positive():
    cfg = ExperimentConfig.from_yaml(
        Path(__file__).resolve().parents[1] / "configs" / "smoke.yaml"
    )
    specs = list(expand_matrix(cfg))
    assert len(specs) == 4
    ids = {s.algorithm_id for s in specs}
    assert ids == {"vlm_dfs", "greedy_nearest"}
    assert all(s.seed in (0, 1) for s in specs)


def test_run_id_for_stable_positive():
    cfg = ExperimentConfig.from_yaml(
        Path(__file__).resolve().parents[1] / "configs" / "smoke.yaml"
    )
    spec = next(iter(expand_matrix(cfg)))
    rid = run_id_for(spec)
    assert "vlm_dfs" in rid or "greedy_nearest" in rid
    assert "seed" in rid


def test_unknown_profile_negative():
    with pytest.raises(KeyError, match="Unknown profile"):
        get_profile("not_a_real_profile")


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
