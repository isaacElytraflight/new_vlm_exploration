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
    filter_algorithms,
    frozen_config_json,
    is_timestamped_ablation_id,
    make_ablation_experiment_id,
    migrate_bare_ablation_campaign,
    resolve_scene_path,
    run_id_for,
)
from experiments.profiles import (  # noqa: E402
    get_profile,
    normalize_brain_id,
    resolve_algorithm_profile,
)


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
    assert len(specs) == 8
    ids = {s.algorithm_id for s in specs}
    assert ids == {
        "vlm_dfs",
        "greedy_nearest",
        "vlm_frontier_graph",
        "vlm_choice_dijkstra",
    }
    assert all(s.seed in (0, 1) for s in specs)
    by_algo = {s.algorithm_id: s.profile.brain_id for s in specs}
    assert by_algo["vlm_dfs"] == "vlm_tree_dfs"
    assert by_algo["greedy_nearest"] == "greedy_nearest"
    assert by_algo["vlm_frontier_graph"] == "vlm_frontier_graph"
    assert by_algo["vlm_choice_dijkstra"] == "vlm_choice_dijkstra"


def test_smoke_brains_are_distinct_algorithms_positive():
    """Goal C exit: matrix cells load distinct brains, not param-toggle clones."""
    cfg = ExperimentConfig.from_yaml(
        Path(__file__).resolve().parents[1] / "configs" / "smoke.yaml"
    )
    brains = {s.profile.brain_id for s in expand_matrix(cfg)}
    assert brains == {
        "vlm_tree_dfs",
        "greedy_nearest",
        "vlm_frontier_graph",
        "vlm_choice_dijkstra",
    }

def test_resolve_algorithm_brain_field_positive():
    profile = resolve_algorithm_profile({"id": "cell", "brain": "vlm_dfs"})
    assert profile.brain_id == "vlm_tree_dfs"


def test_resolve_algorithm_brain_overrides_profile_positive():
    profile = resolve_algorithm_profile(
        {
            "id": "cell",
            "profile": "exploration_policy_vlm_default",
            "brain": "greedy_nearest",
        }
    )
    assert profile.brain_id == "greedy_nearest"
    assert profile.id == "exploration_policy_vlm_default"


def test_legacy_greedy_profile_maps_to_true_greedy_brain_positive():
    profile = get_profile("exploration_policy_greedy")
    assert profile.brain_id == "greedy_nearest"


def test_normalize_brain_alias_positive():
    assert normalize_brain_id("vlm_dfs") == "vlm_tree_dfs"


def test_unknown_brain_negative():
    with pytest.raises(KeyError, match="Unknown brain"):
        normalize_brain_id("not_a_brain")


def test_frozen_config_includes_brain_id_positive():
    cfg = ExperimentConfig.from_yaml(
        Path(__file__).resolve().parents[1] / "configs" / "smoke.yaml"
    )
    spec = next(s for s in expand_matrix(cfg) if s.algorithm_id == "greedy_nearest")
    payload = frozen_config_json(spec)
    assert '"brain_id": "greedy_nearest"' in payload


def test_run_id_for_short_name_positive():
    cfg = ExperimentConfig.from_yaml(
        Path(__file__).resolve().parents[1] / "configs" / "smoke.yaml"
    )
    greedy = next(s for s in expand_matrix(cfg) if s.algorithm_id == "greedy_nearest" and s.seed == 0)
    assert run_id_for(greedy) == "greedy_nearest_seed0"


def test_run_id_for_omits_scene_and_campaign_negative():
    cfg = ExperimentConfig.from_yaml(
        Path(__file__).resolve().parents[1] / "configs" / "smoke.yaml"
    )
    spec = next(iter(expand_matrix(cfg)))
    rid = run_id_for(spec)
    assert "__" not in rid
    assert spec.scene_id not in rid
    assert spec.experiment_id not in rid


def test_make_ablation_experiment_id_positive():
    assert make_ablation_experiment_id("20260908_120000") == "ablation_run_20260908_120000"
    assert make_ablation_experiment_id("ablation_run_20260908_120000") == "ablation_run_20260908_120000"


def test_make_ablation_experiment_id_empty_uses_timestamp_negative():
    eid = make_ablation_experiment_id("")
    assert eid.startswith("ablation_run_")
    assert len(eid) > len("ablation_run_")


def test_make_ablation_experiment_id_rejects_bare_placeholder_positive():
    eid = make_ablation_experiment_id("ablation_run")
    assert eid.startswith("ablation_run_")
    assert eid != "ablation_run"


def test_is_timestamped_ablation_id_negative():
    assert not is_timestamped_ablation_id("ablation_run")
    assert not is_timestamped_ablation_id("ablation_run_")
    assert is_timestamped_ablation_id("ablation_run_20260908_223000")


def test_migrate_bare_ablation_campaign_positive(tmp_path: Path):
    bare = tmp_path / "ablation_run"
    bare.mkdir()
    (bare / "campaign_info.json").write_text(
        '{"experiment_id": "ablation_run"}\n', encoding="utf-8"
    )
    assert migrate_bare_ablation_campaign(tmp_path, "ablation_run_20260908_223000")
    dest = tmp_path / "ablation_run_20260908_223000"
    assert dest.is_dir()
    assert not bare.exists()
    info = (dest / "campaign_info.json").read_text(encoding="utf-8")
    assert "ablation_run_20260908_223000" in info


def test_migrate_bare_ablation_campaign_missing_negative(tmp_path: Path):
    assert not migrate_bare_ablation_campaign(tmp_path, "ablation_run_20260908_223000")


def test_fresh_cli_does_not_migrate_bare_folder_positive(tmp_path: Path, monkeypatch):
    """Fresh must not attempt to rename a legacy ablation_run/ into the new campaign."""
    import experiments.run_experiment as re

    project = tmp_path / "proj"
    (project / "sim" / "data" / "experiments" / "ablation_run").mkdir(parents=True)
    cfg = project / "smoke.yaml"
    cfg.write_text(
        "\n".join(
            [
                "experiment_id: ablation_run",
                "n_runs_per_cell: 1",
                "timeout_s: 10",
                "artifact_root: sim/data/experiments",
                "algorithms:",
                "  - id: greedy_nearest",
                "    brain: greedy_nearest",
                "scenes: [JmbYfDe2QKZ]",
                "seeds: {mode: sequential, start: 0}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )

    calls = {"n": 0}

    def track(*_a, **_k):
        calls["n"] += 1
        raise PermissionError("locked")

    monkeypatch.setattr(re, "migrate_bare_ablation_campaign", track)

    class FakeOrch:
        def __init__(self, *a, **k):
            pass

        def run_all(self):
            return []

    monkeypatch.setattr(re, "ExperimentOrchestrator", FakeOrch)
    rc = re.main(
        [
            str(cfg),
            "--project-root",
            str(project),
            "--fresh",
            "--experiment-id",
            "ablation_run_20260908_223736",
            "--dry-run",
        ]
    )
    assert rc == 0
    assert calls["n"] == 0


def test_cli_n_runs_per_cell_override_positive(tmp_path: Path, monkeypatch):
    import experiments.run_experiment as re

    project = tmp_path / "proj"
    project.mkdir()
    cfg = project / "smoke.yaml"
    cfg.write_text(
        "\n".join(
            [
                "experiment_id: ablation_run",
                "n_runs_per_cell: 1",
                "timeout_s: 10",
                "artifact_root: sim/data/experiments",
                "algorithms:",
                "  - id: greedy_nearest",
                "    brain: greedy_nearest",
                "scenes: [JmbYfDe2QKZ]",
                "seeds: {mode: sequential, start: 0}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    captured: dict = {}

    class FakeOrch:
        def __init__(self, config, **_k):
            captured["n_runs"] = config.n_runs_per_cell

        def run_all(self):
            return []

    monkeypatch.setattr(re, "ExperimentOrchestrator", FakeOrch)
    rc = re.main(
        [
            str(cfg),
            "--project-root",
            str(project),
            "--fresh",
            "--n-runs-per-cell",
            "4",
            "--dry-run",
        ]
    )
    assert rc == 0
    assert captured["n_runs"] == 4


def test_cli_n_runs_per_cell_invalid_negative(tmp_path: Path):
    import experiments.run_experiment as re

    project = tmp_path / "proj"
    project.mkdir()
    cfg = project / "smoke.yaml"
    cfg.write_text(
        "\n".join(
            [
                "experiment_id: ablation_run",
                "n_runs_per_cell: 1",
                "timeout_s: 10",
                "algorithms:",
                "  - id: greedy_nearest",
                "    brain: greedy_nearest",
                "scenes: [JmbYfDe2QKZ]",
                "seeds: {mode: sequential, start: 0}",
            ]
        )
        + "\n",
        encoding="utf-8",
    )
    with pytest.raises(SystemExit):
        re.main(
            [
                str(cfg),
                "--project-root",
                str(project),
                "--fresh",
                "--n-runs-per-cell",
                "0",
                "--dry-run",
            ]
        )


def test_unknown_profile_negative():
    with pytest.raises(KeyError, match="Unknown profile"):
        get_profile("not_a_real_profile")

def test_filter_algorithms_subset_positive():
    algos = [
        {"id": "vlm_dfs", "brain": "vlm_tree_dfs"},
        {"id": "greedy_nearest", "brain": "greedy_nearest"},
        {"id": "vlm_frontier_graph", "brain": "vlm_frontier_graph"},
    ]
    filtered = filter_algorithms(algos, ["greedy_nearest", "vlm_dfs"])
    assert [a["id"] for a in filtered] == ["vlm_dfs", "greedy_nearest"]


def test_filter_algorithms_none_keeps_all_positive():
    algos = [
        {"id": "a", "brain": "vlm_tree_dfs"},
        {"id": "b", "brain": "greedy_nearest"},
    ]
    assert filter_algorithms(algos, None) == algos
    assert filter_algorithms(algos, []) == algos


def test_filter_algorithms_unknown_id_negative():
    algos = [{"id": "vlm_dfs", "brain": "vlm_tree_dfs"}]
    with pytest.raises(ValueError, match="unknown algorithm"):
        filter_algorithms(algos, ["not_a_brain"])


def test_filter_algorithms_empty_result_negative():
    """All requested ids missing from config must not silently run zero cells."""
    algos = [{"id": "vlm_dfs", "brain": "vlm_tree_dfs"}]
    with pytest.raises(ValueError, match="unknown algorithm"):
        filter_algorithms(algos, ["greedy_nearest"])


def test_harness_negative_control():
    with pytest.raises(AssertionError):
        assert 1 == 2
