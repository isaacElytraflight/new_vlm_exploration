"""Parse experiment.yaml and expand (algorithm × scene × seed) matrix."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Iterator, List

import yaml

from experiments.profiles import ExplorationProfile, resolve_algorithm_profile


DEFAULT_SCENE_ROOT = "/data/scene_datasets/mp3d"


@dataclass(frozen=True)
class EvalConfig:
    fov_deg: float = 360.0
    reveal_radius_m: float = 5.0


@dataclass(frozen=True)
class RunSpec:
    experiment_id: str
    algorithm_id: str
    profile: ExplorationProfile
    scene_id: str
    scene_path: str
    seed: int
    timeout_s: int
    eval_config: EvalConfig
    artifact_root: Path


@dataclass
class ExperimentConfig:
    experiment_id: str
    n_runs_per_cell: int
    timeout_s: int
    eval: EvalConfig
    algorithms: List[Dict[str, str]]
    scenes: List[str]
    seeds: Dict[str, Any]
    artifact_root: Path = field(default_factory=lambda: Path("sim/data/experiments"))
    container_name: str = "habitat3-sim"
    scene_root: str = DEFAULT_SCENE_ROOT

    @classmethod
    def from_yaml(cls, path: Path | str) -> "ExperimentConfig":
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            raise ValueError("experiment config must be a mapping")

        eval_raw = data.get("eval") or {}
        eval_cfg = EvalConfig(
            fov_deg=float(eval_raw.get("fov_deg", 360.0)),
            reveal_radius_m=float(eval_raw.get("reveal_radius_m", 5.0)),
        )
        artifact = data.get("artifact_root")
        return cls(
            experiment_id=str(data["experiment_id"]),
            n_runs_per_cell=int(data.get("n_runs_per_cell", 1)),
            timeout_s=int(data.get("timeout_s", 7200)),
            eval=eval_cfg,
            algorithms=list(data.get("algorithms") or []),
            scenes=[str(s) for s in (data.get("scenes") or [])],
            seeds=dict(data.get("seeds") or {"mode": "sequential", "start": 0}),
            artifact_root=Path(artifact) if artifact else Path("sim/data/experiments"),
            container_name=str(data.get("container_name", "habitat3-sim")),
            scene_root=str(data.get("scene_root", DEFAULT_SCENE_ROOT)),
        )


def resolve_scene_path(scene_id: str, *, scene_root: str = DEFAULT_SCENE_ROOT) -> str:
    """Map Matterport scene id to Habitat .glb path inside the container."""
    sid = scene_id.strip()
    if sid.endswith(".glb"):
        return sid if sid.startswith("/") else f"{scene_root.rstrip('/')}/{sid}"
    return f"{scene_root.rstrip('/')}/{sid}/{sid}.glb"


def expand_seeds(seeds_cfg: Dict[str, Any], n_runs_per_cell: int) -> List[int]:
    mode = str(seeds_cfg.get("mode", "sequential"))
    if mode == "list":
        raw = seeds_cfg.get("list") or []
        if not raw:
            raise ValueError("seeds.mode=list requires non-empty seeds.list")
        return [int(s) for s in raw]
    if mode == "sequential":
        start = int(seeds_cfg.get("start", 0))
        return [start + i for i in range(max(1, int(n_runs_per_cell)))]
    raise ValueError(f"unsupported seeds.mode: {mode!r}")


def filter_algorithms(
    algorithms: List[Dict[str, str]],
    selected_ids: List[str] | None,
) -> List[Dict[str, str]]:
    """Keep algorithms whose id is in selected_ids (order preserved).

    None or empty selected_ids means keep all. Unknown ids raise ValueError.
    """
    if not selected_ids:
        return list(algorithms)
    wanted = [str(x).strip() for x in selected_ids if str(x).strip()]
    if not wanted:
        return list(algorithms)
    by_id = {str(a.get("id", "")): a for a in algorithms}
    missing = [aid for aid in wanted if aid not in by_id]
    if missing:
        raise ValueError(f"unknown algorithm id(s): {', '.join(missing)}")
    # Preserve YAML order among the enabled subset.
    wanted_set = set(wanted)
    return [a for a in algorithms if str(a.get("id", "")) in wanted_set]


def expand_matrix(config: ExperimentConfig) -> Iterator[RunSpec]:
    if not config.algorithms:
        raise ValueError("algorithms[] must not be empty")
    if not config.scenes:
        raise ValueError("scenes[] must not be empty")

    seeds = expand_seeds(config.seeds, config.n_runs_per_cell)
    for algo in config.algorithms:
        algorithm_id = str(algo["id"])
        profile = resolve_algorithm_profile(algo)
        for scene_id in config.scenes:
            scene_path = resolve_scene_path(scene_id, scene_root=config.scene_root)
            for seed in seeds:
                yield RunSpec(
                    experiment_id=config.experiment_id,
                    algorithm_id=algorithm_id,
                    profile=profile,
                    scene_id=scene_id,
                    scene_path=scene_path,
                    seed=int(seed),
                    timeout_s=int(config.timeout_s),
                    eval_config=config.eval,
                    artifact_root=config.artifact_root,
                )


def run_id_for(spec: RunSpec) -> str:
    """Short per-cell folder / DB id: ``{algorithm_id}_seed{seed}``.

    Scene / campaign metadata lives in ``run_info.json``, not the folder name.
    """
    return f"{spec.algorithm_id}_seed{spec.seed}"


def make_ablation_experiment_id(stamp: str | None = None) -> str:
    """Campaign folder name: ``ablation_run_<YYYYMMDD_HHMMSS>``."""
    from datetime import datetime

    raw = (stamp or "").strip().lstrip("_")
    if not raw or raw == "ablation_run":
        return f"ablation_run_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    if raw.startswith("ablation_run_"):
        return raw
    return f"ablation_run_{raw}"


def frozen_config_json(spec: RunSpec) -> str:
    payload = {
        "experiment_id": spec.experiment_id,
        "algorithm_id": spec.algorithm_id,
        "brain_id": spec.profile.brain_id,
        "profile": spec.profile.to_dict(),
        "scene_id": spec.scene_id,
        "scene_path": spec.scene_path,
        "seed": spec.seed,
        "timeout_s": spec.timeout_s,
        "eval": {
            "fov_deg": spec.eval_config.fov_deg,
            "reveal_radius_m": spec.eval_config.reveal_radius_m,
        },
    }
    return json.dumps(payload, sort_keys=True)
