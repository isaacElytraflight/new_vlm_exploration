"""Per-run artifact package writers (Aarush-aligned metrics layout)."""

from __future__ import annotations

import csv
import json
import os
from pathlib import Path
from typing import Any, Dict, Mapping, Optional, Sequence


PACKAGE_SCHEMA_VERSION = 1


def atomic_write_text(path: Path, text: str, *, encoding: str = "utf-8") -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_text(text, encoding=encoding)
    os.replace(tmp, path)


def atomic_write_bytes(path: Path, data: bytes) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    tmp.write_bytes(data)
    os.replace(tmp, path)


def write_coverage_csv(
    path: Path,
    samples: Sequence[Sequence[float]],
) -> None:
    """Write t_s,distance_m,coverage rows (Aarush coverage-vs-distance raw data)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t_s", "distance_m", "coverage"])
        for row in samples:
            if len(row) < 3:
                continue
            writer.writerow([float(row[0]), float(row[1]), float(row[2])])
    os.replace(tmp, path)


def write_trajectory_csv(
    path: Path,
    trajectory: Sequence[Sequence[float]],
) -> None:
    """Write t_s,x,y — accepts [x,y,t] or [t,x,y] if t is last (collector format)."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".tmp")
    with tmp.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["t_s", "x", "y"])
        for row in trajectory:
            if len(row) < 2:
                continue
            if len(row) >= 3:
                # Collector stores [x, y, t_s]
                x, y, t_s = float(row[0]), float(row[1]), float(row[2])
                writer.writerow([t_s, x, y])
            else:
                writer.writerow([0.0, float(row[0]), float(row[1])])
    os.replace(tmp, path)


def write_summary_json(path: Path, summary: Mapping[str, Any]) -> None:
    atomic_write_text(path, json.dumps(dict(summary), indent=2, sort_keys=True) + "\n")


def write_revisit_bins_json(path: Path, bins: Mapping[int, int]) -> None:
    payload = {str(int(k)): int(v) for k, v in bins.items()}
    atomic_write_text(path, json.dumps(payload, indent=2, sort_keys=True) + "\n")


def build_run_info(
    *,
    run_id: str,
    experiment_id: str,
    algorithm_id: str,
    brain_id: str,
    scene_id: str,
    scene_path: str,
    seed: int,
    status: str,
    eval_fov_deg: float,
    eval_reveal_radius_m: float,
    environment: Optional[Mapping[str, Any]] = None,
    extra: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    """Sidecar metadata so short folder names stay human-readable."""
    info: Dict[str, Any] = {
        "schema_version": PACKAGE_SCHEMA_VERSION,
        "run_id": run_id,
        "experiment_id": experiment_id,
        "algorithm_id": algorithm_id,
        "brain_id": brain_id,
        "scene_id": scene_id,
        "scene_path": scene_path,
        "seed": int(seed),
        "status": status,
        "eval": {
            "fov_deg": float(eval_fov_deg),
            "reveal_radius_m": float(eval_reveal_radius_m),
        },
        "environment": dict(environment or {}),
    }
    if extra:
        info.update(dict(extra))
    return info


def write_run_info(path: Path, info: Mapping[str, Any]) -> None:
    atomic_write_text(path, json.dumps(dict(info), indent=2, sort_keys=True) + "\n")


def build_manifest(
    *,
    run_id: str,
    experiment_id: str,
    algorithm_id: str,
    scene_id: str,
    seed: int,
    status: str,
    eval_fov_deg: float,
    eval_reveal_radius_m: float,
    relative_paths: Mapping[str, str],
    byte_sizes: Optional[Mapping[str, int]] = None,
    extra: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    manifest: Dict[str, Any] = {
        "schema_version": PACKAGE_SCHEMA_VERSION,
        "run_id": run_id,
        "experiment_id": experiment_id,
        "algorithm_id": algorithm_id,
        "scene_id": scene_id,
        "seed": int(seed),
        "status": status,
        "eval": {
            "fov_deg": float(eval_fov_deg),
            "reveal_radius_m": float(eval_reveal_radius_m),
        },
        "paths": dict(relative_paths),
        "byte_sizes": dict(byte_sizes or {}),
    }
    if extra:
        manifest.update(dict(extra))
    return manifest


def write_manifest(path: Path, manifest: Mapping[str, Any]) -> None:
    atomic_write_text(path, json.dumps(dict(manifest), indent=2, sort_keys=True) + "\n")


def package_byte_sizes(run_dir: Path, relative_paths: Mapping[str, str]) -> Dict[str, int]:
    sizes: Dict[str, int] = {}
    for key, rel in relative_paths.items():
        p = Path(run_dir) / rel
        if p.is_file():
            sizes[key] = p.stat().st_size
    return sizes


def total_package_bytes(byte_sizes: Mapping[str, int]) -> int:
    return int(sum(int(v) for v in byte_sizes.values()))


def write_run_package(
    run_dir: Path,
    *,
    run_id: str,
    experiment_id: str,
    algorithm_id: str,
    scene_id: str,
    seed: int,
    status: str,
    eval_fov_deg: float,
    eval_reveal_radius_m: float,
    coverage_samples: Sequence[Sequence[float]],
    trajectory: Sequence[Sequence[float]],
    summary: Mapping[str, Any],
    revisit_bins: Mapping[int, int],
    extra_paths: Optional[Mapping[str, str]] = None,
    brain_id: str = "",
    scene_path: str = "",
    environment: Optional[Mapping[str, Any]] = None,
) -> Dict[str, Any]:
    """Write metrics/ layout + manifest + run_info. Media/logs paths may be filled later."""
    run_dir = Path(run_dir)
    metrics = run_dir / "metrics"
    media = run_dir / "media"
    logs = run_dir / "logs"
    metrics.mkdir(parents=True, exist_ok=True)
    media.mkdir(parents=True, exist_ok=True)
    logs.mkdir(parents=True, exist_ok=True)

    write_coverage_csv(metrics / "coverage_vs_distance.csv", coverage_samples)
    write_trajectory_csv(metrics / "trajectory.csv", trajectory)
    write_summary_json(metrics / "summary.json", summary)
    write_revisit_bins_json(metrics / "revisit_bins.json", revisit_bins)

    run_info = build_run_info(
        run_id=run_id,
        experiment_id=experiment_id,
        algorithm_id=algorithm_id,
        brain_id=brain_id or algorithm_id,
        scene_id=scene_id,
        scene_path=scene_path,
        seed=seed,
        status=status,
        eval_fov_deg=eval_fov_deg,
        eval_reveal_radius_m=eval_reveal_radius_m,
        environment=environment,
    )
    write_run_info(run_dir / "run_info.json", run_info)

    paths: Dict[str, str] = {
        "coverage_vs_distance": "metrics/coverage_vs_distance.csv",
        "trajectory": "metrics/trajectory.csv",
        "summary": "metrics/summary.json",
        "revisit_bins": "metrics/revisit_bins.json",
        "run_info": "run_info.json",
        "final_grid_map": "media/final_grid_map.png",
        "final_nav_plan": "media/final_nav_plan.png",
        "map_timelapse": "media/map_timelapse_10x.mp4",
        "events": "logs/events.jsonl.gz",
    }
    if extra_paths:
        paths.update(dict(extra_paths))

    sizes = package_byte_sizes(run_dir, paths)
    manifest = build_manifest(
        run_id=run_id,
        experiment_id=experiment_id,
        algorithm_id=algorithm_id,
        scene_id=scene_id,
        seed=seed,
        status=status,
        eval_fov_deg=eval_fov_deg,
        eval_reveal_radius_m=eval_reveal_radius_m,
        relative_paths=paths,
        byte_sizes=sizes,
        extra={"total_bytes": total_package_bytes(sizes), "brain_id": brain_id or algorithm_id},
    )
    write_manifest(run_dir / "manifest.json", manifest)
    return manifest
