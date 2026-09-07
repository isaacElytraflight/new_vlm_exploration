"""Host-side batch orchestrator: matrix expand → docker episode → SQLite."""

from __future__ import annotations

import json
import shlex
import subprocess
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from experiments.config import (
    ExperimentConfig,
    RunSpec,
    expand_matrix,
    frozen_config_json,
    run_id_for,
)
from experiments.db import ExperimentDB, utc_now_iso
from experiments.metrics import compute_revisit_bins, coverage_ratio
from experiments.progress import ProgressWriter


@dataclass
class OrchestratorResult:
    run_id: str
    status: str
    error_message: Optional[str] = None


class ExperimentOrchestrator:
    def __init__(
        self,
        config: ExperimentConfig,
        *,
        project_root: Path,
        dry_run: bool = False,
        progress_file: Path | str | None = None,
    ) -> None:
        self.config = config
        self.project_root = project_root.resolve()
        self.dry_run = dry_run
        self.progress_file = Path(progress_file) if progress_file else None
        self.progress = ProgressWriter(self.progress_file)
        self.db_path = self.project_root / config.artifact_root / "results.sqlite"
        self.db = ExperimentDB(self.db_path)
        self.db.initialize()
        self._specs = list(expand_matrix(config))

    def run_all(self) -> List[OrchestratorResult]:
        total = len(self._specs)
        self.progress.update(
            experiment_id=self.config.experiment_id,
            count=0,
            total=total,
            percent=0,
            step="starting",
            detail=f"Starting ablation ({total} run(s)).",
            complete=False,
            cancel_requested=False,
        )
        results: List[OrchestratorResult] = []
        for index, spec in enumerate(self._specs, start=1):
            if self.progress.cancel_requested():
                self.progress.update(
                    step="cancelled",
                    detail="Ablation cancelled before next run.",
                    complete=True,
                )
                break
            results.append(self.run_one(spec, index=index, total=total))
        failed = [r for r in results if r.status not in {"completed", "dry_run"}]
        self.progress.update(
            count=len(results),
            total=total,
            percent=100,
            step="complete" if not failed else "finished_with_errors",
            detail=(
                f"Finished {len(results)}/{total} run(s); failed={len(failed)}."
            ),
            complete=True,
        )
        return results

    def _report(
        self,
        *,
        index: int,
        total: int,
        spec: RunSpec,
        step: str,
        detail: str,
        phase: str = "",
    ) -> None:
        run_label = (
            f"Run {index}/{total} — {spec.algorithm_id} @ {spec.scene_id} seed={spec.seed}"
        )
        full_detail = f"{run_label} — {detail}"
        if phase:
            full_detail = f"{run_label} — {phase}: {detail}"
        self.progress.update(
            experiment_id=spec.experiment_id,
            count=max(0, index - 1),
            total=total,
            percent=int(round(100.0 * (index - 1) / total)) if total else 0,
            step=step,
            detail=full_detail,
            run_id=run_id_for(spec),
            algorithm_id=spec.algorithm_id,
            scene_id=spec.scene_id,
            seed=spec.seed,
            phase=phase,
            complete=False,
        )

    def run_one(self, spec: RunSpec, *, index: int = 1, total: int = 1) -> OrchestratorResult:
        run_id = run_id_for(spec)
        artifact_dir = self.project_root / spec.artifact_root / spec.experiment_id / run_id
        artifact_dir.mkdir(parents=True, exist_ok=True)
        started_at = utc_now_iso()

        if self.dry_run:
            print(f"[dry-run] would execute {run_id}")
            return OrchestratorResult(run_id=run_id, status="dry_run")

        self._report(
            index=index,
            total=total,
            spec=spec,
            step="recording_run",
            detail="writing SQLite row",
        )

        self.db.insert_run_start(
            run_id=run_id,
            experiment_id=spec.experiment_id,
            algorithm_id=spec.algorithm_id,
            scene_id=spec.scene_id,
            seed=spec.seed,
            config_json=frozen_config_json(spec),
            artifact_dir=str(artifact_dir.relative_to(self.project_root)),
            started_at=started_at,
        )

        metrics_path = artifact_dir / "run_metrics.json"
        status = "error"
        error_message: Optional[str] = None
        payload: Dict[str, Any] = {}
        progress_file = (
            self.progress.path.parent / f".episode_progress_{run_id}.json"
            if self.progress.path
            else None
        )

        try:
            self._report(index=index, total=total, spec=spec, step="prepare_scene", detail="setting scene + spawn seed")
            self._prepare_scene(spec)
            self._report(index=index, total=total, spec=spec, step="start_episode", detail="launching ROS + Habitat stack")
            self._start_episode(spec)
            self._report(index=index, total=total, spec=spec, step="wait_explore_node", detail="waiting for /explore")
            self._wait_for_explore_node(spec, index=index, total=total)
            self._report(index=index, total=total, spec=spec, step="apply_profile", detail=spec.profile.id)
            self._apply_profile(spec)
            self._report(index=index, total=total, spec=spec, step="collect_metrics", detail="episode running", phase="starting")
            payload = self._collect_metrics(spec, metrics_path, progress_file=progress_file)
            status = str(payload.get("status", "error"))
            error_message = payload.get("error_message")
        except Exception as exc:  # noqa: BLE001 — record orchestration failures
            status = "error"
            error_message = str(exc)
            metrics_path.write_text(
                json.dumps({"status": status, "error_message": error_message}, indent=2),
                encoding="utf-8",
            )
        finally:
            self._report(index=index, total=total, spec=spec, step="stop_episode", detail="cleanup")
            self._stop_episode()

        finished_at = utc_now_iso()
        duration_s = payload.get("duration_s")
        distance_m = payload.get("distance_m")
        final_coverage = payload.get("final_coverage")
        if final_coverage is None and payload.get("mapped_m2") is not None:
            final_coverage = coverage_ratio(
                float(payload["mapped_m2"]), float(payload.get("gt_m2") or 0.0)
            )

        trajectory = payload.get("trajectory") or []
        revisit = compute_revisit_bins(
            [(float(p[0]), float(p[1])) for p in trajectory if len(p) >= 2]
        )

        self.db.finalize_run(
            run_id,
            status=status,
            finished_at=finished_at,
            error_message=error_message,
            final_coverage=final_coverage,
            distance_m=distance_m,
            duration_s=duration_s,
            coverage_samples=payload.get("coverage_samples"),
            revisit_bins=revisit,
        )

        (artifact_dir / "revisit_bins.json").write_text(
            json.dumps(revisit, indent=2), encoding="utf-8"
        )

        self.progress.update(
            count=index,
            total=total,
            percent=int(round(100.0 * index / total)) if total else 100,
            step="run_complete",
            detail=f"Run {index}/{total} finished with status={status}",
            run_id=run_id,
            algorithm_id=spec.algorithm_id,
            scene_id=spec.scene_id,
            seed=spec.seed,
            phase=payload.get("phase", ""),
            complete=False,
        )
        return OrchestratorResult(run_id=run_id, status=status, error_message=error_message)

    def _docker(self, script: str, *, check: bool = True) -> subprocess.CompletedProcess[str]:
        cmd = [
            "docker",
            "exec",
            self.config.container_name,
            "bash",
            "-lc",
            script,
        ]
        return subprocess.run(cmd, capture_output=True, text=True, check=check)

    def _prepare_scene(self, spec: RunSpec) -> None:
        scene_path = spec.scene_path
        run_id = run_id_for(spec)
        self._docker(
            f"mkdir -p /data/experiments && printf %s {shlex.quote(scene_path)} > /data/selected_scene.path"
        )
        env_file = f"/data/experiments/.env_{run_id}"
        self._docker(
            " && ".join(
                [
                    "mkdir -p /data/experiments",
                    (
                        f"printf '%s\\n' "
                        f"{shlex.quote(f'HABITAT_SPAWN_SEED={int(spec.seed)}')} "
                        f"{shlex.quote(f'HABITAT_SENSOR_RANGE_M={float(spec.eval_config.reveal_radius_m)}')} "
                        f"> {shlex.quote(env_file)}"
                    ),
                ]
            )
        )
        self._episode_env_file = env_file

    def _start_episode(self, spec: RunSpec) -> None:
        env_file = getattr(self, "_episode_env_file", "")
        start_cmd = (
            f"tmux kill-session -t habitat 2>/dev/null || true; "
            f"tmux new-session -d -s habitat "
            f"'set -a && source {shlex.quote(env_file)} && set +a && bash /workspace/scripts/start_sim.sh'"
        )
        proc = self._docker(start_cmd, check=False)
        if proc.returncode != 0:
            raise RuntimeError(
                f"failed to start episode tmux session: {proc.stderr.strip() or proc.stdout}"
            )
        time.sleep(5.0)

    def _wait_for_explore_node(
        self,
        spec: RunSpec,
        *,
        index: int,
        total: int,
        timeout_s: float = 180.0,
    ) -> None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            if self.progress.cancel_requested():
                raise RuntimeError("ablation cancelled while waiting for explore node")
            proc = self._docker(
                "source /opt/ros/jazzy/setup.bash && "
                "source /opt/explorer_workspace/ros_workspace/install/setup.bash && "
                "ros2 param describe /explore dfs_prefer_highest_openness",
                check=False,
            )
            if proc.returncode == 0:
                return
            self._report(
                index=index,
                total=total,
                spec=spec,
                step="wait_explore_node",
                detail="still waiting for /explore",
            )
            time.sleep(2.0)
        raise TimeoutError("explore node did not become ready within timeout")

    def _apply_profile(self, spec: RunSpec) -> None:
        profile = spec.profile.id
        proc = self._docker(
            "source /opt/ros/jazzy/setup.bash && "
            "source /opt/explorer_workspace/ros_workspace/install/setup.bash && "
            f"bash /workspace/scripts/apply_exploration_profile.sh {shlex.quote(profile)}",
            check=False,
        )
        if proc.returncode != 0:
            raise RuntimeError(
                f"apply_exploration_profile failed: {proc.stderr.strip() or proc.stdout}"
            )

    def _collect_metrics(
        self,
        spec: RunSpec,
        host_output: Path,
        *,
        progress_file: Path | None,
    ) -> Dict[str, Any]:
        container_out = f"/data/experiments/{run_id_for(spec)}_metrics.json"
        container_progress = (
            f"/data/experiments/.episode_progress_{run_id_for(spec)}.json"
            if progress_file is not None
            else ""
        )
        progress_arg = (
            f"--progress-file {shlex.quote(container_progress)} "
            if container_progress
            else ""
        )
        cmd = [
            "docker",
            "exec",
            self.config.container_name,
            "bash",
            "-lc",
            (
                "source /opt/ros/jazzy/setup.bash && "
                "source /opt/explorer_workspace/ros_workspace/install/setup.bash && "
                "python3 /workspace/scripts/experiment_collect.py "
                f"--timeout-s {int(spec.timeout_s)} "
                f"--output {shlex.quote(container_out)} "
                f"{progress_arg}"
            ),
        ]
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

        deadline = time.time() + float(spec.timeout_s) + 120.0
        while proc.poll() is None and time.time() < deadline:
            if self.progress.cancel_requested():
                proc.kill()
                raise RuntimeError("ablation cancelled during metrics collection")
            if progress_file is not None:
                self._merge_episode_progress(progress_file, container_progress, spec)
            time.sleep(2.0)

        if proc.poll() is None:
            proc.kill()
            raise TimeoutError("experiment_collect did not finish in time")

        stdout, stderr = proc.communicate(timeout=5)
        if proc.returncode != 0:
            raise RuntimeError(
                f"experiment_collect failed: {(stderr or stdout).strip()}"
            )
        cat = self._docker(f"cat {shlex.quote(container_out)}", check=False)
        if cat.returncode != 0:
            raise RuntimeError(f"metrics file missing: {container_out}")
        payload = json.loads(cat.stdout)
        host_output.write_text(json.dumps(payload, indent=2), encoding="utf-8")
        return payload

    def _merge_episode_progress(
        self,
        host_progress: Path,
        container_progress: str,
        spec: RunSpec,
    ) -> None:
        cat = self._docker(f"cat {shlex.quote(container_progress)}", check=False)
        if cat.returncode != 0:
            return
        try:
            episode = json.loads(cat.stdout)
        except json.JSONDecodeError:
            return
        phase = str(episode.get("phase") or "")
        detail = str(episode.get("detail") or episode.get("step") or "episode running")
        current = self.progress.read()
        self.progress.update(
            phase=phase,
            detail=current.get("detail", "").split(" — ")[0] + (f" — {phase}: {detail}" if phase else f" — {detail}"),
            algorithm_id=spec.algorithm_id,
            scene_id=spec.scene_id,
            seed=spec.seed,
        )
        try:
            host_progress.write_text(cat.stdout, encoding="utf-8")
        except OSError:
            pass

    def _stop_episode(self) -> None:
        self._docker("bash /workspace/scripts/stop_sim.sh", check=False)
        time.sleep(2.0)
