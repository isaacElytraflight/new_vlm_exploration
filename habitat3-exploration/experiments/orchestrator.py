"""Host-side batch orchestrator: matrix expand → docker episode → SQLite."""

from __future__ import annotations

import json
import shlex
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

from experiments.config import (
    ExperimentConfig,
    RunSpec,
    expand_matrix,
    frozen_config_json,
    run_id_for,
)
from experiments.db import ExperimentDB, utc_now_iso
from experiments.media_cleanup import cleanup_run_media
from experiments.metrics import compute_revisit_bins, coverage_ratio
from experiments.package import write_run_package, write_run_info
from experiments.progress import ProgressWriter
from experiments.state import ExperimentState, config_fingerprint


def build_tmux_episode_start_cmd(env_file: str) -> str:
    """Shell snippet to (re)create the habitat tmux session running start_sim.sh.

    Killing the last tmux session can tear down the server; without start-server
    + a short settle, `new-session` races and fails with
    "server exited unexpectedly".
    """
    return (
        "tmux kill-session -t habitat 2>/dev/null || true; "
        "sleep 0.5; "
        "tmux start-server; "
        "tmux new-session -d -s habitat "
        f"'set -a && source {shlex.quote(env_file)} && set +a && "
        f"bash /workspace/scripts/start_sim.sh'"
    )


def tmux_start_looks_ok(*, new_session_rc: int, has_session_rc: int) -> bool:
    """True when new-session succeeded and `tmux has-session -t habitat` agrees."""
    return int(new_session_rc) == 0 and int(has_session_rc) == 0


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
        resume: bool = True,
        fresh: bool = False,
    ) -> None:
        self.config = config
        self.project_root = project_root.resolve()
        self.dry_run = dry_run
        self.resume = bool(resume) and not bool(fresh)
        self.fresh = bool(fresh)
        self.progress_file = Path(progress_file) if progress_file else None
        self.progress = ProgressWriter(self.progress_file)
        self.db_path = self.project_root / config.artifact_root / "results.sqlite"
        self.db = ExperimentDB(self.db_path)
        self.db.initialize()
        self._specs = list(expand_matrix(config))
        self._exp_dir = (
            self.project_root / config.artifact_root / config.experiment_id
        )
        self._exp_dir.mkdir(parents=True, exist_ok=True)
        self._state = ExperimentState(self._exp_dir / "experiment_state.json")
        self._fingerprint = config_fingerprint(
            json.dumps(
                {
                    "experiment_id": config.experiment_id,
                    "algorithms": config.algorithms,
                    "scenes": config.scenes,
                    "seeds": config.seeds,
                    "n_runs_per_cell": config.n_runs_per_cell,
                    "timeout_s": config.timeout_s,
                    "eval": {
                        "fov_deg": config.eval.fov_deg,
                        "reveal_radius_m": config.eval.reveal_radius_m,
                    },
                },
                sort_keys=True,
            )
        )

    def _completed_ids(self) -> Set[str]:
        completed: Set[str] = set()
        if not self.resume:
            return completed
        completed.update(self.db.list_completed_run_ids(self.config.experiment_id))
        completed.update(self._state.completed_ids())
        return completed

    def run_all(self) -> List[OrchestratorResult]:
        total = len(self._specs)
        if self.fresh:
            self._state.load_or_create(
                experiment_id=self.config.experiment_id,
                total_runs=total,
                fingerprint=self._fingerprint,
                fresh=True,
            )
        else:
            self._state.load_or_create(
                experiment_id=self.config.experiment_id,
                total_runs=total,
                fingerprint=self._fingerprint,
                fresh=False,
            )

        interrupted = self.db.mark_running_interrupted(self.config.experiment_id)
        for rid in interrupted:
            self._state.set_interrupted(rid)

        completed_ids = self._completed_ids()
        pending = [s for s in self._specs if run_id_for(s) not in completed_ids]
        completed_prior = total - len(pending)
        resumed = self.resume and completed_prior > 0

        self.progress.update(
            experiment_id=self.config.experiment_id,
            count=completed_prior,
            total=total,
            percent=int(round(100.0 * completed_prior / total)) if total else 0,
            step="starting",
            detail=(
                f"Resuming {completed_prior}/{total}."
                if resumed
                else f"Starting ablation ({total} run(s))."
            ),
            complete=False,
            cancel_requested=False,
            completed_prior=completed_prior,
            remaining=len(pending),
            current_index=completed_prior,
            resumed=resumed,
        )

        write_run_info(
            self._exp_dir / "campaign_info.json",
            {
                "schema_version": 1,
                "experiment_id": self.config.experiment_id,
                "algorithms": list(self.config.algorithms),
                "scenes": list(self.config.scenes),
                "seeds": dict(self.config.seeds),
                "n_runs_per_cell": self.config.n_runs_per_cell,
                "timeout_s": self.config.timeout_s,
                "eval": {
                    "fov_deg": self.config.eval.fov_deg,
                    "reveal_radius_m": self.config.eval.reveal_radius_m,
                },
                "container_name": self.config.container_name,
                "total_runs": total,
                "fresh": self.fresh,
                "resume": self.resume,
            },
        )

        results: List[OrchestratorResult] = []
        # Count already-completed cells as successes for the summary.
        for run_id in sorted(completed_ids):
            if any(run_id_for(s) == run_id for s in self._specs):
                results.append(OrchestratorResult(run_id=run_id, status="completed"))

        done_count = completed_prior
        for offset, spec in enumerate(pending):
            if self.progress.cancel_requested():
                self.progress.update(
                    step="cancelled",
                    detail="Ablation cancelled before next run.",
                    complete=True,
                    completed_prior=completed_prior,
                    remaining=len(pending) - offset,
                    current_index=done_count,
                    resumed=resumed,
                )
                break
            index = done_count + 1
            result = self.run_one(spec, index=index, total=total, resumed=resumed)
            results.append(result)
            done_count = index
            remaining = max(0, total - done_count)
            self.progress.update(
                completed_prior=completed_prior,
                remaining=remaining,
                current_index=done_count,
                resumed=resumed,
            )

        failed = [r for r in results if r.status not in {"completed", "dry_run"}]
        self.progress.update(
            count=done_count,
            total=total,
            percent=100 if done_count >= total else int(round(100.0 * done_count / total)),
            step="complete" if not failed else "finished_with_errors",
            detail=(
                f"Finished {done_count}/{total} run(s); failed={len(failed)}."
            ),
            complete=True,
            completed_prior=completed_prior,
            remaining=max(0, total - done_count),
            current_index=done_count,
            resumed=resumed,
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
        resumed: bool = False,
        completed_prior: int = 0,
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
            completed_prior=completed_prior,
            remaining=max(0, total - (index - 1)),
            current_index=max(0, index - 1),
            resumed=resumed,
        )

    def run_one(
        self,
        spec: RunSpec,
        *,
        index: int = 1,
        total: int = 1,
        resumed: bool = False,
    ) -> OrchestratorResult:
        run_id = run_id_for(spec)
        artifact_dir = self.project_root / spec.artifact_root / spec.experiment_id / run_id
        artifact_dir.mkdir(parents=True, exist_ok=True)
        started_at = utc_now_iso()
        completed_prior = max(0, index - 1)

        if self.dry_run:
            print(f"[dry-run] would execute {run_id}")
            return OrchestratorResult(run_id=run_id, status="dry_run")

        self._report(
            index=index,
            total=total,
            spec=spec,
            step="recording_run",
            detail="writing SQLite row",
            resumed=resumed,
            completed_prior=completed_prior,
        )
        self._state.set_interrupted(run_id)

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
        # Keep episode progress inside the run package (not experiments/ root).
        progress_file = artifact_dir / ".episode_progress.json"
        stop_flag = artifact_dir / ".collectors_stop"

        try:
            if stop_flag.exists():
                stop_flag.unlink()
            self._report(
                index=index,
                total=total,
                spec=spec,
                step="prepare_scene",
                detail="setting scene + spawn seed",
                resumed=resumed,
                completed_prior=completed_prior,
            )
            self._prepare_scene(spec)
            self._report(
                index=index,
                total=total,
                spec=spec,
                step="start_episode",
                detail="launching ROS + Habitat stack",
                resumed=resumed,
                completed_prior=completed_prior,
            )
            self._start_episode(spec)
            self._report(
                index=index,
                total=total,
                spec=spec,
                step="wait_explore_node",
                detail="waiting for /explore",
                resumed=resumed,
                completed_prior=completed_prior,
            )
            self._wait_for_explore_node(
                spec, index=index, total=total, resumed=resumed, completed_prior=completed_prior
            )
            self._report(
                index=index,
                total=total,
                spec=spec,
                step="apply_profile",
                detail=f"{spec.profile.id} brain={spec.profile.brain_id}",
                resumed=resumed,
                completed_prior=completed_prior,
            )
            self._apply_profile(spec)
            self._start_sidecar_collectors(spec, artifact_dir, stop_flag)
            self._report(
                index=index,
                total=total,
                spec=spec,
                step="collect_metrics",
                detail="episode running",
                phase="starting",
                resumed=resumed,
                completed_prior=completed_prior,
            )
            payload = self._collect_metrics(spec, metrics_path, progress_file=progress_file)
            status = str(payload.get("status", "error"))
            error_message = payload.get("error_message")
            if self.progress.cancel_requested() and status == "completed":
                # rare: completed as cancel landed
                pass
        except Exception as exc:  # noqa: BLE001 — record orchestration failures
            msg = str(exc)
            if self.progress.cancel_requested() or "cancel" in msg.lower():
                status = "interrupted"
                error_message = msg or "interrupted by operator"
            else:
                status = "error"
                error_message = msg
            metrics_path.write_text(
                json.dumps({"status": status, "error_message": error_message}, indent=2),
                encoding="utf-8",
            )
        finally:
            self._stop_sidecar_collectors(stop_flag)
            self._report(
                index=index,
                total=total,
                spec=spec,
                step="stop_episode",
                detail="cleanup",
                resumed=resumed,
                completed_prior=completed_prior,
            )
            self._stop_episode()
            self._pull_sidecar_artifacts(spec, artifact_dir)
            cleanup_run_media(
                artifact_dir,
                container_name=self.config.container_name,
                container_run_dir=self._container_run_dir(spec),
                try_encode=True,
            )

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

        summary = {
            "status": status,
            "error_message": error_message,
            "final_coverage": final_coverage,
            "distance_m": distance_m,
            "duration_s": duration_s,
            "algorithm_id": spec.algorithm_id,
            "scene_id": spec.scene_id,
            "seed": spec.seed,
            "eval": {
                "fov_deg": spec.eval_config.fov_deg,
                "reveal_radius_m": spec.eval_config.reveal_radius_m,
            },
            "started_at": started_at,
            "finished_at": finished_at,
        }

        write_run_package(
            artifact_dir,
            run_id=run_id,
            experiment_id=spec.experiment_id,
            algorithm_id=spec.algorithm_id,
            scene_id=spec.scene_id,
            seed=spec.seed,
            status=status,
            eval_fov_deg=spec.eval_config.fov_deg,
            eval_reveal_radius_m=spec.eval_config.reveal_radius_m,
            coverage_samples=payload.get("coverage_samples") or [],
            trajectory=trajectory,
            summary=summary,
            revisit_bins=revisit,
            brain_id=spec.profile.brain_id,
            scene_path=spec.scene_path,
            environment={
                "container_name": self.config.container_name,
                "project_root": str(self.project_root),
                "artifact_root": str(spec.artifact_root),
                "navigation_stack": "nav2",
                "sim": "habitat3",
            },
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

        if status == "completed":
            self._state.mark_completed(run_id)
        elif status == "interrupted":
            self._state.set_interrupted(run_id)
        else:
            self._state.mark_failed(run_id)

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
            completed_prior=completed_prior,
            remaining=max(0, total - index),
            current_index=index,
            resumed=resumed,
        )
        return OrchestratorResult(run_id=run_id, status=status, error_message=error_message)

    def _container_run_dir(self, spec: RunSpec) -> str:
        return f"/data/experiments/{spec.experiment_id}/{run_id_for(spec)}"

    def _start_sidecar_collectors(
        self, spec: RunSpec, artifact_dir: Path, stop_flag: Path
    ) -> None:
        run_dir = self._container_run_dir(spec)
        media_dir = f"{run_dir}/media"
        logs_dir = f"{run_dir}/logs"
        stop_container = f"{run_dir}/.collectors_stop"
        self._docker(
            f"mkdir -p {shlex.quote(media_dir)} {shlex.quote(logs_dir)} && "
            f"rm -f {shlex.quote(stop_container)}"
        )
        # Detached exec so sidecars survive after this call returns (nohup alone
        # is not enough — docker exec often reaps the process group on exit).
        # Must use system Python 3.12 — conda python3 lacks rclpy bindings.
        for script, args in (
            (
                "experiment_media_recorder.py",
                f"--out-dir {shlex.quote(media_dir)} "
                f"--stop-flag {shlex.quote(stop_container)} "
                f"--max-runtime-s {int(spec.timeout_s) + 120}",
            ),
            (
                "experiment_event_logger.py",
                f"--output {shlex.quote(logs_dir + '/events.jsonl')} --gzip "
                f"--stop-flag {shlex.quote(stop_container)} "
                f"--max-runtime-s {int(spec.timeout_s) + 120}",
            ),
        ):
            log_name = (
                "media_recorder.log"
                if "media_recorder" in script
                else "event_logger.log"
            )
            subprocess.run(
                [
                    "docker",
                    "exec",
                    "-d",
                    self.config.container_name,
                    "bash",
                    "-lc",
                    (
                        "source /opt/ros/jazzy/setup.bash && "
                        "source /opt/explorer_workspace/ros_workspace/install/setup.bash && "
                        f"/usr/bin/python3 /workspace/scripts/{script} {args} "
                        f"> {shlex.quote(logs_dir + '/' + log_name)} 2>&1"
                    ),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
        self._sidecar_stop = stop_container

    def _stop_sidecar_collectors(self, stop_flag: Path) -> None:
        stop_container = getattr(self, "_sidecar_stop", None)
        if stop_container:
            self._docker(f"touch {shlex.quote(stop_container)}", check=False)
            # Allow JPEG dump + ffmpeg finalize (can take several seconds).
            time.sleep(8.0)

    def _pull_sidecar_artifacts(self, spec: RunSpec, artifact_dir: Path) -> None:
        """Ensure media/logs exist on host (bind mount usually already synced)."""
        run_dir = self._container_run_dir(spec)
        # Copy collector log snippet if present.
        for rel in (
            "media/final_grid_map.png",
            "media/final_nav_plan.png",
            "media/map_timelapse_10x.mp4",
            "logs/events.jsonl.gz",
            "logs/events.jsonl",
            "logs/media_recorder.log",
            "logs/event_logger.log",
        ):
            host_path = artifact_dir / rel
            if host_path.is_file():
                continue
            host_path.parent.mkdir(parents=True, exist_ok=True)
            subprocess.run(
                [
                    "docker",
                    "cp",
                    f"{self.config.container_name}:{run_dir}/{rel}",
                    str(host_path),
                ],
                capture_output=True,
                check=False,
            )

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
        run_dir = self._container_run_dir(spec)
        brain_id = spec.profile.brain_id
        self._docker(
            f"mkdir -p {shlex.quote(run_dir)} && "
            f"printf %s {shlex.quote(scene_path)} > /data/selected_scene.path && "
            f"printf %s {shlex.quote(brain_id)} > /data/selected_brain.id"
        )
        env_file = f"{run_dir}/.env"
        self._docker(
            (
                f"printf '%s\\n' "
                f"{shlex.quote(f'HABITAT_SPAWN_SEED={int(spec.seed)}')} "
                f"{shlex.quote(f'HABITAT_SENSOR_RANGE_M={float(spec.eval_config.reveal_radius_m)}')} "
                f"{shlex.quote(f'EXPLORER_BRAIN_ID={brain_id}')} "
                f"> {shlex.quote(env_file)}"
            )
        )
        self._episode_env_file = env_file

    def _start_episode(self, spec: RunSpec) -> None:
        env_file = getattr(self, "_episode_env_file", "")
        start_cmd = build_tmux_episode_start_cmd(env_file)
        last_err = ""
        for attempt in range(1, 4):
            proc = self._docker(start_cmd, check=False)
            has = self._docker("tmux has-session -t habitat", check=False)
            if tmux_start_looks_ok(
                new_session_rc=proc.returncode, has_session_rc=has.returncode
            ):
                time.sleep(5.0)
                return
            last_err = (proc.stderr or proc.stdout or "").strip() or (
                f"new_session_rc={proc.returncode} has_session_rc={has.returncode}"
            )
            # Recover from a dead/wedged tmux server, then retry.
            self._docker(
                "tmux kill-server 2>/dev/null || true; sleep 0.5; tmux start-server",
                check=False,
            )
            time.sleep(0.5 * attempt)
        raise RuntimeError(
            f"failed to start episode tmux session after retries: {last_err}"
        )

    def _wait_for_explore_node(
        self,
        spec: RunSpec,
        *,
        index: int,
        total: int,
        timeout_s: float = 180.0,
        resumed: bool = False,
        completed_prior: int = 0,
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
                resumed=resumed,
                completed_prior=completed_prior,
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
        run_dir = self._container_run_dir(spec)
        container_out = f"{run_dir}/run_metrics.json"
        container_progress = (
            f"{run_dir}/.episode_progress.json" if progress_file is not None else ""
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
                "/usr/bin/python3 /workspace/scripts/experiment_collect.py "
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
        # Also stash a short collector log under the package.
        collector_log = host_output.parent / "logs" / "collector.log"
        collector_log.parent.mkdir(parents=True, exist_ok=True)
        collector_log.write_text((stdout or "") + "\n" + (stderr or ""), encoding="utf-8")
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
