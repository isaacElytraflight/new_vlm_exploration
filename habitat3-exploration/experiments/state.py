"""Durable inter-run experiment state for crash-safe resume."""

from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence

from experiments.db import utc_now_iso


def config_fingerprint(frozen_matrix_json: str) -> str:
    return hashlib.sha256(frozen_matrix_json.encode("utf-8")).hexdigest()[:16]


def default_state(
    *,
    experiment_id: str,
    total_runs: int,
    fingerprint: str,
) -> Dict[str, Any]:
    return {
        "experiment_id": experiment_id,
        "config_fingerprint": fingerprint,
        "total_runs": int(total_runs),
        "completed_run_ids": [],
        "failed_run_ids": [],
        "interrupted_run_id": None,
        "updated_at": utc_now_iso(),
    }


class ExperimentState:
    """JSON state file updated only after a run is fully finalized."""

    def __init__(self, path: Path | str) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)

    def read(self) -> Optional[Dict[str, Any]]:
        if not self.path.is_file():
            return None
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None
        return data if isinstance(data, dict) else None

    def write(self, data: Dict[str, Any]) -> None:
        payload = dict(data)
        payload["updated_at"] = utc_now_iso()
        tmp = self.path.with_suffix(self.path.suffix + ".tmp")
        tmp.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        os.replace(tmp, self.path)

    def load_or_create(
        self,
        *,
        experiment_id: str,
        total_runs: int,
        fingerprint: str,
        fresh: bool = False,
    ) -> Dict[str, Any]:
        if fresh and self.path.is_file():
            self.path.unlink(missing_ok=True)
        existing = self.read()
        if (
            existing
            and existing.get("experiment_id") == experiment_id
            and existing.get("config_fingerprint") == fingerprint
        ):
            existing["total_runs"] = int(total_runs)
            return existing
        state = default_state(
            experiment_id=experiment_id,
            total_runs=total_runs,
            fingerprint=fingerprint,
        )
        self.write(state)
        return state

    def mark_completed(self, run_id: str) -> Dict[str, Any]:
        state = self.read() or default_state(
            experiment_id="", total_runs=0, fingerprint=""
        )
        completed: List[str] = list(state.get("completed_run_ids") or [])
        failed: List[str] = list(state.get("failed_run_ids") or [])
        if run_id not in completed:
            completed.append(run_id)
        if run_id in failed:
            failed = [r for r in failed if r != run_id]
        state["completed_run_ids"] = completed
        state["failed_run_ids"] = failed
        if state.get("interrupted_run_id") == run_id:
            state["interrupted_run_id"] = None
        self.write(state)
        return state

    def mark_failed(self, run_id: str) -> Dict[str, Any]:
        state = self.read() or default_state(
            experiment_id="", total_runs=0, fingerprint=""
        )
        failed: List[str] = list(state.get("failed_run_ids") or [])
        if run_id not in failed:
            failed.append(run_id)
        state["failed_run_ids"] = failed
        if state.get("interrupted_run_id") == run_id:
            state["interrupted_run_id"] = None
        self.write(state)
        return state

    def set_interrupted(self, run_id: Optional[str]) -> Dict[str, Any]:
        state = self.read() or default_state(
            experiment_id="", total_runs=0, fingerprint=""
        )
        state["interrupted_run_id"] = run_id
        self.write(state)
        return state

    def completed_ids(self) -> Sequence[str]:
        state = self.read()
        if not state:
            return []
        return list(state.get("completed_run_ids") or [])
