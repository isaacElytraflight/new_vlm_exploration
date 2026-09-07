"""JSON progress file for Elytra ablation UI polling."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Optional


def default_progress() -> Dict[str, Any]:
    return {
        "experiment_id": "",
        "count": 0,
        "total": 0,
        "percent": 0,
        "step": "Idle",
        "detail": "Waiting to start.",
        "run_id": "",
        "algorithm_id": "",
        "scene_id": "",
        "seed": 0,
        "phase": "",
        "complete": False,
        "cancel_requested": False,
    }


class ProgressWriter:
    """Atomically-ish update a progress JSON file for host/UI consumers."""

    def __init__(self, path: Path | str | None) -> None:
        self.path = Path(path) if path else None
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)

    def read(self) -> Dict[str, Any]:
        if self.path is None or not self.path.is_file():
            return default_progress()
        try:
            data = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(data, dict):
                return default_progress()
            merged = default_progress()
            merged.update(data)
            return merged
        except (OSError, json.JSONDecodeError):
            return default_progress()

    def update(self, **fields: Any) -> Dict[str, Any]:
        if self.path is None:
            return default_progress()
        data = self.read()
        data.update(fields)
        count = int(data.get("count") or 0)
        total = int(data.get("total") or 0)
        if "percent" not in fields and total > 0:
            # Weight runs evenly; within-run percent can refine later.
            data["percent"] = int(round(100.0 * count / total))
        self.path.write_text(json.dumps(data, indent=2), encoding="utf-8")
        return data

    def cancel_requested(self) -> bool:
        return bool(self.read().get("cancel_requested"))

    def request_cancel(self) -> None:
        self.update(cancel_requested=True, step="Stopping", detail="Cancel requested; finishing current run.")
