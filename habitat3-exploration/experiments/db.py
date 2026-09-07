"""SQLite persistence for Goal B experiment runs."""

from __future__ import annotations

import json
import sqlite3
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Iterator, List, Optional, Sequence, Tuple


SCHEMA_SQL = """
CREATE TABLE IF NOT EXISTS runs (
  run_id            TEXT PRIMARY KEY,
  experiment_id     TEXT NOT NULL,
  algorithm_id      TEXT NOT NULL,
  scene_id          TEXT NOT NULL,
  seed              INTEGER NOT NULL,
  started_at        TEXT NOT NULL,
  finished_at       TEXT,
  status            TEXT NOT NULL,
  error_message     TEXT,
  final_coverage    REAL,
  distance_m        REAL,
  duration_s        REAL,
  config_json       TEXT NOT NULL,
  artifact_dir      TEXT
);

CREATE TABLE IF NOT EXISTS coverage_samples (
  run_id      TEXT NOT NULL,
  t_s         REAL NOT NULL,
  distance_m  REAL NOT NULL,
  coverage    REAL NOT NULL,
  FOREIGN KEY (run_id) REFERENCES runs(run_id)
);

CREATE TABLE IF NOT EXISTS revisit_bins (
  run_id          TEXT NOT NULL,
  revisit_count   INTEGER NOT NULL,
  cell_count      INTEGER NOT NULL,
  FOREIGN KEY (run_id) REFERENCES runs(run_id)
);

CREATE INDEX IF NOT EXISTS idx_runs_experiment ON runs(experiment_id);
CREATE INDEX IF NOT EXISTS idx_runs_algo_scene ON runs(experiment_id, algorithm_id, scene_id);
"""


@dataclass
class RunRecord:
    run_id: str
    experiment_id: str
    algorithm_id: str
    scene_id: str
    seed: int
    started_at: str
    finished_at: Optional[str]
    status: str
    error_message: Optional[str]
    final_coverage: Optional[float]
    distance_m: Optional[float]
    duration_s: Optional[float]
    config_json: str
    artifact_dir: Optional[str]


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


class ExperimentDB:
    def __init__(self, db_path: Path | str) -> None:
        self.db_path = Path(db_path)
        self.db_path.parent.mkdir(parents=True, exist_ok=True)

    @contextmanager
    def connect(self) -> Iterator[sqlite3.Connection]:
        conn = sqlite3.connect(self.db_path)
        conn.row_factory = sqlite3.Row
        try:
            yield conn
            conn.commit()
        finally:
            conn.close()

    def initialize(self) -> None:
        with self.connect() as conn:
            conn.executescript(SCHEMA_SQL)

    def insert_run_start(
        self,
        *,
        run_id: str,
        experiment_id: str,
        algorithm_id: str,
        scene_id: str,
        seed: int,
        config_json: str,
        artifact_dir: str,
        started_at: Optional[str] = None,
    ) -> None:
        with self.connect() as conn:
            conn.execute(
                """
                INSERT INTO runs (
                  run_id, experiment_id, algorithm_id, scene_id, seed,
                  started_at, status, config_json, artifact_dir
                ) VALUES (?, ?, ?, ?, ?, ?, 'running', ?, ?)
                """,
                (
                    run_id,
                    experiment_id,
                    algorithm_id,
                    scene_id,
                    int(seed),
                    started_at or utc_now_iso(),
                    config_json,
                    artifact_dir,
                ),
            )

    def finalize_run(
        self,
        run_id: str,
        *,
        status: str,
        finished_at: Optional[str] = None,
        error_message: Optional[str] = None,
        final_coverage: Optional[float] = None,
        distance_m: Optional[float] = None,
        duration_s: Optional[float] = None,
        coverage_samples: Optional[Sequence[Sequence[float]]] = None,
        revisit_bins: Optional[Dict[int, int]] = None,
    ) -> None:
        with self.connect() as conn:
            conn.execute(
                """
                UPDATE runs SET
                  finished_at = ?,
                  status = ?,
                  error_message = ?,
                  final_coverage = ?,
                  distance_m = ?,
                  duration_s = ?
                WHERE run_id = ?
                """,
                (
                    finished_at or utc_now_iso(),
                    status,
                    error_message,
                    final_coverage,
                    distance_m,
                    duration_s,
                    run_id,
                ),
            )
            if coverage_samples:
                conn.executemany(
                    """
                    INSERT INTO coverage_samples (run_id, t_s, distance_m, coverage)
                    VALUES (?, ?, ?, ?)
                    """,
                    [
                        (run_id, float(r[0]), float(r[1]), float(r[2]))
                        for r in coverage_samples
                        if len(r) >= 3
                    ],
                )
            if revisit_bins:
                conn.executemany(
                    """
                    INSERT INTO revisit_bins (run_id, revisit_count, cell_count)
                    VALUES (?, ?, ?)
                    """,
                    [(run_id, int(k), int(v)) for k, v in revisit_bins.items()],
                )

    def get_run(self, run_id: str) -> Optional[RunRecord]:
        with self.connect() as conn:
            row = conn.execute(
                "SELECT * FROM runs WHERE run_id = ?", (run_id,)
            ).fetchone()
        if row is None:
            return None
        return RunRecord(**dict(row))

    def list_runs(self, experiment_id: str) -> List[RunRecord]:
        with self.connect() as conn:
            rows = conn.execute(
                "SELECT * FROM runs WHERE experiment_id = ? ORDER BY started_at",
                (experiment_id,),
            ).fetchall()
        return [RunRecord(**dict(r)) for r in rows]

    def export_jsonl(self, experiment_id: str, out_path: Path | str) -> int:
        out = Path(out_path)
        out.parent.mkdir(parents=True, exist_ok=True)
        runs = self.list_runs(experiment_id)
        with out.open("w", encoding="utf-8") as fh:
            for run in runs:
                fh.write(json.dumps(run.__dict__, sort_keys=True) + "\n")
        return len(runs)
