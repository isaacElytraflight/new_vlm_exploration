"""Aggregate experiment SQLite rows into mean/std summary tables."""

from __future__ import annotations

import math
import sqlite3
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass(frozen=True)
class AggregateRow:
    experiment_id: str
    algorithm_id: str
    scene_id: str
    n_completed: int
    n_total: int
    coverage_mean: Optional[float]
    coverage_std: Optional[float]
    distance_mean: Optional[float]
    distance_std: Optional[float]
    duration_mean: Optional[float]
    duration_std: Optional[float]


def _mean_std(values: Sequence[float]) -> Tuple[Optional[float], Optional[float]]:
    if not values:
        return None, None
    if len(values) == 1:
        return float(values[0]), 0.0
    mean = sum(values) / len(values)
    var = sum((v - mean) ** 2 for v in values) / (len(values) - 1)
    return float(mean), float(math.sqrt(max(0.0, var)))


def aggregate_experiment(
    db_path: Path | str,
    experiment_id: str,
    *,
    statuses: Sequence[str] = ("completed",),
) -> List[AggregateRow]:
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    try:
        groups = conn.execute(
            """
            SELECT algorithm_id, scene_id,
                   COUNT(*) AS n_total,
                   SUM(CASE WHEN status IN ({}) THEN 1 ELSE 0 END) AS n_completed
            FROM runs
            WHERE experiment_id = ?
            GROUP BY algorithm_id, scene_id
            ORDER BY algorithm_id, scene_id
            """.format(",".join("?" * len(statuses))),
            (*statuses, experiment_id),
        ).fetchall()

        rows: List[AggregateRow] = []
        for g in groups:
            algo = str(g["algorithm_id"])
            scene = str(g["scene_id"])
            detail = conn.execute(
                """
                SELECT final_coverage, distance_m, duration_s
                FROM runs
                WHERE experiment_id = ? AND algorithm_id = ? AND scene_id = ?
                  AND status IN ({})
                """.format(",".join("?" * len(statuses))),
                (experiment_id, algo, scene, *statuses),
            ).fetchall()
            cov = [float(r["final_coverage"]) for r in detail if r["final_coverage"] is not None]
            dist = [float(r["distance_m"]) for r in detail if r["distance_m"] is not None]
            dur = [float(r["duration_s"]) for r in detail if r["duration_s"] is not None]
            c_mean, c_std = _mean_std(cov)
            d_mean, d_std = _mean_std(dist)
            t_mean, t_std = _mean_std(dur)
            rows.append(
                AggregateRow(
                    experiment_id=experiment_id,
                    algorithm_id=algo,
                    scene_id=scene,
                    n_completed=int(g["n_completed"]),
                    n_total=int(g["n_total"]),
                    coverage_mean=c_mean,
                    coverage_std=c_std,
                    distance_mean=d_mean,
                    distance_std=d_std,
                    duration_mean=t_mean,
                    duration_std=t_std,
                )
            )
        return rows
    finally:
        conn.close()


def format_aggregate_table(rows: Sequence[AggregateRow]) -> str:
    if not rows:
        return "(no runs)"

    headers = [
        "algorithm",
        "scene",
        "completed",
        "coverage_mean±std",
        "distance_m_mean±std",
        "duration_s_mean±std",
    ]
    lines = ["\t".join(headers)]
    for row in rows:
        cov = _fmt_pair(row.coverage_mean, row.coverage_std)
        dist = _fmt_pair(row.distance_mean, row.distance_std)
        dur = _fmt_pair(row.duration_mean, row.duration_std)
        lines.append(
            "\t".join(
                [
                    row.algorithm_id,
                    row.scene_id,
                    f"{row.n_completed}/{row.n_total}",
                    cov,
                    dist,
                    dur,
                ]
            )
        )
    return "\n".join(lines)


def _fmt_pair(mean: Optional[float], std: Optional[float]) -> str:
    if mean is None:
        return "—"
    if std is None:
        return f"{mean:.3f}"
    return f"{mean:.3f}±{std:.3f}"
