#!/usr/bin/env python3
"""CLI: aggregate Goal B SQLite results into mean±std tables."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from experiments.aggregate import aggregate_experiment, format_aggregate_table  # noqa: E402
from experiments.db import ExperimentDB  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Aggregate experiment SQLite results.")
    parser.add_argument("experiment_id", help="experiment_id from experiment.yaml")
    parser.add_argument(
        "--db",
        type=Path,
        default=_ROOT / "sim/data/experiments/results.sqlite",
        help="Path to results.sqlite",
    )
    parser.add_argument(
        "--jsonl",
        type=Path,
        default=None,
        help="Optional path to export runs as JSONL",
    )
    args = parser.parse_args(argv)

    db = ExperimentDB(args.db)
    if args.jsonl:
        n = db.export_jsonl(args.experiment_id, args.jsonl)
        print(f"Exported {n} run(s) to {args.jsonl}")

    rows = aggregate_experiment(args.db, args.experiment_id)
    print(format_aggregate_table(rows))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
