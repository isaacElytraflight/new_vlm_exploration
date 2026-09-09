#!/usr/bin/env python3
"""CLI: run an experiment matrix (Goal B batch orchestrator)."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# Allow `python run_experiment.py` from the experiments/ directory.
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from experiments.config import (  # noqa: E402
    ExperimentConfig,
    filter_algorithms,
    make_ablation_experiment_id,
)
from experiments.orchestrator import ExperimentOrchestrator  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Run a Goal B experiment matrix.")
    parser.add_argument(
        "config",
        type=Path,
        help="Path to experiment.yaml",
    )
    parser.add_argument(
        "--project-root",
        type=Path,
        default=_ROOT,
        help="habitat3-exploration project root (default: parent of experiments/)",
    )
    parser.add_argument(
        "--progress-file",
        type=Path,
        default=None,
        help="JSON file updated for Elytra ablation progress UI",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Expand matrix and print planned runs without executing docker",
    )
    resume_group = parser.add_mutually_exclusive_group()
    resume_group.add_argument(
        "--resume",
        dest="resume",
        action="store_true",
        default=True,
        help="Skip completed runs and resume incomplete experiment (default)",
    )
    resume_group.add_argument(
        "--fresh",
        dest="fresh",
        action="store_true",
        help="Ignore prior completions and start a new campaign",
    )
    parser.add_argument(
        "--experiment-id",
        type=str,
        default=None,
        help="Campaign folder id (prefer ablation_run_<timestamp>)",
    )
    parser.add_argument(
        "--experiment-id-suffix",
        type=str,
        default=None,
        help="Deprecated: becomes experiment_id ablation_run_<suffix>",
    )
    parser.add_argument(
        "--algorithms",
        type=str,
        default=None,
        help="Comma-separated algorithm ids to run (subset of YAML algorithms[])",
    )
    args = parser.parse_args(argv)

    config = ExperimentConfig.from_yaml(args.config)
    if args.experiment_id:
        config.experiment_id = make_ablation_experiment_id(str(args.experiment_id).strip())
    elif args.experiment_id_suffix:
        config.experiment_id = make_ablation_experiment_id(str(args.experiment_id_suffix).strip())
    elif args.fresh:
        # Fresh without an explicit id → new ablation_run_<timestamp> folder.
        config.experiment_id = make_ablation_experiment_id()

    if args.algorithms is not None:
        selected = [p.strip() for p in str(args.algorithms).split(",") if p.strip()]
        config.algorithms = filter_algorithms(config.algorithms, selected)

    orch = ExperimentOrchestrator(
        config,
        project_root=args.project_root.resolve(),
        dry_run=args.dry_run,
        progress_file=args.progress_file,
        resume=not bool(args.fresh),
        fresh=bool(args.fresh),
    )
    results = orch.run_all()
    failed = [r for r in results if r.status not in {"completed", "dry_run"}]
    print(f"Finished {len(results)} run(s); failed={len(failed)}")
    for row in results:
        suffix = f" — {row.error_message}" if row.error_message else ""
        print(f"  {row.run_id}: {row.status}{suffix}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
