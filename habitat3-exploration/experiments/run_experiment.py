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

from experiments.config import ExperimentConfig  # noqa: E402
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
    args = parser.parse_args(argv)

    config = ExperimentConfig.from_yaml(args.config)
    orch = ExperimentOrchestrator(
        config,
        project_root=args.project_root.resolve(),
        dry_run=args.dry_run,
        progress_file=args.progress_file,
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
