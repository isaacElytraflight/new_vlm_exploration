#!/usr/bin/env python3
"""Render pretty figures from a per-run artifact package.

Usage:
  python experiments/render_run.py sim/data/experiments/<exp>/<run_id>/
  python experiments/render_run.py <run_dir> --out <dir> --open-video
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import subprocess
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Allow `python render_run.py` from experiments/ or project root.
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))


def load_manifest(run_dir: Path) -> Dict[str, Any]:
    path = run_dir / "manifest.json"
    if not path.is_file():
        raise FileNotFoundError(f"manifest.json not found in {run_dir}")
    return json.loads(path.read_text(encoding="utf-8"))


def load_coverage_csv(path: Path) -> Tuple[List[float], List[float]]:
    distances: List[float] = []
    coverages: List[float] = []
    if not path.is_file():
        return distances, coverages
    with path.open(encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            distances.append(float(row["distance_m"]))
            coverages.append(float(row["coverage"]) * 100.0)
    return distances, coverages


def load_revisit_bins(path: Path) -> Dict[int, int]:
    if not path.is_file():
        return {}
    raw = json.loads(path.read_text(encoding="utf-8"))
    return {int(k): int(v) for k, v in raw.items()}


def load_summary(path: Path) -> Dict[str, Any]:
    if not path.is_file():
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def render_figures(run_dir: Path, out_dir: Path) -> Dict[str, Path]:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib import patheffects as pe
    except ImportError as exc:  # pragma: no cover
        raise SystemExit(
            "matplotlib is required: pip install matplotlib"
        ) from exc

    run_dir = Path(run_dir)
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest = load_manifest(run_dir)
    paths = manifest.get("paths") or {}

    cov_path = run_dir / paths.get("coverage_vs_distance", "metrics/coverage_vs_distance.csv")
    revisit_path = run_dir / paths.get("revisit_bins", "metrics/revisit_bins.json")
    summary_path = run_dir / paths.get("summary", "metrics/summary.json")
    grid_path = run_dir / paths.get("final_grid_map", "media/final_grid_map.png")
    plan_path = run_dir / paths.get("final_nav_plan", "media/final_nav_plan.png")

    distances, coverages = load_coverage_csv(cov_path)
    revisit = load_revisit_bins(revisit_path)
    summary = load_summary(summary_path)
    outputs: Dict[str, Path] = {}

    # --- Coverage vs distance ---
    fig, ax = plt.subplots(figsize=(8, 4.5), dpi=140)
    fig.patch.set_facecolor("#0f1419")
    ax.set_facecolor("#171c22")
    if distances:
        ax.plot(
            distances,
            coverages,
            color="#5ec8ff",
            linewidth=2.2,
            solid_capstyle="round",
        )
        ax.fill_between(distances, coverages, color="#5ec8ff", alpha=0.12)
    ax.set_xlabel("Distance traveled (m)", color="#c5ccd6")
    ax.set_ylabel("Coverage (%)", color="#c5ccd6")
    ax.set_title("Coverage vs distance", color="#e8ecf1", pad=12)
    ax.tick_params(colors="#9aa3b2")
    for spine in ax.spines.values():
        spine.set_color("#2a3340")
    ax.grid(True, color="#2a3340", linewidth=0.6, alpha=0.8)
    curve_path = out_dir / "coverage_vs_distance.png"
    fig.tight_layout()
    fig.savefig(curve_path, facecolor=fig.get_facecolor())
    plt.close(fig)
    outputs["coverage_vs_distance"] = curve_path

    # --- Revisit histogram ---
    fig, ax = plt.subplots(figsize=(7, 4), dpi=140)
    fig.patch.set_facecolor("#0f1419")
    ax.set_facecolor("#171c22")
    if revisit:
        keys = sorted(revisit.keys())
        vals = [revisit[k] for k in keys]
        ax.bar(keys, vals, color="#ffb454", width=0.7, edgecolor="#0f1419")
    ax.set_xlabel("Visit count", color="#c5ccd6")
    ax.set_ylabel("Cell count", color="#c5ccd6")
    ax.set_title("Path revisit histogram", color="#e8ecf1", pad=12)
    ax.tick_params(colors="#9aa3b2")
    for spine in ax.spines.values():
        spine.set_color("#2a3340")
    hist_path = out_dir / "revisit_histogram.png"
    fig.tight_layout()
    fig.savefig(hist_path, facecolor=fig.get_facecolor())
    plt.close(fig)
    outputs["revisit_histogram"] = hist_path

    # --- Composite panel ---
    fig = plt.figure(figsize=(11, 6.5), dpi=140)
    fig.patch.set_facecolor("#0f1419")
    gs = fig.add_gridspec(2, 2, height_ratios=[3.2, 1.2], hspace=0.25, wspace=0.15)

    ax_g = fig.add_subplot(gs[0, 0])
    ax_p = fig.add_subplot(gs[0, 1])
    ax_t = fig.add_subplot(gs[1, :])
    for a in (ax_g, ax_p, ax_t):
        a.set_facecolor("#171c22")
        a.tick_params(colors="#9aa3b2")
        for spine in a.spines.values():
            spine.set_color("#2a3340")

    def show_img(ax, path: Path, title: str) -> None:
        ax.set_title(title, color="#e8ecf1", fontsize=11)
        ax.set_xticks([])
        ax.set_yticks([])
        if path.is_file():
            img = plt.imread(str(path))
            ax.imshow(img)
        else:
            ax.text(
                0.5,
                0.5,
                "missing",
                ha="center",
                va="center",
                color="#9aa3b2",
                transform=ax.transAxes,
            )

    show_img(ax_g, grid_path, "Final grid map")
    show_img(ax_p, plan_path, "Final nav plan")

    ax_t.set_xticks([])
    ax_t.set_yticks([])
    final_cov = summary.get("final_coverage")
    if final_cov is None:
        final_cov = manifest.get("summary", {}).get("final_coverage")
    cov_pct = f"{100.0 * float(final_cov):.1f}%" if final_cov is not None else "n/a"
    dist = summary.get("distance_m")
    dist_s = f"{float(dist):.1f} m" if dist is not None else "n/a"
    lines = [
        f"{manifest.get('algorithm_id', '?')}  ·  {manifest.get('scene_id', '?')}  ·  seed {manifest.get('seed', '?')}",
        f"status={manifest.get('status', summary.get('status', '?'))}   coverage={cov_pct}   distance={dist_s}",
        f"eval FOV={manifest.get('eval', {}).get('fov_deg', '?')}°   "
        f"radius={manifest.get('eval', {}).get('reveal_radius_m', '?')} m",
    ]
    text = ax_t.text(
        0.02,
        0.5,
        "\n".join(lines),
        color="#e8ecf1",
        fontsize=12,
        va="center",
        family="DejaVu Sans Mono",
        transform=ax_t.transAxes,
    )
    text.set_path_effects([pe.withStroke(linewidth=2, foreground="#0f1419")])
    ax_t.set_title("Run summary", color="#e8ecf1", fontsize=11, loc="left")

    panel_path = out_dir / "summary_panel.png"
    fig.savefig(panel_path, facecolor=fig.get_facecolor(), bbox_inches="tight")
    plt.close(fig)
    outputs["summary_panel"] = panel_path
    return outputs


def open_video(path: Path) -> None:
    if not path.is_file():
        print(f"Video not found: {path}")
        return
    system = platform.system()
    try:
        if system == "Windows":
            os.startfile(str(path))  # type: ignore[attr-defined]
        elif system == "Darwin":
            subprocess.run(["open", str(path)], check=False)
        else:
            subprocess.run(["xdg-open", str(path)], check=False)
    except OSError as exc:
        print(f"Could not open video: {exc}")


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="Render figures from a run package.")
    parser.add_argument("run_dir", type=Path, help="Path to run package directory")
    parser.add_argument(
        "--out",
        type=Path,
        default=None,
        help="Output directory (default: <run_dir>/figures)",
    )
    parser.add_argument(
        "--open-video",
        action="store_true",
        help="Open map_timelapse_10x.mp4 in the default player if present",
    )
    args = parser.parse_args(argv)

    run_dir = args.run_dir.resolve()
    out_dir = (args.out or (run_dir / "figures")).resolve()
    outputs = render_figures(run_dir, out_dir)

    manifest = load_manifest(run_dir)
    video_rel = (manifest.get("paths") or {}).get(
        "map_timelapse", "media/map_timelapse_10x.mp4"
    )
    video_path = run_dir / video_rel

    print(f"Wrote figures under {out_dir}")
    for name, path in outputs.items():
        print(f"  {name}: {path}")
    print(f"Timelapse: {video_path}" + (" (exists)" if video_path.is_file() else " (missing)"))

    if args.open_video:
        open_video(video_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
