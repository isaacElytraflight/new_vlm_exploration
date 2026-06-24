#!/usr/bin/env bash
# Wrapper for smoke_test.py — used by the "GPU Smoke Test" elytra button.
set -euo pipefail

source /opt/conda/etc/profile.d/conda.sh
conda activate habitat

# Un-quiet Magnum so the "Renderer: ..." line (expect NVIDIA) is visible.
export MAGNUM_LOG=default
export HABITAT_SIM_LOG=default

exec python /workspace/scripts/smoke_test.py
