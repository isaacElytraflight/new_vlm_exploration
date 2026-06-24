#!/usr/bin/env bash
# One-time scene dataset download into /data (bind-mounted to sim/data on the
# host, so image rebuilds never re-download).
set -euo pipefail

source /opt/conda/etc/profile.d/conda.sh
conda activate habitat

python -m habitat_sim.utils.datasets_download \
    --uids habitat_test_scenes replica_cad_dataset \
    --data-path /data

echo "Datasets downloaded to /data:"
ls /data
