#!/usr/bin/env bash
# One-time scene dataset download into /data (bind-mounted to sim/data on the
# host, so image rebuilds never re-download).
#
# Free Habitat test scenes + ReplicaCAD are fetched here.
# Default research scene Matterport3D JmbYfDe2QKZ is NOT redistributed (ToS).
# See habitat3-exploration/README.md → "Matterport3D default scene".
set -euo pipefail

source /opt/conda/etc/profile.d/conda.sh
conda activate habitat

python -m habitat_sim.utils.datasets_download \
    --uids habitat_test_scenes replica_cad_dataset \
    --data-path /data

echo "Datasets downloaded to /data:"
ls /data

MP3D_GLB="/data/scene_datasets/mp3d/JmbYfDe2QKZ/JmbYfDe2QKZ.glb"
if [[ -f "${MP3D_GLB}" ]]; then
  echo "OK: default MP3D scene present: ${MP3D_GLB}"
else
  cat <<'EOF'

----------------------------------------------------------------------
Matterport3D default scene (JmbYfDe2QKZ) is missing under /data.
It is intentionally not in git or the Docker image (Matterport ToS).

On each new machine, from the HOST repo (not inside this container):

  1. Get Matterport access + download_mp.py:
       https://niessner.github.io/Matterport/
       Email signed ToS to matterport3d@googlegroups.com

  2. Pull only this house (~200 MB) into sim/data:
       python habitat3-exploration/sim/scripts/download_mp3d_habitat_scene.py \
         --i-agree-to-mp-tos --scene JmbYfDe2QKZ

  3. Confirm:
       ls habitat3-exploration/sim/data/scene_datasets/mp3d/JmbYfDe2QKZ/

Full walkthrough: habitat3-exploration/README.md (Matterport3D default scene)
Habitat docs: https://github.com/facebookresearch/habitat-sim/blob/main/DATASETS.md
----------------------------------------------------------------------
EOF
fi
