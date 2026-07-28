#!/usr/bin/env bash
# Download Matterport3D habitat mesh for one house into the bind-mounted /data tree.
#
# Prerequisites (cannot be automated without your Matterport agreement):
#   1. Request access: https://niessner.github.io/Matterport/
#      Sign ToS → email matterport3d@googlegroups.com
#   2. Matterport sends you download_mp.py (not in this repo).
#
# Usage (from host or inside habitat3-sim):
#   bash /workspace/scripts/download_mp3d_scene.sh /path/to/download_mp.py [SCENE_ID]
#
# Default SCENE_ID = JmbYfDe2QKZ
set -euo pipefail

DOWNLOAD_MP="${1:-}"
SCENE_ID="${2:-JmbYfDe2QKZ}"
DATA_ROOT="${DATA_ROOT:-/data}"
OUT_ROOT="${DATA_ROOT}/scene_datasets/mp3d"
SCENE_DIR="${OUT_ROOT}/${SCENE_ID}"
CONFIG_URL="http://dl.fbaipublicfiles.com/habitat/mp3d/config_v1/mp3d.scene_dataset_config.json"

if [[ -z "${DOWNLOAD_MP}" || ! -f "${DOWNLOAD_MP}" ]]; then
  echo "Usage: $0 /path/to/download_mp.py [SCENE_ID]" >&2
  echo "download_mp.py comes from Matterport after you sign the MP3D ToS." >&2
  echo "Request: https://niessner.github.io/Matterport/" >&2
  exit 2
fi

mkdir -p "${OUT_ROOT}" "${SCENE_DIR}"

if [[ ! -f "${OUT_ROOT}/mp3d.scene_dataset_config.json" ]]; then
  echo "Fetching mp3d.scene_dataset_config.json…" >&2
  curl -fsSL -o "${OUT_ROOT}/mp3d.scene_dataset_config.json" "${CONFIG_URL}"
fi

# Official Habitat path: habitat task archive only (not the full RGB-D dump).
# Some download_mp.py versions accept --id <scan>; fall back to full habitat task.
echo "Downloading MP3D habitat task for ${SCENE_ID} → ${OUT_ROOT}" >&2
if python "${DOWNLOAD_MP}" --help 2>&1 | grep -q -- '--id'; then
  python "${DOWNLOAD_MP}" --task habitat --id "${SCENE_ID}" -o "${OUT_ROOT}"
else
  echo "Note: this download_mp.py has no --id; downloading full habitat task (large)." >&2
  python "${DOWNLOAD_MP}" --task habitat -o "${OUT_ROOT}"
fi

# Normalize layout to …/mp3d/<id>/<id>.glb if the script nested differently.
if [[ ! -f "${SCENE_DIR}/${SCENE_ID}.glb" ]]; then
  found="$(find "${OUT_ROOT}" -type f -name "${SCENE_ID}.glb" 2>/dev/null | head -n 1 || true)"
  if [[ -n "${found}" ]]; then
    mkdir -p "${SCENE_DIR}"
    # Copy sibling assets next to the glb.
    src_dir="$(dirname "${found}")"
    cp -n "${src_dir}/"* "${SCENE_DIR}/" 2>/dev/null || true
  fi
fi

if [[ -f "${SCENE_DIR}/${SCENE_ID}.glb" ]]; then
  echo "OK: ${SCENE_DIR}/${SCENE_ID}.glb" >&2
  ls -lah "${SCENE_DIR}" >&2
else
  echo "Download finished but ${SCENE_DIR}/${SCENE_ID}.glb not found." >&2
  echo "Search result under ${OUT_ROOT}:" >&2
  find "${OUT_ROOT}" -iname "*${SCENE_ID}*" | head -40 >&2
  exit 1
fi
