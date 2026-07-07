#!/usr/bin/env bash
# Download SemanticKITTI odometry labels (~172 MB) and merge into kitti_odometry tree.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
KITTI="${1:-${ROOT}/data/kitti_odometry}"
URL="http://www.semantic-kitti.org/assets/data_odometry_labels.zip"

if [[ ! -f "${KITTI}/dataset/sequences/00/calib.txt" ]]; then
  echo "Run scripts/download_kitti_odometry.sh first." >&2
  exit 1
fi

TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

echo "Downloading SemanticKITTI labels ..."
curl -fsSL "${URL}" -o "${TMP}/labels.zip"
unzip -q -o "${TMP}/labels.zip" -d "${TMP}/extract"

# Zip may unpack as dataset/sequences/XX/labels or sequences/XX/labels
if [[ -d "${TMP}/extract/dataset/sequences" ]]; then
  SRC="${TMP}/extract/dataset/sequences"
elif [[ -d "${TMP}/extract/sequences" ]]; then
  SRC="${TMP}/extract/sequences"
else
  echo "Unexpected zip layout under ${TMP}/extract" >&2
  find "${TMP}/extract" -maxdepth 3 -type d >&2
  exit 1
fi

echo "Merging labels into ${KITTI}/dataset/sequences ..."
for seq in "${SRC}"/*; do
  [[ -d "${seq}/labels" ]] || continue
  id="$(basename "${seq}")"
  mkdir -p "${KITTI}/dataset/sequences/${id}"
  cp -a "${seq}/labels" "${KITTI}/dataset/sequences/${id}/"
  echo "  sequence ${id}: $(ls "${seq}/labels" | wc -l) label files"
done

echo ""
echo "Labels installed. Velodyne scans still required for lidar preprocess:"
echo "  ${KITTI}/dataset/sequences/00/velodyne/000000.bin"
echo "Download: https://www.cvlibs.net/datasets/kitti/eval_odometry.php (data_odometry_velodyne.zip)"
