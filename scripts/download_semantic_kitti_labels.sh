#!/usr/bin/env bash
# Download SemanticKITTI odometry labels (~172 MB) and merge into kitti_odometry tree.
#
# Does nothing when the labels are already there. This archive is the slowest
# step in run_all.sh -- ten minutes on a bad day -- and re-fetching it to
# overwrite identical files is pure cost. --force fetches anyway, which is the
# repair for a partial or corrupted unpack.
#
# Usage:
#   ./scripts/download_semantic_kitti_labels.sh [KITTI_ROOT]
#   ./scripts/download_semantic_kitti_labels.sh --force [KITTI_ROOT]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
DATA="$(camloc_data_dir "${ROOT}")"

force=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --force) force=true; shift ;;
    -h|--help) sed -n '2,13p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) echo "Unknown option: $1" >&2; exit 1 ;;
    *) break ;;
  esac
done

KITTI="${1:-${DATA}/kitti_odometry}"
URL="http://www.semantic-kitti.org/assets/data_odometry_labels.zip"

if [[ ! -f "${KITTI}/dataset/sequences/00/calib.txt" ]]; then
  echo "Run scripts/download_kitti_odometry.sh first." >&2
  exit 1
fi

# Labels ship for the eleven training sequences, 00-10. The merge below copies
# one sequence at a time, so an interrupted run leaves the later ones missing --
# and checking only sequence 00, the one everything here evaluates, would call
# that complete and never repair it.
labels_complete() {
  local seq
  for seq in 00 01 02 03 04 05 06 07 08 09 10; do
    [[ -f "${KITTI}/dataset/sequences/${seq}/labels/000000.label" ]] || return 1
  done
}

if [[ "${force}" != true ]] && labels_complete; then
  echo "SemanticKITTI labels already merged into ${KITTI}; nothing to do."
  echo "Re-download with --force."
  exit 0
fi

TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

# curl's own message says what failed but not what to do about it. These
# archives are large and served from one host, so a timeout here is common.
download_failed() {
  echo "" >&2
  echo "Could not download SemanticKITTI labels from:" >&2
  echo "  $1" >&2
  echo "Retry -- the host is often slow rather than down. If it is unreachable" >&2
  echo "from this machine, fetch the archive elsewhere and unpack it into:" >&2
  echo "  ${KITTI}" >&2
  exit 1
}

echo "Downloading SemanticKITTI labels ..."
curl -fsSL "${URL}" -o "${TMP}/labels.zip" || download_failed "${URL}"
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
