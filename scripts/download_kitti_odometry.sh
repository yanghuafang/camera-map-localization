#!/usr/bin/env bash
# Download KITTI Odometry poses + calibration (no registration required for these archives).
#
# Does nothing when the data is already there. run_all.sh runs this on every
# pass, and re-fetching an archive to overwrite byte-identical files is a slow
# way to accomplish nothing. --force fetches anyway, which is the repair for a
# partial or corrupted unpack.
#
# Usage:
#   ./scripts/download_kitti_odometry.sh [DEST]
#   ./scripts/download_kitti_odometry.sh --force [DEST]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
DATA="$(camloc_data_dir "${ROOT}")"

force=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --force) force=true; shift ;;
    -h|--help) sed -n '2,12p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    -*) echo "Unknown option: $1" >&2; exit 1 ;;
    *) break ;;
  esac
done

DEST="${1:-${DATA}/kitti_odometry}"

# Complete means every file the two archives carry, not just the first one:
# poses for sequences 00-10, calibration for 00-21. Checking a single file
# would call an interrupted unzip complete and leave the gap to surface later
# as a confusing failure in eval_sequence.
odometry_complete() {
  local seq
  for seq in 00 01 02 03 04 05 06 07 08 09 10; do
    [[ -f "${DEST}/poses/${seq}.txt" ]] || return 1
  done
  for seq in 00 10 21; do
    [[ -f "${DEST}/dataset/sequences/${seq}/calib.txt" ]] || return 1
  done
}

if [[ "${force}" != true ]] && odometry_complete; then
  echo "KITTI Odometry poses and calibration already at ${DEST}; nothing to do."
  echo "Re-download with --force."
  exit 0
fi
POSES_URL="https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_poses.zip"
CALIB_URL="https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_calib.zip"

mkdir -p "${DEST}"
TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

# curl's own message says what failed but not what to do about it. These
# archives are large and served from one host, so a timeout here is common.
download_failed() {
  echo "" >&2
  echo "Could not download KITTI Odometry poses and calibration from:" >&2
  echo "  $1" >&2
  echo "Retry -- the host is often slow rather than down. If it is unreachable" >&2
  echo "from this machine, fetch the archive elsewhere and unpack it into:" >&2
  echo "  ${DEST}" >&2
  exit 1
}

echo "Downloading poses to ${DEST} ..."
curl -fsSL "${POSES_URL}" -o "${TMP}/poses.zip" || download_failed "${POSES_URL}"
unzip -q -o "${TMP}/poses.zip" -d "${DEST}"

echo "Downloading calibration ..."
curl -fsSL "${CALIB_URL}" -o "${TMP}/calib.zip" || download_failed "${CALIB_URL}"
unzip -q -o "${TMP}/calib.zip" -d "${DEST}"

# Normalize poses layout: some archives use dataset/poses/.
if [[ -d "${DEST}/dataset/poses" && ! -d "${DEST}/poses" ]]; then
  ln -sfn dataset/poses "${DEST}/poses"
fi

echo ""
echo "KITTI Odometry ready at: ${DEST}"
echo "  poses/00.txt"
echo "  dataset/sequences/00/calib.txt"
echo ""
echo "Run evaluation:"
echo "  ./scripts/run_real_kitti.sh"
echo ""
echo "Optional: download grayscale images (large, ~22 GB) from"
echo "  https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_gray.zip"
