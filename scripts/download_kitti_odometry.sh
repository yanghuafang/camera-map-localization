#!/usr/bin/env bash
# Download KITTI Odometry poses + calibration (no registration required for these archives).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEST="${1:-${ROOT}/data/kitti_odometry}"
POSES_URL="https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_poses.zip"
CALIB_URL="https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_calib.zip"

mkdir -p "${DEST}"
TMP="$(mktemp -d)"
trap 'rm -rf "${TMP}"' EXIT

echo "Downloading poses to ${DEST} ..."
curl -fsSL "${POSES_URL}" -o "${TMP}/poses.zip"
unzip -q -o "${TMP}/poses.zip" -d "${DEST}"

echo "Downloading calibration ..."
curl -fsSL "${CALIB_URL}" -o "${TMP}/calib.zip"
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
echo "  ./build/apps/eval_sequence/eval_sequence --kitti-root ${DEST} --sequence 00 --max-frames 200"
echo ""
echo "Optional: download grayscale images (large, ~22 GB) from"
echo "  https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_gray.zip"
