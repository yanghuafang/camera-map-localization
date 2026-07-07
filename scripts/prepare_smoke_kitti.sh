#!/usr/bin/env bash
# Create a minimal KITTI Odometry layout for local smoke tests (no images required).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${ROOT}/data/smoke_kitti"
SEQ="00"
NFRAMES="${1:-120}"

mkdir -p "${OUT}/poses"
mkdir -p "${OUT}/dataset/sequences/${SEQ}"

cp "${ROOT}/tests/data/calib_minimal.txt" "${OUT}/dataset/sequences/${SEQ}/calib.txt"

POSES="${OUT}/poses/${SEQ}.txt"
: > "${POSES}"
for ((i = 0; i < NFRAMES; ++i)); do
  # Straight drive along camera forward (+Z in cam0 / world) at 0.5 m/frame.
  # KITTI pose is row-major 3x4 [R|t]; identity rotation, translation in tz.
  t=$(awk -v i="$i" 'BEGIN { printf "%.6f", i * 0.5 }')
  echo "1 0 0 0 0 1 0 0 0 0 1 ${t}" >> "${POSES}"
done

echo "Smoke KITTI root: ${OUT}"
echo "  poses: ${POSES} (${NFRAMES} frames)"
echo "  calib: ${OUT}/dataset/sequences/${SEQ}/calib.txt"
