#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
BUILD="${ROOT}/build"
KITTI="${ROOT}/data/kitti_odometry"

if [[ ! -f "${KITTI}/poses/00.txt" && ! -f "${KITTI}/dataset/poses/00.txt" ]]; then
  echo "KITTI odometry not found. Downloading poses + calib (~2 MB) ..."
  chmod +x "${ROOT}/scripts/download_kitti_odometry.sh"
  "${ROOT}/scripts/download_kitti_odometry.sh" "${KITTI}"
fi

if [[ ! -x "${BUILD}/apps/eval_sequence/eval_sequence" ]]; then
  cmake -S "${ROOT}" -B "${BUILD}" -DCAMLOC_BUILD_CUDA=ON -DCAMLOC_BUILD_TESTS=ON
  cmake --build "${BUILD}" -j"$(camloc_nproc)"
fi

OUT="${ROOT}/data/eval_seq00.csv"
echo "=== eval_sequence (KF sampling plane, synthesized perception) ==="
"${BUILD}/apps/eval_sequence/eval_sequence" \
  --kitti-root "${KITTI}" \
  --sequence 00 \
  --max-frames 60 \
  --skip-frames 10 \
  --output-csv "${OUT}"

if command -v nvidia-smi >/dev/null 2>&1; then
  echo ""
  echo "=== eval_sequence with CUDA (faster pose grid) ==="
  "${BUILD}/apps/eval_sequence/eval_sequence" \
    --kitti-root "${KITTI}" \
    --sequence 00 \
    --max-frames 120 \
    --skip-frames 10 \
    --use-cuda
fi
