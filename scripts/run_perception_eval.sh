#!/usr/bin/env bash
# Preprocess SemanticKITTI labels + evaluate with real perception JSON.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
BUILD="${ROOT}/build"
KITTI="${ROOT}/data/kitti_odometry"
PERCEPTION="${ROOT}/data/perception"
SEQ=00
START=10
END=109

if [[ ! -f "${KITTI}/dataset/sequences/${SEQ}/velodyne/$(printf '%06d.bin' "${START}")" ]]; then
  echo "Velodyne scans not found under ${KITTI}/dataset/sequences/${SEQ}/velodyne/" >&2
  echo "Install SemanticKITTI labels: ./scripts/download_semantic_kitti_labels.sh" >&2
  echo "Then add velodyne .bin files from KITTI Odometry (seq ${SEQ})." >&2
  echo "Falling back to synthesized-perception eval only." >&2
  exec "${ROOT}/scripts/run_real_kitti.sh"
fi

if [[ ! -d "${KITTI}/dataset/sequences/${SEQ}/labels" ]]; then
  chmod +x "${ROOT}/scripts/download_semantic_kitti_labels.sh"
  "${ROOT}/scripts/download_semantic_kitti_labels.sh" "${KITTI}"
fi

if [[ ! -x "${BUILD}/apps/preprocess_kitti/preprocess_kitti" ]]; then
  cmake -S "${ROOT}" -B "${BUILD}" -DCAMLOC_BUILD_CUDA=ON -DCAMLOC_BUILD_TESTS=ON
  cmake --build "${BUILD}" -j"$(camloc_nproc)"
fi

echo "=== preprocess_kitti (lidar → JSON) ==="
"${BUILD}/apps/preprocess_kitti/preprocess_kitti" \
  --mode lidar \
  --kitti-root "${KITTI}" \
  --output-root "${PERCEPTION}" \
  --sequence "${SEQ}" \
  --start "${START}" \
  --end "${END}"

echo ""
echo "=== eval with real perception ==="
"${BUILD}/apps/eval_sequence/eval_sequence" \
  --kitti-root "${KITTI}" \
  --perception-root "${PERCEPTION}" \
  --sequence "${SEQ}" \
  --max-frames $((END + 1)) \
  --skip-frames "${START}" \
  --use-cuda \
  --output-csv "${ROOT}/data/eval_seq${SEQ}_perception.csv"
