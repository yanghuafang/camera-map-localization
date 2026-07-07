#!/usr/bin/env bash
# Oracle vs real/noisy perception map-matching study on KITTI seq 00.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
BUILD="${ROOT}/build"
KITTI="${ROOT}/data/kitti_odometry"
PERCEPTION="${ROOT}/data/perception"
SEQ=00
START=10
END=200
NOISE_PX="${NOISE_PX:-4}"

if [[ ! -x "${BUILD}/apps/eval_perception_compare/eval_perception_compare" ]]; then
  cmake -S "${ROOT}" -B "${BUILD}" -DCAMLOC_BUILD_CUDA=ON -DCAMLOC_BUILD_TESTS=ON
  cmake --build "${BUILD}" -j"$(camloc_nproc)" --target eval_perception_compare eval_sequence preprocess_kitti
fi

LABEL_DIR="${KITTI}/dataset/sequences/${SEQ}/labels"
if [[ -d "${LABEL_DIR}" ]] && [[ ! -f "${PERCEPTION}/${SEQ}/$(printf '%06d.lanes.json' "${START}")" ]]; then
  echo "=== preprocess labels → perception JSON (no velodyne required) ==="
  mkdir -p "${PERCEPTION}/${SEQ}"
  # Project label point-cloud rows via lidar when velodyne exists; otherwise skip.
  if [[ -f "${KITTI}/dataset/sequences/${SEQ}/velodyne/$(printf '%06d.bin' "${START}")" ]]; then
    "${BUILD}/apps/preprocess_kitti/preprocess_kitti" \
      --mode lidar \
      --kitti-root "${KITTI}" \
      --output-root "${PERCEPTION}" \
      --sequence "${SEQ}" \
      --start "${START}" \
      --end "${END}"
  else
    echo "No velodyne; noisy/oracle compare will use synthesized lanes + noise." >&2
  fi
fi

OUT="${ROOT}/data/eval_perception_compare_seq${SEQ}.csv"
echo ""
echo "=== oracle vs real vs noisy (noise_px=${NOISE_PX}) ==="
"${BUILD}/apps/eval_perception_compare/eval_perception_compare" \
  --kitti-root "${KITTI}" \
  --perception-root "${PERCEPTION}" \
  --sequence "${SEQ}" \
  --skip-frames "${START}" \
  --max-frames $((END + 1)) \
  --noise-px "${NOISE_PX}" \
  --use-cuda \
  --output-csv "${OUT}"

echo ""
echo "=== noise sweep (oracle baseline via noisy synth when no file lanes) ==="
for PX in 0 2 4 8 12; do
  SUM=$("${BUILD}/apps/eval_sequence/eval_sequence" \
    --kitti-root "${KITTI}" \
    --perception-root "${PERCEPTION}" \
    --sequence "${SEQ}" \
    --skip-frames "${START}" \
    --max-frames $((END + 1)) \
    --perception-mode noisy \
    --noise-px "${PX}" \
    --use-cuda 2>&1 | awk '/Translation RMSE/{print $4}')
  echo "noise_px=${PX}  rmse_m=${SUM}"
done

echo ""
echo "Results: ${OUT}"
