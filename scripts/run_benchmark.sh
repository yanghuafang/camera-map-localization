#!/usr/bin/env bash
# Run accuracy + performance benchmark suite (smoke + optional real KITTI).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
BUILD="${ROOT}/build"

"${ROOT}/scripts/prepare_smoke_kitti.sh" 120

if [[ ! -x "${BUILD}/apps/benchmark/benchmark" ]]; then
  cmake -S "${ROOT}" -B "${BUILD}" -DCAMLOC_BUILD_CUDA=ON -DCAMLOC_BUILD_TESTS=ON
  cmake --build "${BUILD}" -j"$(camloc_nproc)" --target benchmark
fi

OUT="${ROOT}/data/benchmark_results.json"

echo "=== camera-map-localization benchmark suite ==="
"${BUILD}/apps/benchmark/benchmark" \
  --repo-root "${ROOT}" \
  --filter smoke \
  --output-json "${OUT}"

echo ""
echo "=== micro-benchmarks (DT + pose grid) ==="
"${BUILD}/apps/benchmark/benchmark" \
  --repo-root "${ROOT}" \
  --micro

if [[ -f "${ROOT}/data/kitti_odometry/poses/00.txt" ]]; then
  echo ""
  echo "=== kitti00 subset (if data present) ==="
  "${BUILD}/apps/benchmark/benchmark" \
    --repo-root "${ROOT}" \
    --filter kitti00 \
    --output-json "${ROOT}/data/benchmark_kitti00.json" || true
fi

echo ""
echo "Full results: ${OUT}"
