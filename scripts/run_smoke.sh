#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
BUILD="${ROOT}/build"
KITTI_ROOT="${ROOT}/data/smoke_kitti"

"${ROOT}/scripts/prepare_smoke_kitti.sh" 120

if [[ ! -x "${BUILD}/apps/run_sequence/run_sequence" ]]; then
  cmake -S "${ROOT}" -B "${BUILD}" -DCAMLOC_BUILD_CUDA=ON -DCAMLOC_BUILD_TESTS=ON
  cmake --build "${BUILD}" -j"$(camloc_nproc)"
fi

echo "=== CPU run (synthesized perception) ==="
"${BUILD}/apps/run_sequence/run_sequence" \
  --kitti-root "${KITTI_ROOT}" \
  --sequence 00 \
  --max-frames 50 \
  --use-gt-plane

if command -v nvidia-smi >/dev/null 2>&1; then
  echo ""
  echo "=== CUDA run ==="
  "${BUILD}/apps/run_sequence/run_sequence" \
    --kitti-root "${KITTI_ROOT}" \
    --sequence 00 \
    --max-frames 50 \
    --use-cuda
fi
