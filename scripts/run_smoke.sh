#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
DATA="$(camloc_data_dir "${ROOT}")"
BUILD="$(camloc_build_dir "${ROOT}")"
KITTI_ROOT="${DATA}/smoke_kitti"

"${ROOT}/scripts/prepare_smoke_kitti.sh"

if [[ ! -x "${BUILD}/apps/run_sequence/run_sequence" ]]; then
  cmake -S "${ROOT}" -B "${BUILD}" -DCAMLOC_BUILD_TESTS=ON
  cmake --build "${BUILD}" -j"$(camloc_nproc)"
fi

echo "=== oracle perception, sampling plane at ground truth ==="
"${BUILD}/apps/run_sequence/run_sequence" \
  --kitti-root "${KITTI_ROOT}" \
  --sequence 00 \
  --max-frames 50 \
  --use-gt-plane
