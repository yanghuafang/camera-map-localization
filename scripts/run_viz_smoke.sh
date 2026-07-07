#!/usr/bin/env bash
# Render visualization for smoke KITTI (no camera images required).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
BUILD="${ROOT}/build"
OUT="${ROOT}/data/viz_smoke"

"${ROOT}/scripts/prepare_smoke_kitti.sh" 120

if [[ ! -x "${BUILD}/apps/viz_frame/viz_frame" ]]; then
  cmake -S "${ROOT}" -B "${BUILD}" -DCAMLOC_BUILD_CUDA=ON -DCAMLOC_BUILD_TESTS=ON
  cmake --build "${BUILD}" -j"$(camloc_nproc)" --target viz_frame
fi

mkdir -p "${OUT}"

"${BUILD}/apps/viz_frame/viz_frame" \
  --kitti-root "${ROOT}/data/smoke_kitti" \
  --perception-mode oracle \
  --use-gt-plane \
  --frame 20 \
  --trajectory \
  --output-dir "${OUT}"

echo ""
echo "Open ${OUT}/frame_000020_panel.png"
