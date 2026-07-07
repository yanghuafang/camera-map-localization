#!/usr/bin/env bash
# Local mirror of .github/workflows/ci.yml (CPU build + ctest + smoke benchmark).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
BUILD="${ROOT}/build"

CMAKE_EXTRA=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --cuda) CMAKE_EXTRA+=(-DCAMLOC_BUILD_CUDA=ON) ;;
    -h|--help)
      echo "Usage: $0 [--cuda]"
      echo "  --cuda  Enable CUDA build (falls back to CPU stubs if nvcc missing)"
      exit 0
      ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
  shift
done

if [[ ${#CMAKE_EXTRA[@]} -eq 0 ]]; then
  CMAKE_EXTRA=(-DCAMLOC_BUILD_CUDA=OFF)
fi

cmake -S "${ROOT}" -B "${BUILD}" "${CMAKE_EXTRA[@]}" -DCAMLOC_BUILD_TESTS=ON
cmake --build "${BUILD}" -j"$(camloc_nproc)"

"${ROOT}/scripts/prepare_smoke_kitti.sh" 120

ctest --test-dir "${BUILD}" --output-on-failure

"${BUILD}/apps/benchmark/benchmark" \
  --repo-root "${ROOT}" \
  --filter smoke_oracle_cpu \
  --output-json "${ROOT}/data/benchmark_ci.json"

echo "CI checks passed."
