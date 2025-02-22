#!/usr/bin/env bash
# ci.sh — configure, build and run the unit tests.
#
# One script, so that "does this still work" is one command with no arguments
# and the gates run in a fixed order -- format first because it is cheapest and
# needs no build, so the first failure is the one you read.
#
# Usage:
#   ./scripts/ci.sh
#   ./scripts/ci.sh --debug       # -O0 -g instead of the default
#   ./scripts/ci.sh --no-style    # skip clang-format
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"

usage() {
  cat <<'USAGE'
Usage: ci.sh [--debug|--release] [--no-style]

Run the gates in order: format, configure, build, unit tests.

Build options:
  --debug      CMAKE_BUILD_TYPE=Debug.
  --release    CMAKE_BUILD_TYPE=Release.

Other options:
  --no-style   Skip the clang-format gate.
  -h, --help   Show this help.

Environment:
  CAMLOC_BUILD_DIR  Build directory. Defaults to a sibling of the repository,
                    ../<repo>-build[-<tags>], so each configuration keeps its
                    own and none of them sit inside the source tree.
USAGE
}

CMAKE_EXTRA=()
build_tags=()
build_type=""
run_style=true
while [[ $# -gt 0 ]]; do
  case "$1" in
    --debug) build_type=Debug ;;
    --release) build_type=Release ;;
    --no-style) run_style=false ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
  shift
done

[[ -n "${build_type}" ]] && CMAKE_EXTRA+=("-DCMAKE_BUILD_TYPE=${build_type}")

# The build type tags the directory only when it is not the default. Derived
# from build_type after the parse rather than inside it, so `--debug --release`
# cannot leave a Release build sitting in a directory named debug.
if [[ "${build_type}" == Debug ]]; then
  build_tags=(debug)
fi

# One directory per configuration, beside the repository. See camloc_build_dir.
BUILD="$(camloc_build_dir "${ROOT}" ${build_tags[@]+"${build_tags[@]}"})"
export CAMLOC_BUILD_DIR="${BUILD}"

if [[ "${run_style}" == true ]]; then
  "${ROOT}/scripts/format.sh" --check
fi

cmake -S "${ROOT}" -B "${BUILD}" ${CMAKE_EXTRA[@]+"${CMAKE_EXTRA[@]}"} -DCAMLOC_BUILD_TESTS=ON
cmake --build "${BUILD}" -j"$(camloc_nproc)"

ctest --test-dir "${BUILD}" --output-on-failure

echo "CI checks passed."
