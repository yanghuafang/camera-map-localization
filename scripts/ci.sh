#!/usr/bin/env bash
# ci.sh — configure, build and run the unit tests.
#
# One script, so that "does this still work" is one command with no arguments
# and the gates run in a fixed order -- format first because it is cheapest and
# needs no build, so the first failure is the one you read.
#
# Usage:
#   ./scripts/ci.sh
#   ./scripts/ci.sh --debug       # -O0 -g instead of the default Release
#   ./scripts/ci.sh --asan --ubsan
#   ./scripts/ci.sh --tsan        # races in the CPU pose-grid fan-out
#   ./scripts/ci.sh --no-style    # skip clang-format
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"

usage() {
  cat <<'USAGE'
Usage: ci.sh [--debug|--release] [--asan] [--ubsan] [--tsan] [--no-style]

Run the gates in order: format, configure, build, unit tests.

Build options:
  --debug      CMAKE_BUILD_TYPE=Debug. Note the default is Release: unoptimized
               Eigen makes the pose grid ~200x slower, so timings from a debug
               build say nothing about the algorithm.
  --release    CMAKE_BUILD_TYPE=Release (the default; accepted for symmetry).
  --asan       AddressSanitizer.
  --ubsan      UndefinedBehaviorSanitizer, non-recovering.
  --tsan       ThreadSanitizer, for the CPU pose-grid fan-out. Not combinable
               with --asan: the two replace the same allocator.
               Either sanitizer implies RelWithDebInfo unless a build type is
               given: -O0 multiplies with the sanitizer overhead on Eigen-heavy
               code and makes the suite unusable.

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
sanitizers=()
build_tags=()
build_type=""
run_style=true
while [[ $# -gt 0 ]]; do
  case "$1" in
    --debug) build_type=Debug ;;
    --release) build_type=Release ;;
    --asan) sanitizers+=(address); build_tags+=(asan) ;;
    --ubsan) sanitizers+=(undefined); build_tags+=(ubsan) ;;
    --tsan) sanitizers+=(thread); build_tags+=(tsan) ;;
    --no-style) run_style=false ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
  shift
done

# One -fsanitize= list: the flag has to carry every sanitizer at once, because a
# second -fsanitize= does not add to the first.
# ASan and TSan both replace the allocator, so asking for both produces a
# build that fails to link rather than one that checks twice.
if [[ " ${sanitizers[*]-} " == *" thread "* &&
      " ${sanitizers[*]-} " == *" address "* ]]; then
  echo "--tsan cannot be combined with --asan" >&2
  exit 1
fi

if [[ ${#sanitizers[@]} -gt 0 ]]; then
  CMAKE_EXTRA+=("-DCAMLOC_SANITIZER=$(IFS=,; echo "${sanitizers[*]}")")
  # RelWithDebInfo, not Debug. The instinct is that a sanitizer wants -O0 for
  # readable frames, but this tree is Eigen expression templates: unoptimized it
  # runs some two hundred times slower, and that multiplies with the sanitizer's
  # own overhead rather than adding to it -- one integration test went from half
  # a second to nine minutes. -O2 with -g and -fno-omit-frame-pointer (both set
  # by CAMLOC_SANITIZER) gives frames that are still worth reading. An explicit
  # --debug or --release still wins.
  build_type="${build_type:-RelWithDebInfo}"
fi

[[ -n "${build_type}" ]] && CMAKE_EXTRA+=("-DCMAKE_BUILD_TYPE=${build_type}")

# The build type tags the directory only when it is not the default, so the
# ordinary Release build gets the bare ../<repo>-build. Derived
# from build_type after the parse rather than inside it, so `--debug --release`
# cannot leave a Release build sitting in a directory named debug.
if [[ "${build_type}" == Debug ]]; then
  build_tags=(debug ${build_tags[@]+"${build_tags[@]}"})
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
