#!/usr/bin/env bash
# ci.sh — local mirror of the GitHub Actions gates.
#
# Same gates in the same order: format (cheapest, no build), configure, build,
# unit tests, smoke benchmark, clang-tidy (slowest). Actions splits these across
# lint.yml and build.yml to parallelize; here they are sequential so the first
# failure is the one you read.
#
# The flags map onto CMakePresets.json, which is what the workflows configure
# with -- so `cmake --preset asan-ubsan` and `./scripts/ci.sh --asan --ubsan`
# build the same thing in the same directory.
#
# The flags below configure the build the gates then run against; the gates
# themselves are the same either way.
#
# Usage:
#   ./scripts/ci.sh
#   ./scripts/ci.sh --cuda        # CUDA build; CI has no GPU and always uses CPU
#   ./scripts/ci.sh --cuda-host   # type-check the GPU host paths, no nvcc needed
#   ./scripts/ci.sh --debug       # -O0 -g instead of the default Release
#   ./scripts/ci.sh --asan --ubsan
#   ./scripts/ci.sh --no-style    # skip clang-format / clang-tidy
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
DATA="$(camloc_data_dir "${ROOT}")"

usage() {
  cat <<'EOF'
Usage: ci.sh [--cuda] [--cuda-host] [--debug|--release] [--asan] [--ubsan]
             [--no-style]

Run the same checks as GitHub Actions: format, build, unit tests, smoke
benchmark, clang-tidy.

Build options:
  --cuda       Enable the CUDA build (falls back to CPU stubs if nvcc missing).
               CI has no GPU, so the workflow always builds CPU-only.
  --cuda-host  Compile the CUDA host paths against the CPU stub. Needs neither
               nvcc nor a GPU, and is what keeps the code inside
               #ifdef CAMLOC_CUDA_ENABLED from drifting unnoticed.
  --debug      CMAKE_BUILD_TYPE=Debug. Note the default is Release: unoptimized
               Eigen makes the pose grid ~200x slower, so timings from a debug
               build say nothing about the algorithm.
  --release    CMAKE_BUILD_TYPE=Release (the default; accepted for symmetry).
  --asan       AddressSanitizer.
  --ubsan      UndefinedBehaviorSanitizer, non-recovering.
               Either sanitizer implies RelWithDebInfo unless a build type is
               given: -O0 multiplies with the sanitizer overhead on Eigen-heavy
               code and makes the suite unusable.

Other options:
  --no-style   Skip the clang-format and clang-tidy gates.
  -h, --help   Show this help.

Environment:
  CAMLOC_BUILD_DIR  Build directory. Defaults to a sibling of the repository,
                    ../<repo>-build[-<tags>], so each configuration keeps its
                    own and none of them sit inside the source tree.
EOF
}

CMAKE_EXTRA=()
sanitizers=()
build_tags=()
build_type=""
cuda_flag=""
run_style=true
while [[ $# -gt 0 ]]; do
  case "$1" in
    --cuda) cuda_flag=-DCAMLOC_BUILD_CUDA=ON ;;
    --cuda-host) CMAKE_EXTRA+=(-DCAMLOC_CUDA_HOST_ONLY=ON); build_tags+=(cudahost) ;;
    --debug) build_type=Debug ;;
    --release) build_type=Release ;;
    --asan) sanitizers+=(address); build_tags+=(asan) ;;
    --ubsan) sanitizers+=(undefined); build_tags+=(ubsan) ;;
    --no-style) run_style=false ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
  shift
done

CMAKE_EXTRA+=("${cuda_flag:--DCAMLOC_BUILD_CUDA=OFF}")

# One -fsanitize= list: the flag has to carry every sanitizer at once, because a
# second -fsanitize= does not add to the first.
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
# ordinary Release build gets the bare ../<repo>-build. Derived from build_type
# after the parse rather than inside it, so `--debug --release` cannot leave a
# Release build sitting in a directory named debug.
if [[ "${build_type}" == Debug ]]; then
  build_tags=(debug ${build_tags[@]+"${build_tags[@]}"})
fi

# One directory per configuration, beside the repository. See camloc_build_dir.
BUILD="$(camloc_build_dir "${ROOT}" ${build_tags[@]+"${build_tags[@]}"})"
export CAMLOC_BUILD_DIR="${BUILD}"

if [[ "${run_style}" == true ]]; then
  "${ROOT}/scripts/format.sh" --check
fi

cmake -S "${ROOT}" -B "${BUILD}" "${CMAKE_EXTRA[@]}" -DCAMLOC_BUILD_TESTS=ON
cmake --build "${BUILD}" -j"$(camloc_nproc)"

"${ROOT}/scripts/prepare_smoke_kitti.sh" 120

ctest --test-dir "${BUILD}" --output-on-failure

"${BUILD}/apps/benchmark/benchmark" \
  --data-root "${DATA}" \
  --filter smoke_oracle_cpu \
  --output-json "${DATA}/benchmark_ci.json"

# Last because it is the slowest, and because it reads the compile database the
# configure above just wrote.
if [[ "${run_style}" == true ]]; then
  "${ROOT}/scripts/tidy.sh"
fi

echo "CI checks passed."
