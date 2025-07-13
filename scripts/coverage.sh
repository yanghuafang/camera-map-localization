#!/usr/bin/env bash
# coverage.sh — report how much of the library the test suite reaches.
#
# Builds an instrumented library and test binary, runs the suite, and prints a
# per-file line and function report. The question it answers is which of
# cam_loc's own lines run during ctest -- so apps/ and tests/ are excluded from
# the report, since neither is what the tests are testing.
#
# Two things the report is expected to show, so they are not read as
# regressions. src/cuda/distance_transform_gpu.cc is not built at all without
# nvcc, and CudaTest skips without a GPU, so the GPU path reads as absent rather
# than uncovered. viz/ and benchmark/ are exercised by one test each and sit
# lower than core/ by design: they are tools, not the algorithm.
#
# The build goes in its own directory, so the ordinary one is left alone --
# instrumented binaries are several times slower, which is fine here and is not
# what the benchmark should ever measure.
#
# Usage:
#   ./scripts/coverage.sh
#   ./scripts/coverage.sh --html    # also write a browsable line-by-line report
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
BUILD="$(camloc_build_dir "${ROOT}" coverage)"

usage() {
  cat <<'USAGE'
Usage: coverage.sh [--html]

Build an instrumented tree, run the unit tests, and report line and function
coverage of src/ and include/.

Options:
  --html      Also write a browsable line-by-line report and print its path.
  -h, --help  Show this help.

Environment:
  CAMLOC_BUILD_DIR  Where to build (default: ../<repo>-build-coverage).
USAGE
}

want_html=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --html) want_html=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
  shift
done

# llvm-profdata and llvm-cov must come from the same toolchain as the compiler
# below. A raw profile carries a format version, and a newer tool rejects an
# older profile outright ("raw profile version mismatch: ... expected version
# = 11"). On macOS that rules out the Homebrew llvm keg that format.sh and
# tidy.sh use -- those two only parse source and do not care, but these read
# what Apple Clang wrote, so they come from xcrun.
if [[ "$(uname -s)" == Darwin ]]; then
  profdata="$(xcrun --find llvm-profdata 2>/dev/null || echo llvm-profdata)"
  cov="$(xcrun --find llvm-cov 2>/dev/null || echo llvm-cov)"
else
  profdata=llvm-profdata
  cov=llvm-cov
fi

for tool in "${profdata}" "${cov}"; do
  if ! command -v "${tool}" >/dev/null 2>&1; then
    echo "${tool} not found. Install the build environment:" >&2
    echo "  macOS:  ./scripts/install_deps_macos.sh" >&2
    echo "  Ubuntu: ./scripts/install_deps_ubuntu.sh" >&2
    exit 1
  fi
done
echo "Using $(command -v "${cov}")"

# Clang, whatever the platform default is -- GCC writes .gcda, which these tools
# do not read.
export CXX="${CXX:-clang++}"

# See camloc_clang_gcc_flag: on Linux clang can select a GCC whose headers are
# absent, and then not even the compiler check in project() passes.
cxx_flags="$(camloc_clang_gcc_flag "${CXX}")"

cmake -S "${ROOT}" -B "${BUILD}" \
  -DCMAKE_CXX_FLAGS="${cxx_flags}" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCAMLOC_BUILD_CUDA=OFF \
  -DCAMLOC_BUILD_TESTS=ON \
  -DCAMLOC_COVERAGE=ON
cmake --build "${BUILD}" -j"$(camloc_nproc)"

"${ROOT}/scripts/prepare_smoke_kitti.sh" 120

raw_dir="${BUILD}/coverage-raw"
rm -rf "${raw_dir}"
mkdir -p "${raw_dir}"

# %m gives one profile per binary image, so a stale profile from an earlier
# build cannot merge into this one's.
LLVM_PROFILE_FILE="${raw_dir}/%m.profraw" \
  ctest --test-dir "${BUILD}" --output-on-failure

profile="${BUILD}/cam_loc.profdata"
"${profdata}" merge -sparse "${raw_dir}"/*.profraw -o "${profile}"

# Report cam_loc's own sources only. apps/ is argument parsing and printing,
# tests/ is the thing doing the measuring, and the header-only dependencies are
# instrumented wherever they are installed -- so the regex has to name those
# install prefixes, not just the build tree.
tests_binary="${BUILD}/tests/cam_loc_tests"
report_args=(
  "${tests_binary}"
  -instr-profile="${profile}"
  -ignore-filename-regex='(/tests/|/apps/|-build/|/usr/|/opt/|/Library/|-deps/)'
)

"${cov}" report "${report_args[@]}"

if [[ "${want_html}" == true ]]; then
  html_dir="${BUILD}/coverage-html"
  "${cov}" show "${report_args[@]}" -format=html -output-dir="${html_dir}"
  echo ""
  echo "Line-by-line report: ${html_dir}/index.html"
fi
