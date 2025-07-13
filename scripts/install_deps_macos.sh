#!/usr/bin/env bash
# install_deps_macos.sh — install the build environment on macOS via Homebrew.
#
# Installs everything the build needs, including the C++ libraries, so the
# configure needs no network -- which is the difference between a working
# machine and a confusing CMake failure when the route to github.com is blocked
# or slow.
#
# Three of the four libraries come from Homebrew. Note eigen@3, not eigen: the
# unversioned formula is 5.x, and Eigen's own version file declares 5.x
# incompatible with a request for 3.4, so CMake would decline it. eigen@3 is
# 3.4.1 and is keg-only, which CMakeLists.txt handles by adding the keg to
# CMAKE_PREFIX_PATH.
#
# stb has no Homebrew formula at all, so it is cloned once per machine into the
# directory CMakeLists.txt hints at. Ubuntu packages it as libstb-dev, so this
# step has no counterpart there.
#
# llvm is for clang-format and clang-tidy, not for compiling: Xcode ships
# neither, and the compiler used is Apple Clang. The keg is not linked into
# PATH, which is why lib.sh resolves both tools through `brew --prefix llvm`
# rather than expecting them on PATH.
#
# Not installed here, and why:
#   CUDA   — unavailable on macOS. Builds fall back to the CPU stub.
#   ROS 2  — a large opt-in for the RViz playback only; see docs/VISUALIZATION.md.
#
# Usage:
#   ./scripts/install_deps_macos.sh
#   ./scripts/install_deps_macos.sh --dry-run
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
Usage: install_deps_macos.sh [--groups LIST] [--dry-run]

Install everything needed to build, test, lint and document the project on
macOS, including the C++ libraries. stb has no Homebrew formula and is cloned
into the shared source cache instead. Eigen comes from eigen@3 (3.4.1); the
unversioned formula is 5.x, which this project does not build against.

Options:
  --groups LIST  Comma-separated subset to install; default is all of them.
                 build  cmake, ninja, the C++ libraries, and the stb clone
                 style  llvm (clang-format, clang-tidy)
                 docs   doxygen, graphviz
  --dry-run      Print what would be installed and exit.
  -h, --help     Show this help.
EOF
}

dry_run=false
groups=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --groups) groups="${2:-}"; shift ;;
    --groups=*) groups="${1#*=}" ;;
    --dry-run) dry_run=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
  shift
done

if [[ "$(uname -s)" != Darwin ]]; then
  echo "This script installs dependencies on macOS. On Ubuntu use:" >&2
  echo "  ./scripts/install_deps_ubuntu.sh" >&2
  exit 1
fi

# cmake and ninja build; llvm supplies the two style gates; doxygen and graphviz
# build the API reference. curl and unzip ship with macOS, so the download
# scripts need nothing here.
# Grouped so a caller can take only what it needs -- CI builds do not need
# Doxygen, and the formatting job needs neither the libraries nor the stb clone.
GROUP_build=(
  cmake ninja        # build
  eigen@3            # Eigen3::Eigen -- 3.4.1; the unversioned formula is 5.x
  nlohmann-json      # nlohmann_json::nlohmann_json
  googletest         # GTest::gtest_main
)
GROUP_style=(llvm)   # clang-format, clang-tidy
GROUP_docs=(doxygen graphviz)

ALL_GROUPS=(build style docs)
: "${groups:=}"
if [[ -z "${groups}" ]]; then
  selected=("${ALL_GROUPS[@]}")
else
  IFS=',' read -r -a selected <<< "${groups}"
fi

PACKAGES=()
for group in "${selected[@]}"; do
  var="GROUP_${group}[@]"
  if [[ -z "${!var+set}" ]]; then
    echo "Unknown group: ${group}" >&2
    echo "Known groups: ${ALL_GROUPS[*]}" >&2
    exit 1
  fi
  PACKAGES+=("${!var}")
done

# Sources with no formula, cloned into the directory CMakeLists.txt hints at.
# Name, URL, and the branch to track. Part of the build group: nothing else
# needs stb, and the clone is a network round trip worth skipping when it does.
DEPS_CACHE="$(cd "${ROOT}/.." && pwd)/camera-map-localization-deps"
CACHED_SOURCES=()
if [[ " ${selected[*]} " == *" build "* ]]; then
  CACHED_SOURCES=(
    "stb https://github.com/nothings/stb.git master"
  )
fi

if [[ "${dry_run}" == true ]]; then
  echo "Would install with Homebrew: ${PACKAGES[*]}"
  for entry in ${CACHED_SOURCES[@]+"${CACHED_SOURCES[@]}"}; do
    set -- ${entry}
    echo "Would clone $1 ($3) into ${DEPS_CACHE}/$1"
  done
  exit 0
fi

# The Apple Clang toolchain and the SDK come from the Command Line Tools, which
# Homebrew itself needs. tidy.sh also asks xcrun for the SDK path, so a missing
# CLT surfaces there as a page of unresolved standard headers.
if ! xcode-select -p >/dev/null 2>&1; then
  echo "Xcode Command Line Tools not found; requesting the installer ..."
  xcode-select --install || true
  echo "Re-run this script once that installer finishes." >&2
  exit 1
fi

if ! command -v brew >/dev/null 2>&1; then
  echo "Homebrew not found. Install it from https://brew.sh, then re-run:" >&2
  echo "  /bin/bash -c \"\$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)\"" >&2
  exit 1
fi

brew install "${PACKAGES[@]}"

mkdir -p "${DEPS_CACHE}"
for entry in ${CACHED_SOURCES[@]+"${CACHED_SOURCES[@]}"}; do
  # Word-split deliberately: each entry is "name url tag".
  set -- ${entry}
  name="$1"
  url="$2"
  tag="$3"
  dest="${DEPS_CACHE}/${name}"
  if [[ -d "${dest}" ]]; then
    echo "${name} already cached at ${dest}"
    continue
  fi
  echo "Cloning ${name} ${tag} into ${dest} ..."
  if ! git clone --depth 1 --branch "${tag}" "${url}" "${dest}"; then
    echo "" >&2
    echo "Could not clone ${name} from ${url}." >&2
    echo "There is no fallback: the configure will fail until a checkout is at" >&2
    echo "  ${dest}" >&2
    echo "Copy one from another machine if the route to github.com is blocked." >&2
    exit 1
  fi
done

echo ""
echo "macOS build environment ready."
echo "  Build and test:  ${ROOT}/scripts/ci.sh"
echo "  API docs:        ${ROOT}/scripts/docs.sh"
