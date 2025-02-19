#!/usr/bin/env bash
# install_deps_macos.sh — install the build environment on macOS via Homebrew.
#
# Installs everything the build needs, including the C++ libraries, so the
# configure needs no network -- which is the difference between a working
# machine and a confusing CMake failure when the route to github.com is blocked
# or slow.
#
# Note eigen@3, not eigen: the unversioned formula is 5.x, and Eigen's own
# version file declares 5.x incompatible with a request for 3.4, so CMake would
# decline it. eigen@3 is 3.4.1 and is keg-only, which CMakeLists.txt handles by
# adding the keg to CMAKE_PREFIX_PATH.
#
# Not installed here, and why:
#   CUDA   — unavailable on macOS.
#   ROS 2  — a large opt-in with its own vendor instructions.
#
# Usage:
#   ./scripts/install_deps_macos.sh
#   ./scripts/install_deps_macos.sh --dry-run
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'USAGE'
Usage: install_deps_macos.sh [--dry-run]

Install everything needed to build the project on macOS, including the C++
libraries. Eigen comes from eigen@3 (3.4.1); the unversioned formula is 5.x,
which this project does not build against.

Options:
  --dry-run      Print what would be installed and exit.
  -h, --help     Show this help.
USAGE
}

dry_run=false
while [[ $# -gt 0 ]]; do
  case "$1" in
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

PACKAGES=(
  cmake ninja        # build
  eigen@3            # Eigen3::Eigen -- 3.4.1; the unversioned formula is 5.x
  googletest         # GTest::gtest_main
)

if [[ "${dry_run}" == true ]]; then
  echo "Would install with Homebrew: ${PACKAGES[*]}"
  exit 0
fi

# The Apple Clang toolchain and the SDK come from the Command Line Tools, which
# Homebrew itself needs.
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

echo ""
echo "macOS build environment ready."
echo "  Configure and build: see ${ROOT}/docs/BUILD.md"
