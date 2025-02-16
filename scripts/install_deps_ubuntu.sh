#!/usr/bin/env bash
# install_deps_ubuntu.sh — install the build environment on Ubuntu via apt.
#
# Installs everything the build needs, including the C++ libraries, so the
# configure needs no network at all -- which is the difference between a working
# machine and a confusing CMake failure when the route to github.com is blocked
# or slow.
#
# Not installed here, and why:
#   CUDA   — a large, driver-coupled install with its own NVIDIA instructions.
#   ROS 2  — a large opt-in with its own vendor instructions.
#
# Usage:
#   ./scripts/install_deps_ubuntu.sh
#   ./scripts/install_deps_ubuntu.sh --dry-run
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'USAGE'
Usage: install_deps_ubuntu.sh [--dry-run]

Install everything needed to build the project on Ubuntu, including the C++
libraries. With these present the build needs no network.

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

if [[ "$(uname -s)" != Linux ]]; then
  echo "This script installs dependencies on Ubuntu. On macOS use:" >&2
  echo "  ./scripts/install_deps_macos.sh" >&2
  exit 1
fi

# A warning rather than an error: the package names below are Debian-family and
# usually resolve on Debian and its derivatives, but only Ubuntu is tested.
if [[ -f /etc/os-release ]]; then
  # shellcheck disable=SC1091
  source /etc/os-release
  if [[ "${ID:-}" != ubuntu ]]; then
    echo "Warning: this script targets Ubuntu; package names may differ on ${PRETTY_NAME:-this system}." >&2
  fi
fi

PACKAGES=(
  # Toolchain and build
  build-essential
  cmake
  ninja-build
  git
  # C++ libraries the project links against
  libeigen3-dev
)

if [[ "${dry_run}" == true ]]; then
  echo "Would install with apt: ${PACKAGES[*]}"
  exit 0
fi

sudo apt-get update
# --no-install-recommends keeps this to what is actually used.
sudo apt-get install -y --no-install-recommends "${PACKAGES[@]}"

echo ""
echo "Ubuntu build environment ready."
echo "  Configure and build: see ${ROOT}/docs/BUILD.md"
