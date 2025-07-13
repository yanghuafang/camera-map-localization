#!/usr/bin/env bash
# install_deps_ubuntu.sh — install the build environment on Ubuntu via apt.
#
# Installs everything the build needs, including the C++ libraries. Ubuntu
# packages all four, so the configure needs no network at all -- which is the
# difference between a working machine and a confusing CMake failure when the
# route to github.com is blocked or slow.
#
# Unlike the macOS side, clang-format and clang-tidy come from the distro and
# land on PATH, so lib.sh finds them without a keg prefix. Their version follows
# the release; the tree formats identically under 18 through 22, and
# .clang-format restates the one Google default that changed across that range.
#
# Not installed here, and why:
#   CUDA   — a large, driver-coupled install with its own NVIDIA instructions,
#            and every build works without it via the CPU stub. See
#            docs/BUILD.md.
#   ROS 2  — a large opt-in for the RViz playback only; see docs/VISUALIZATION.md.
#
# Usage:
#   ./scripts/install_deps_ubuntu.sh
#   ./scripts/install_deps_ubuntu.sh --dry-run
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
Usage: install_deps_ubuntu.sh [--groups LIST] [--dry-run]

Install everything needed to build, test, lint and document the project on
Ubuntu, including the C++ libraries. With these present the build needs no
network.

Options:
  --groups LIST  Comma-separated subset to install; default is all of them.
                 build     toolchain and the C++ libraries
                 style     clang-format, clang-tidy
                 coverage  clang, llvm (scripts/coverage.sh)
                 docs      doxygen, graphviz
                 data      curl, unzip (scripts/download_*.sh)
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

# Grouped so a caller can take only what it needs. Every CI job used to install
# all of this -- five Linux jobs each pulling Doxygen, Graphviz and LLVM to
# compile a CPU build that calls none of them. The groups are the whole reason
# the workflows can stop restating package names of their own.
GROUP_build=(
  # Toolchain and build
  build-essential
  cmake
  ninja-build
  git
  # C++ libraries the project links against. 24.04 and 26.04 both carry
  # versions new enough for the minimums CMakeLists.txt asks for.
  libeigen3-dev
  nlohmann-json3-dev
  libgtest-dev
  libstb-dev
)
GROUP_style=(clang-format clang-tidy)
# scripts/coverage.sh: it forces CXX=clang++ because the report is built from
# LLVM's source-based instrumentation, not GCC's .gcda files. clang and llvm
# are both distro-default aliases, so they stay on the same major version --
# which matters, since llvm-profdata rejects a raw profile written by a
# different one.
GROUP_coverage=(clang llvm)
GROUP_docs=(doxygen graphviz)
# scripts/download_*.sh
GROUP_data=(curl unzip)

ALL_GROUPS=(build style coverage docs data)
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

if [[ "${dry_run}" == true ]]; then
  echo "Would install with apt: ${PACKAGES[*]}"
  exit 0
fi

sudo apt-get update
# --no-install-recommends keeps this to what is actually used, matching the CI
# workflow so a local environment and a runner resolve the same packages.
sudo apt-get install -y --no-install-recommends "${PACKAGES[@]}"

echo ""
echo "Ubuntu build environment ready."
echo "  Build and test:  ${ROOT}/scripts/ci.sh"
echo "  API docs:        ${ROOT}/scripts/docs.sh"
