# Shared helpers for camera-map-localization scripts.
# Source after defining ROOT:
#   source "${ROOT}/scripts/lib.sh"
#
# No `set -euo pipefail` here: shell options are not scoped to a file, so setting
# them in something sourced would change the caller's shell too.

# Number of online CPUs, for parallel builds.
camloc_nproc() {
  if command -v nproc >/dev/null 2>&1; then
    nproc
  else
    getconf _NPROCESSORS_ONLN 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4
  fi
}

# Build directory for a configuration, as a sibling of the repository.
#
#   camloc_build_dir "${ROOT}"                -> ../camera-map-localization-build
#   camloc_build_dir "${ROOT}" debug          -> ../camera-map-localization-build-debug
#
# The default configuration gets the bare name; every departure from it adds a
# tag. Two configurations differing only in build type therefore still get
# different directories, because the non-default one carries a `debug` tag.
#
# Out of tree, so `git status` never has to look past build output and deleting a
# configuration is an rm -rf on something that is not the working tree. Kept
# beside the repository rather than in a shared scratch directory so that a tree
# copied to another host leaves its build behind.
#
# One directory per configuration, so switching between build types is not a
# full rebuild each time.
#
# CAMLOC_BUILD_DIR overrides the whole scheme.
camloc_build_dir() {
  local root="$1"
  shift
  if [[ -n "${CAMLOC_BUILD_DIR:-}" ]]; then
    echo "${CAMLOC_BUILD_DIR}"
    return
  fi
  local name
  name="$(basename "${root}")"
  local dir="${name}-build"
  local tag
  for tag in "$@"; do
    dir="${dir}-${tag}"
  done
  echo "$(cd "${root}/.." && pwd)/${dir}"
}
