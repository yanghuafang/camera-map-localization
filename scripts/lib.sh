# Shared helpers for camera-map-localization scripts.
# Portable across Linux (Ubuntu) and macOS. Source after defining ROOT:
#   source "${ROOT}/scripts/lib.sh"

# Echo the number of online CPUs. Uses nproc on Linux, getconf/sysctl on macOS,
# and falls back to 4 so parallel builds work everywhere.
camloc_nproc() {
  if command -v nproc >/dev/null 2>&1; then
    nproc
  else
    getconf _NPROCESSORS_ONLN 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4
  fi
}
