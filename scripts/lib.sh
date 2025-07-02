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
#   camloc_build_dir "${ROOT}" asan ubsan     -> ../camera-map-localization-build-asan-ubsan
#   camloc_build_dir "${ROOT}" debug          -> ../camera-map-localization-build-debug
#
# The default configuration gets the bare name; every departure from it adds a
# tag. Two configurations differing only in build type therefore still get
# different directories, because the non-default one carries a `debug` tag.
#
# Out of tree, so `git status` never has to look past build output and deleting a
# configuration is an rm -rf on something that is not the working tree. Kept
# beside the repository rather than in a shared scratch directory so that a tree
# copied to another host (see remote_ubuntu.sh) leaves its build behind.
#
# One directory per configuration, so switching between Release and a sanitizer
# build is not a full rebuild each time -- and so an instrumented binary cannot
# be mistaken for the one being benchmarked.
#
# CAMLOC_BUILD_DIR overrides the whole scheme, which is what CI uses to keep its
# build inside the workspace where the artifact upload can find it.
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

# Dataset directory, as a sibling of the repository.
#
#   camloc_data_dir "${ROOT}"   -> ../camera-map-localization-data
#
# Holds smoke_kitti/ (generated), kitti_odometry/ and perception/ (downloaded or
# preprocessed), and the CSV and JSON the eval and benchmark apps write.
#
# Outside the repository for the same reasons as the build directories, plus one
# of its own: KITTI's velodyne set alone is ~80 GB, and a dataset that large
# inside a working tree makes every `git status`, every editor index and every
# rsync pay for it. Nothing here is ours to version -- it is downloaded or
# regenerated -- so the repository is better off not having a place to put it.
#
# CAMLOC_DATA_DIR overrides the scheme, which is what CI uses to keep the
# benchmark JSON inside the workspace where the artifact upload can find it.
# CMakeLists.txt computes the same default for the tests; keep the two in step.
camloc_data_dir() {
  local root="$1"
  if [[ -n "${CAMLOC_DATA_DIR:-}" ]]; then
    echo "${CAMLOC_DATA_DIR}"
    return
  fi
  local name
  name="$(basename "${root}")"
  local dir
  dir="$(cd "${root}/.." && pwd)/${name}-data"
  # Created here rather than at each call site: every caller either writes into
  # it or looks for something under it.
  mkdir -p "${dir}"
  echo "${dir}"
}

# Echo a --gcc-install-dir= flag pinning clang to a usable libstdc++, or
# nothing when the default already works.
#
# On Linux, clang takes libstdc++ from the highest-numbered directory under
# /usr/lib/gcc, whether or not that GCC's headers are installed. Ubuntu 26.04
# leaves gcc 16's CRT objects there while shipping only gcc 15's headers, so
# clang selects a toolchain with no <cmath> and no libstdc++.so. Every
# clang-based tool then fails at once, and the error names a standard header
# rather than the real cause: clang-tidy reports "'cmath' file not found" on
# every file in the tree, and coverage.sh cannot configure at all. g++ is
# unaffected, so the ordinary build keeps passing and only the clang tools break.
#
# Probed rather than assumed: the flag is emitted only when the default cannot
# compile and link, so a machine where clang is set up correctly gets nothing.
# Nothing on macOS either, where clang uses its own libc++.
camloc_clang_gcc_flag() {
  [[ "$(uname -s)" == Linux ]] || return 0
  local cxx="${1:-clang++}"
  command -v "${cxx}" >/dev/null 2>&1 || return 0

  local probe
  probe="$(mktemp -d)"
  printf '#include <cmath>\nint main() { return (int)std::sqrt(0.0); }\n' \
    >"${probe}/probe.cc"
  if "${cxx}" "${probe}/probe.cc" -o "${probe}/probe" >/dev/null 2>&1; then
    rm -rf "${probe}"
    return 0
  fi

  # Newest GCC that actually ships C++ headers -- /usr/include/c++/<n> is the
  # thing that is missing, so it is the thing to look for.
  local dir version best=""
  for dir in /usr/include/c++/*/; do
    version="$(basename "${dir}")"
    [[ "${version}" =~ ^[0-9]+$ ]] || continue
    if [[ -z "${best}" || "${version}" -gt "${best}" ]]; then best="${version}"; fi
  done

  local candidate
  for candidate in /usr/lib/gcc/*/"${best}"; do
    [[ -n "${best}" && -d "${candidate}" ]] || continue
    if "${cxx}" "--gcc-install-dir=${candidate}" "${probe}/probe.cc" \
         -o "${probe}/probe" >/dev/null 2>&1; then
      rm -rf "${probe}"
      echo "--gcc-install-dir=${candidate}"
      return 0
    fi
  done
  rm -rf "${probe}"
}

# ROS 2 directory, as a sibling of the repository.
#
#   camloc_ros_dir "${ROOT}"   -> ../camera-map-localization-build-ros
#
# Holds the distribution in env/ and the colcon workspace in ws/. Named with
# the same -build-<tag> scheme as camloc_build_dir, and out of the tree for the
# same reasons -- doubly so here, where the RoboStack prefix on macOS is close
# to 4 GB and colcon regenerates ws/ from scratch whenever it is deleted.
#
# Not camloc_build_dir "$1" ros, because that honours CAMLOC_BUILD_DIR and
# would then return the C++ build directory instead.
camloc_ros_dir() {
  local root="$1"
  if [[ -n "${CAMLOC_ROS_DIR:-}" ]]; then
    echo "${CAMLOC_ROS_DIR}"
    return
  fi
  local name
  name="$(basename "${root}")"
  echo "$(cd "${root}/.." && pwd)/${name}-build-ros"
}

# Resolve a clang tool (clang-format, clang-tidy) and echo its path.
#
# Order: CAMLOC_CLANG_FORMAT / CAMLOC_CLANG_TIDY override, the Homebrew llvm keg,
# then PATH. The keg comes first because Xcode ships neither tool, so on macOS
# PATH would otherwise miss them entirely or find an unrelated install.
#
# Path goes to stdout for $(...) capture; the banner goes to stderr. The banner
# names the binary and version because differing results between two machines are
# almost always version skew, and that turns it into a one-line diagnosis.
camloc_resolve_clang_tool() {
  local tool="$1"
  local override_var
  override_var="CAMLOC_$(echo "${tool}" | tr 'a-z-' 'A-Z_')"

  local candidates=()
  [[ -n "${!override_var:-}" ]] && candidates+=("${!override_var}")
  if [[ "$(uname -s)" == Darwin ]] && command -v brew >/dev/null 2>&1; then
    local keg
    keg="$(brew --prefix llvm 2>/dev/null || true)"
    [[ -n "${keg}" ]] && candidates+=("${keg}/bin/${tool}")
  fi
  candidates+=("${tool}")

  local resolved="" candidate
  for candidate in "${candidates[@]}"; do
    if [[ -x "${candidate}" ]]; then
      resolved="${candidate}"
      break
    fi
    if command -v "${candidate}" >/dev/null 2>&1; then
      resolved="$(command -v "${candidate}")"
      break
    fi
  done

  if [[ -z "${resolved}" ]]; then
    {
      echo "${tool} not found. Set ${override_var}, or install it:"
      echo "  macOS:  brew install llvm"
      echo "  Ubuntu: sudo apt install ${tool}"
    } >&2
    return 1
  fi

  # Not always on the first line: LLVM's own builds print an
  # "LLVM (http://llvm.org/):" banner and put the version underneath.
  local version
  version="$("${resolved}" --version 2>/dev/null \
    | sed -n 's/.*version \([0-9][0-9.]*\).*/\1/p' | head -1)"
  echo "Using ${resolved} — ${tool} ${version:-(version unknown)}" >&2

  echo "${resolved}"
}

# Every hand-written C++ source and header, one per line.
#
# Named roots rather than a whole-tree find: a hand-made in-tree `cmake -B build`
# fills build/ with CMake's own generated sources, and cam_loc's style is not
# theirs to follow.
#
# The prune covers the other direction. A dependency that is *vendored* into the
# tree -- an upstream ROS package copied under ros/, a Qt or OpenSSL subtree
# under src/ -- would otherwise sit inside a named root and be reformatted to
# Google style on the next ./format.sh. Directories with these conventional
# names are skipped, so vendored code keeps whatever style its upstream uses.
#
# A subtree that must follow another project's conventions but does not sit
# under one of these names can instead carry its own .clang-format and
# .clang-tidy: both tools read the nearest config above each file.
camloc_source_files() {
  local root="$1"
  find "${root}/src" "${root}/include" "${root}/apps" "${root}/tests" "${root}/ros" \
    \( -type d \( -name third_party -o -name third-party -o -name thirdparty \
                  -o -name vendor -o -name external -o -name _deps \) -prune \) \
    -o \( -type f \( -name '*.cc' -o -name '*.h' -o -name '*.cu' \) -print \) \
    | sort
}
