#!/usr/bin/env bash
# Build cam_loc_ros against the main camera-map-localization CMake build.
#
# ROS 2 is the one dependency the install_deps scripts leave out: a desktop
# install is around 2 GB, and only this script and run_ros_viz.sh need it.
# Rather than point at a wiki page, this installs it -- but it asks first,
# because it is large and, on Ubuntu, needs sudo. --install-ros answers yes up
# front; --no-install-ros fails with instructions instead.
#
# Where it comes from differs by platform:
#   Ubuntu   packages.ros.org via apt, into /opt/ros.
#   macOS    RoboStack via micromamba, into ../<repo>-ros. Homebrew carries no
#            ROS formula, and ROS 2's own macOS support is a Tier 3 source build
#            of some two hundred packages, so the conda-forge binaries are the
#            only route that installs unattended and works on Apple Silicon.
#
# Usage:
#   ./scripts/build_ros.sh
#   ./scripts/build_ros.sh --install-ros      # install ROS 2 without asking
#   ./scripts/build_ros.sh --no-install-ros   # never install; fail if missing
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

install_ros=ask
while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-ros) install_ros=yes ;;
    --no-install-ros) install_ros=no ;;
    -h|--help)
      sed -n '2,20p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
  shift
done
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
ROS_DIR="$(camloc_ros_dir "${ROOT}")"
WS="${ROS_DIR}/ws"

BUILD="$(camloc_build_dir "${ROOT}")"
# macOS keeps ROS in a micromamba prefix beside the repository, next to the
# colcon workspace that builds against it (see camloc_ros_dir). A prefix env
# rather than a named one, so it needs no micromamba root and `rm -rf` on the
# one directory is the uninstall.
#
# Note it cannot be moved afterwards: conda writes the absolute prefix into the
# activation scripts and into some binaries, so a relocated env fails with
# nothing more useful than `ros2: command not found`. Deleting it and re-running
# this script is the way to move it.
ROS_ENV="${ROS_DIR}/env"

if [[ ! -f "${BUILD}/src/libcam_loc_core.a" ]]; then
  echo "Building cam_loc_core first..."
  cmake -S "${ROOT}" -B "${BUILD}" -DCAMLOC_BUILD_TESTS=ON
  cmake --build "${BUILD}" -j"$(camloc_nproc)"
fi

mkdir -p "${WS}/src"
if [[ ! -e "${WS}/src/cam_loc_ros" ]]; then
  ln -sfn "${ROOT}/ros/cam_loc_ros" "${WS}/src/cam_loc_ros"
fi

# Prefer an already-sourced distro, else whatever this platform installs into.
# RoboStack's conda packages put setup.bash at the environment prefix itself,
# which is why macOS looks at CONDA_PREFIX rather than /opt/ros.
resolve_ros_setup() {
  if [[ "$(uname -s)" == Darwin ]]; then
    if [[ -n "${CONDA_PREFIX:-}" && -f "${CONDA_PREFIX}/setup.bash" ]]; then
      echo "${CONDA_PREFIX}/setup.bash"
    elif [[ -f "${ROS_ENV}/setup.bash" ]]; then
      echo "${ROS_ENV}/setup.bash"
    fi
    return
  fi
  if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    echo "/opt/ros/${ROS_DISTRO}/setup.bash"
  else
    ls -1 /opt/ros/*/setup.bash 2>/dev/null | sort | tail -n1 || true
  fi
}

# One confirmation for both platforms; the install is large either way, and the
# question has to be asked before anything is downloaded.
confirm_ros_install() {
  [[ "${install_ros}" == ask ]] || return 0
  # No prompt without a terminal: run_all.sh and remote_ubuntu.sh reach this
  # over ssh, and a 2 GB install is not something to start unattended.
  if [[ ! -t 0 ]]; then
    echo "ROS 2 not found." >&2
    echo "$1" >&2
    echo "This script will not start that unattended. Run it from a terminal," >&2
    echo "or pass:" >&2
    echo "  ./scripts/build_ros.sh --install-ros" >&2
    return 1
  fi
  echo "ROS 2 not found."
  echo "$1"
  local reply
  read -r -p "Install it now? [y/N] " reply
  if [[ ! "${reply}" =~ ^[Yy]$ ]]; then
    echo "Not installing. See docs/VISUALIZATION.md for the manual steps." >&2
    return 1
  fi
}

install_ros2_apt() {
  local codename
  codename="$(. /etc/os-release && echo "${VERSION_CODENAME}")"
  confirm_ros_install "Installing it for Ubuntu ${codename} uses sudo, adds an apt source, and
downloads roughly 2 GB (ros-*-desktop plus colcon)." || return 1

  if [[ ! -f /etc/apt/sources.list.d/ros2.list ]]; then
    sudo apt-get update
    sudo apt-get install -y --no-install-recommends \
      curl gnupg ca-certificates software-properties-common
    sudo add-apt-repository -y universe
    sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${codename} main" \
      | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  fi
  sudo apt-get update

  # The distribution is not hardcoded: packages.ros.org carries exactly one per
  # Ubuntu release, so ask apt which it is. 24.04 answers jazzy, 26.04 lyrical.
  # A table here would be one more thing to update every two years.
  local desktop
  desktop="$(apt-cache search --names-only '^ros-[a-z]+-desktop$' \
             | awk '{print $1}' | sort | head -n1)"
  if [[ -z "${desktop}" ]]; then
    echo "packages.ros.org has no ros-*-desktop for Ubuntu ${codename}." >&2
    echo "That release may not have a ROS 2 distribution yet." >&2
    return 1
  fi

  echo "Installing ${desktop} and colcon ..."
  sudo apt-get install -y "${desktop}" python3-colcon-common-extensions
}

install_ros2_macos() {
  confirm_ros_install "Installing it downloads roughly 2 GB from conda-forge into
  ${ROS_ENV}
(RoboStack's ros-*-desktop plus colcon). It needs no sudo, and touches nothing
outside that directory and Homebrew's micromamba." || return 1

  if ! command -v brew >/dev/null 2>&1; then
    echo "Homebrew not found. Install it from https://brew.sh, then re-run." >&2
    return 1
  fi
  # micromamba rather than a full conda: one static binary, and Homebrew has it.
  command -v micromamba >/dev/null 2>&1 || brew install micromamba

  # Newest first. RoboStack publishes one channel per generation rather than one
  # per Ubuntu release, so unlike the apt path there is nothing to interrogate --
  # try what exists and take the first that resolves.
  local distro
  for distro in jazzy humble; do
    echo "Trying ros-${distro}-desktop from robostack-staging ..."
    if micromamba create -y -p "${ROS_ENV}" \
         -c conda-forge -c robostack-staging \
         "ros-${distro}-desktop" colcon-common-extensions; then
      return 0
    fi
    # A partial prefix would make resolve_ros_setup claim success next run.
    rm -rf "${ROS_ENV}"
  done

  echo "RoboStack has no ros-{jazzy,humble}-desktop for $(uname -m) macOS." >&2
  echo "See https://robostack.github.io for what it currently publishes." >&2
  return 1
}

install_ros2() {
  if [[ "$(uname -s)" == Darwin ]]; then
    install_ros2_macos
  elif command -v apt-get >/dev/null 2>&1; then
    install_ros2_apt
  else
    echo "ROS 2 is missing, and this is neither macOS nor an apt system, so this" >&2
    echo "script cannot install it. See https://docs.ros.org for your platform." >&2
    return 1
  fi
}

ros_setup="$(resolve_ros_setup)"
if [[ -z "${ros_setup}" ]]; then
  if [[ "${install_ros}" == no ]]; then
    echo "ROS 2 not found, and --no-install-ros was given." >&2
    echo "See docs/VISUALIZATION.md." >&2
    exit 1
  fi
  install_ros2 || exit 1
  ros_setup="$(resolve_ros_setup)"
  if [[ -z "${ros_setup}" ]]; then
    echo "ROS 2 still not found after installing." >&2
    exit 1
  fi
fi

# On Linux the distro's setup.bash puts its own bin on PATH. A RoboStack prefix
# does not: colcon, python and ros2 all live in ${prefix}/bin, and nothing has
# activated the environment, so put it there before sourcing.
if [[ "$(uname -s)" == Darwin ]]; then
  export PATH="$(dirname "${ros_setup}")/bin:${PATH}"
fi
# ROS setup scripts reference unset variables (e.g. AMENT_TRACE_SETUP_FILES); nounset trips on
# them, so disable it only around sourcing.
set +u
# shellcheck disable=SC1090
source "${ros_setup}"
set -u
echo "Using ROS 2 distro: ${ROS_DISTRO:-unknown}"

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon not found. ./scripts/build_ros.sh --install-ros installs it, or:" >&2
  if [[ "$(uname -s)" == Darwin ]]; then
    echo "  micromamba install -p \"${ROS_ENV}\" -c conda-forge colcon-common-extensions" >&2
  else
    echo "  sudo apt install python3-colcon-common-extensions" >&2
  fi
  exit 1
fi

cd "${WS}"
colcon build --packages-select cam_loc_ros \
  --cmake-args \
  -DCAMLOC_ROOT="${ROOT}" \
  -DCAMLOC_BUILD_DIR="${BUILD}"

echo ""
echo "Build complete. Source: source ${WS}/install/setup.bash"
