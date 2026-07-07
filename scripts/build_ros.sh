#!/usr/bin/env bash
# Build cam_loc_ros against the main camera-map-localization CMake build.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
WS="${ROOT}/ros/ws"

if [[ ! -f "${ROOT}/build/src/libcam_loc_core.a" ]]; then
  echo "Building cam_loc_core first..."
  cmake -S "${ROOT}" -B "${ROOT}/build" -DCAMLOC_BUILD_CUDA=ON -DCAMLOC_BUILD_TESTS=ON
  cmake --build "${ROOT}/build" -j"$(camloc_nproc)"
fi

mkdir -p "${WS}/src"
if [[ ! -e "${WS}/src/cam_loc_ros" ]]; then
  ln -sfn "${ROOT}/ros/cam_loc_ros" "${WS}/src/cam_loc_ros"
fi

# Source ROS 2: prefer an already-sourced distro, else the newest distro under /opt/ros.
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  ros_setup="/opt/ros/${ROS_DISTRO}/setup.bash"
else
  ros_setup="$(ls -1 /opt/ros/*/setup.bash 2>/dev/null | sort | tail -n1 || true)"
fi
if [[ -z "${ros_setup}" ]]; then
  echo "ROS 2 not found under /opt/ros. Install a ROS 2 distro and source its setup.bash." >&2
  exit 1
fi
# ROS setup scripts reference unset variables (e.g. AMENT_TRACE_SETUP_FILES); nounset trips on
# them, so disable it only around sourcing.
set +u
# shellcheck disable=SC1090
source "${ros_setup}"
set -u
echo "Using ROS 2 distro: ${ROS_DISTRO:-unknown}"

if ! command -v colcon >/dev/null 2>&1; then
  echo "colcon not found. Install it, e.g.: sudo apt install python3-colcon-common-extensions" >&2
  exit 1
fi

cd "${WS}"
colcon build --packages-select cam_loc_ros \
  --cmake-args \
  -DCAMLOC_ROOT="${ROOT}" \
  -DCAMLOC_BUILD_DIR="${ROOT}/build"

echo ""
echo "Build complete. Source: source ${WS}/install/setup.bash"
