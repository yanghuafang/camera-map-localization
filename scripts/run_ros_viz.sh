#!/usr/bin/env bash
# Launch cam_loc ROS RViz visualization on smoke KITTI.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS="${ROOT}/ros/ws"

"${ROOT}/scripts/prepare_smoke_kitti.sh" 120

# Rebuild unless the node executable is actually installed (a failed colcon build still leaves
# install/setup.bash, so checking that alone is not enough).
if [[ ! -x "${WS}/install/cam_loc_ros/lib/cam_loc_ros/cam_loc_viz_node" ]]; then
  "${ROOT}/scripts/build_ros.sh"
fi

# ROS/colcon setup scripts reference unset variables (e.g. COLCON_TRACE); disable nounset
# only around sourcing.
set +u
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u

export CAMLOC_SMOKE="${ROOT}/data/smoke_kitti"

ros2 launch cam_loc_ros cam_loc_viz.launch.py \
  kitti_root:="${CAMLOC_SMOKE}" \
  perception_mode:=oracle \
  use_gt_plane:=true \
  playback_hz:=5.0
