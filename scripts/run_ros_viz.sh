#!/usr/bin/env bash
# Launch cam_loc ROS RViz visualization on smoke KITTI.
#
# `ros2 launch` runs until it is interrupted, so by default this waits for you
# to close RViz. --duration bounds it instead, which is what run_all.sh uses:
# a script whose output is a pass/fail summary cannot sit waiting for a window
# to be closed.
#
# Usage:
#   ./scripts/run_ros_viz.sh                # play until you close RViz
#   ./scripts/run_ros_viz.sh --duration 60  # stop after 60 seconds
set -euo pipefail

duration=0
while [[ $# -gt 0 ]]; do
  case "$1" in
    --duration)
      duration="${2:-}"
      if [[ ! "${duration}" =~ ^[0-9]+$ ]]; then
        echo "--duration takes a whole number of seconds" >&2
        exit 1
      fi
      shift
      ;;
    -h|--help)
      sed -n '2,11p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
    *) echo "Unknown option: $1" >&2; exit 1 ;;
  esac
  shift
done

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
DATA="$(camloc_data_dir "${ROOT}")"
WS="$(camloc_ros_dir "${ROOT}")/ws"

"${ROOT}/scripts/prepare_smoke_kitti.sh" 120

# Always, rather than only when the executable is missing. colcon is
# incremental -- a no-op build is well under a second -- and the version that
# skipped this whenever a binary existed meant an edited source silently ran as
# the old binary. That is a bad way to spend an afternoon: the symptom is a fix
# that appears to do nothing, and it looks like a bug in the fix.
"${ROOT}/scripts/build_ros.sh"

# ROS/colcon setup scripts reference unset variables (e.g. COLCON_TRACE); disable nounset
# only around sourcing.
set +u
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u

export CAMLOC_SMOKE="${DATA}/smoke_kitti"

launch=(ros2 launch cam_loc_ros cam_loc_viz.launch.py
        kitti_root:="${CAMLOC_SMOKE}"
        perception_mode:=oracle
        use_gt_plane:=true
        playback_hz:=5.0)

if [[ "${duration}" -le 0 ]]; then
  exec "${launch[@]}"
fi

# Bounded run. `ros2 launch` runs until interrupted and starts each node in a
# session of its own, so neither signalling our process group nor killing the
# launch takes the nodes with it -- rviz2 outlived the deadline twice here, and
# had to be closed by hand while the script had already printed its summary.
#
# What does work is the launch log: it names the pid of every process it
# started, so the deadline ends exactly those and nothing else.
echo "Playing for ${duration}s, then stopping (--duration ${duration})."

# Redirected to a file rather than piped into tee: in a pipeline $! is the last
# command, so the timer below would signal tee and leave the launch running.
# The log is also what the pid sweep and the death check below read. It is
# printed in full when the run ends.
launch_log="$(mktemp)"

# Pids ros2 launch reports for the nodes it started.
launch_children() {
  sed -n 's/.*process started with pid \[\([0-9]*\)\].*/\1/p' "${launch_log}" \
    2>/dev/null
}

end_launch_children() {
  local pid
  for pid in $(launch_children); do
    kill -s TERM "${pid}" 2>/dev/null || true
  done
  sleep 3
  for pid in $(launch_children); do
    kill -s KILL "${pid}" 2>/dev/null || true
  done
}

started=${SECONDS}
"${launch[@]}" >"${launch_log}" 2>&1 &
launch_pid=$!

(
  sleep "${duration}"
  # SIGINT first: that is the documented clean shutdown, and it lets RViz save
  # nothing and exit tidily. Give it room -- RViz takes a few seconds.
  kill -s INT "${launch_pid}" 2>/dev/null || exit 0
  for _ in $(seq 1 15); do
    kill -0 "${launch_pid}" 2>/dev/null || exit 0
    sleep 1
  done
  end_launch_children
  kill -s KILL "${launch_pid}" 2>/dev/null || true
) &
timer_pid=$!

status=0
wait "${launch_pid}" || status=$?
kill "${timer_pid}" 2>/dev/null || true
wait "${timer_pid}" 2>/dev/null || true

# Unconditional, because ros2 launch exiting is not the same as its nodes
# exiting. This is the sweep that actually closes the window.
end_launch_children

cat "${launch_log}"

# Reaching the deadline is the asked-for outcome, not a failure. Anything that
# stops earlier kept its own exit status, so a crash still reads as one.
if [[ $((SECONDS - started)) -ge ${duration} ]]; then
  status=0
fi

# ros2 launch exits 0 even when a node it started died, so the deadline above
# would otherwise report a clean run of an empty RViz. That is exactly how a
# missing rpath hid here: cam_loc_viz_node aborted on a dyld error two seconds
# in, RViz sat there with no data, and the step passed.
if grep -q "process has died" "${launch_log}"; then
  echo "" >&2
  echo "A launch process died; ros2 launch does not fail the run for it:" >&2
  grep "process has died" "${launch_log}" >&2
  status=1
fi
rm -f "${launch_log}"
exit "${status}"
