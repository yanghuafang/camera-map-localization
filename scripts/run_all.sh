#!/usr/bin/env bash
# run_all.sh — run every other script in scripts/, in dependency order.
#
# Why this exists: an ssh session has no display. run_ros_viz.sh opens RViz and
# build_ros.sh builds the node it needs, so those two can only run from a
# desktop session on the machine itself -- this is the script to run there,
# whichever machine that is.
#
# It runs on both macOS and Ubuntu. The plan adapts in three places -- which
# install_deps script to run, whether ci.sh can reach a real nvcc, and where
# ROS 2 lives -- and is otherwise identical, which is the point: the same steps
# should hold on both.
#
# A failing step is recorded and the run continues. One broken step should not
# hide the state of all the others, and the summary at the end is the point
# of running this rather than the scripts by hand. Each step also writes its own
# log under <repo>-data/run_all_logs/, named in the summary for what failed.
#
# Order is dependency order, not alphabetical: prepare_smoke_kitti.sh writes the
# sequence the run_* scripts read, ci.sh writes the compile database tidy.sh
# reads, and download_kitti_odometry.sh fetches what run_real_kitti.sh needs.
#
# Every script in scripts/ is either in PLAN or in NOT_RUN with a reason, and
# the run aborts if a new one appears in neither -- so adding a script forces a
# decision about it rather than silently leaving it untested.
#
# Usage:
#   ./scripts/run_all.sh                # everything this machine can do
#   ./scripts/run_all.sh --list         # print the plan and the skips, run nothing
#   ./scripts/run_all.sh --no-deps      # skip this platform's install_deps script
#   ./scripts/run_all.sh --no-download  # skip the two dataset downloads (~174 MB)
#   ./scripts/run_all.sh --no-viz       # skip RViz (it is bounded at 60s otherwise)
#   ./scripts/run_all.sh --install-ros  # let build_ros.sh install ROS 2 (~2 GB)

# Deliberately no -e: the loop below inspects each step's exit status itself.
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
DATA="$(camloc_data_dir "${ROOT}")"

usage() {
  cat <<'EOF'
Usage: run_all.sh [--list] [--no-deps] [--no-download] [--no-viz]

Run every script in scripts/ in dependency order, continuing past failures, and
print a pass/fail/skip summary. Intended to be run on the machine itself: the
ROS visualization needs a display, which an ssh session does not have.

Options:
  --list         Print the plan, with the reason for anything that would be
                 skipped, and exit without running it.
  --no-deps      Skip install_deps_macos.sh / install_deps_ubuntu.sh. Use when
                 the packages are already installed, or when you do not want a
                 sudo prompt.
  --no-download  Skip download_kitti_odometry.sh and
                 download_semantic_kitti_labels.sh (~174 MB together). The steps
                 that need that data then skip or fall back on their own.
  --no-viz       Skip run_ros_viz.sh entirely. It is bounded at 60s here, so
                 it ends on its own; use this to skip the window as well.
  --install-ros  Let build_ros.sh install ROS 2 if it is missing (~2 GB; sudo
                 on Ubuntu, RoboStack via micromamba on macOS).
                 Without this the two ROS steps skip when it is absent, since
                 an install that large should be asked for, not assumed.
  -h, --help     Show this help.
EOF
}

run_deps=true
run_download=true
run_viz=true
install_ros=false
list_only=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --list) list_only=true ;;
    --no-deps) run_deps=false ;;
    --no-download) run_download=false ;;
    --no-viz) run_viz=false ;;
    --install-ros) install_ros=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
  shift
done

# build_ros.sh installs ROS 2 itself when asked; otherwise tell it not to, so a
# missing ROS 2 is a clean failure rather than an interactive prompt.
if [[ "${install_ros}" == true ]]; then ros_flag=--install-ros; else ros_flag=--no-install-ros; fi

# The three platform differences, resolved once. Two plans would drift; two
# entries in one plan cannot.
if [[ "$(uname -s)" == Darwin ]]; then
  deps_script=install_deps_macos.sh
  deps_note="Homebrew packages, including the llvm that supplies clang-format and clang-tidy"
  other_deps="install_deps_ubuntu.sh|Ubuntu only"
  # No nvcc on macOS. --cuda-host is the closest equivalent: it compiles
  # everything inside #ifdef CAMLOC_CUDA_ENABLED against the CPU stub, so the
  # GPU host paths are still type-checked here even though no kernel runs.
  ci_args=--cuda-host
  ci_note="configure, build, 53 tests, smoke benchmark, clang-tidy -- CUDA host paths against the CPU stub"
else
  deps_script=install_deps_ubuntu.sh
  deps_note="apt packages, including the clang and llvm that coverage.sh needs; prompts for sudo"
  other_deps="install_deps_macos.sh|macOS only"
  ci_args=--cuda
  ci_note="configure, build, 53 tests, smoke benchmark, clang-tidy -- with the GPU kernels"
fi

# script | arguments | what it is for
PLAN=(
  "${deps_script}||${deps_note}"
  "format.sh|--check|formatting gate; no build, so it fails in seconds"
  "prepare_smoke_kitti.sh|120|the synthetic KITTI sequence every run_* step below reads"
  "ci.sh|${ci_args}|${ci_note}"
  "tidy.sh||clang-tidy on its own, against the compile database ci.sh just wrote"
  "ci.sh|--asan --ubsan|the same tests under ASan and UBSan, in their own build directory"
  "coverage.sh||instrumented rebuild, then a line and function report over src/"
  "docs.sh||Doxygen API reference; fails on any Doxygen warning"
  "run_smoke.sh||run_sequence over the synthetic sequence, CPU and CUDA"
  "run_benchmark.sh||accuracy cases plus the CPU/GPU micro-benchmarks"
  "run_viz_smoke.sh||PNG debug frames; writes files, so it needs no display"
  "download_kitti_odometry.sh||real KITTI Odometry poses and calibration, ~2 MB"
  "run_real_kitti.sh||eval_sequence over real KITTI sequence 00"
  "download_semantic_kitti_labels.sh||SemanticKITTI labels, ~172 MB"
  "run_perception_eval.sh||wants velodyne scans (~80 GB, not downloaded here); falls back to run_real_kitti"
  "run_perception_tuning.sh||oracle against real and noisy perception on sequence 00"
  "build_ros.sh|${ros_flag}|colcon build of cam_loc_ros"
  "run_ros_viz.sh|--duration 60|RViz playback, bounded at 60s so the summary is not held up by an open window"
)

# Scripts deliberately not run here, and why. A plain array rather than an
# associative one: macOS ships bash 3.2, and --list is worth having there.
NOT_RUN=(
  "lib.sh|sourced by the others; not a program"
  "${other_deps}"
  "remote_ubuntu.sh|drives the other machine over ssh; this script is about the one it runs on"
  "run_all.sh|this script"
)

# A script in neither list is a script nobody decided about.
plan_scripts="$(printf '%s\n' "${PLAN[@]}" | cut -d'|' -f1)"
skipped_scripts="$(printf '%s\n' "${NOT_RUN[@]}" | cut -d'|' -f1)"
for path in "${ROOT}"/scripts/*.sh; do
  name="$(basename "${path}")"
  if ! grep -qx "${name}" <<<"${plan_scripts}" &&
     ! grep -qx "${name}" <<<"${skipped_scripts}"; then
    echo "run_all.sh does not know about scripts/${name}." >&2
    echo "Add it to PLAN, or to NOT_RUN with the reason it is left out." >&2
    exit 2
  fi
done

# Empty output means run it; any other output is the reason it is skipped.
skip_reason() {
  local ros_setup
  case "$1" in
    install_deps_macos.sh|install_deps_ubuntu.sh)
      # apt needs sudo, and sudo over ssh has no terminal to authenticate on.
      # That is the environment being unable to run the step, not the step
      # failing, so it skips the way run_ros_viz.sh does without a display.
      # Homebrew needs no sudo, so macOS never reaches this.
      if [[ "${run_deps}" != true ]]; then
        echo "--no-deps"
      elif [[ "$1" == install_deps_ubuntu.sh ]] && ! sudo -n true 2>/dev/null; then
        echo "sudo needs a password and this session has no terminal -- run it on the machine"
      fi
      ;;
    download_semantic_kitti_labels.sh)
      if [[ "${run_download}" != true ]]; then
        echo "--no-download"
      elif [[ ! -f "${DATA}/kitti_odometry/dataset/sequences/00/calib.txt" ]]; then
        echo "needs the KITTI Odometry calibration, which the earlier download did not leave"
      fi
      ;;
    download_kitti_odometry.sh)
      [[ "${run_download}" == true ]] || echo "--no-download"
      ;;
    build_ros.sh|run_ros_viz.sh)
      # Where build_ros.sh put it: a micromamba prefix beside the repository
      # on macOS, a distro under /opt/ros on Ubuntu.
      if [[ "$(uname -s)" == Darwin ]]; then
        ros_setup="$(camloc_ros_dir "${ROOT}")/env/setup.bash"
        [[ -f "${ros_setup}" ]] || ros_setup=""
      else
        ros_setup="$(ls -1 /opt/ros/*/setup.bash 2>/dev/null | head -n1)"
      fi
      if [[ -z "${ros_setup}" && "${install_ros}" != true ]]; then
        echo "ROS 2 not installed -- pass --install-ros, or see docs/VISUALIZATION.md"
      elif [[ "$1" == run_ros_viz.sh ]]; then
        if [[ "${run_viz}" != true ]]; then
          echo "--no-viz"
        elif [[ "$(uname -s)" == Darwin ]]; then
          # Qt talks to Quartz here, so there is no DISPLAY to test. The case
          # that cannot work is an ssh session, which gets no window server.
          [[ -z "${SSH_CONNECTION:-}" ]] ||
            echo "over ssh -- RViz needs the machine's own desktop session"
        elif [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
          echo "no display -- run this from a desktop session on the machine, not over ssh"
        fi
      fi
      ;;
  esac
}

if [[ "${list_only}" == true ]]; then
  printf 'Plan (%d steps):\n\n' "${#PLAN[@]}"
  n=0
  for entry in "${PLAN[@]}"; do
    IFS='|' read -r script args note <<<"${entry}"
    n=$((n + 1))
    reason="$(skip_reason "${script}")"
    printf '%2d. %-34s %s\n' "${n}" "${script} ${args}" "${note}"
    [[ -n "${reason}" ]] && printf '    SKIP: %s\n' "${reason}"
  done
  printf '\nNot run:\n'
  for entry in "${NOT_RUN[@]}"; do
    IFS='|' read -r name why <<<"${entry}"
    printf '    %-34s %s\n' "${name}" "${why}"
  done
  printf '\nSkip reasons are evaluated now, so a step whose input an earlier step\n'
  printf 'produces may read as skipped here and still run.\n'
  exit 0
fi

LOG_DIR="${DATA}/run_all_logs"
rm -rf "${LOG_DIR}"
mkdir -p "${LOG_DIR}"

echo "Logs: ${LOG_DIR}"
echo "Steps: ${#PLAN[@]}"

results=()
failures=0
step=0
for entry in "${PLAN[@]}"; do
  IFS='|' read -r script args note <<<"${entry}"
  step=$((step + 1))

  reason="$(skip_reason "${script}")"
  if [[ -n "${reason}" ]]; then
    printf '\n[%2d/%d] SKIP  %s -- %s\n' "${step}" "${#PLAN[@]}" "${script}" "${reason}"
    results+=("SKIP|${script}|${reason}")
    continue
  fi

  printf '\n[%2d/%d] ---- %s %s\n        %s\n\n' \
    "${step}" "${#PLAN[@]}" "${script}" "${args}" "${note}"

  log="${LOG_DIR}/$(printf '%02d' "${step}")-${script%.sh}.log"
  started=${SECONDS}
  # Unquoted on purpose: args is either empty or one flag.
  # shellcheck disable=SC2086
  "${ROOT}/scripts/${script}" ${args} 2>&1 | tee "${log}"
  status=${PIPESTATUS[0]}
  elapsed=$((SECONDS - started))

  if [[ ${status} -eq 0 ]]; then
    results+=("PASS|${script}|${elapsed}s")
  else
    results+=("FAIL|${script}|exit ${status} after ${elapsed}s, log: ${log#"${ROOT}"/}")
    failures=$((failures + 1))
  fi
done

printf '\n======== summary ========\n'
for result in "${results[@]}"; do
  IFS='|' read -r state script detail <<<"${result}"
  printf '  %-4s  %-34s %s\n' "${state}" "${script}" "${detail}"
done

printf '\n%d passed, %d failed, %d skipped. Logs in %s\n' \
  "$(grep -c '^PASS' <<<"$(printf '%s\n' "${results[@]}")")" \
  "${failures}" \
  "$(grep -c '^SKIP' <<<"$(printf '%s\n' "${results[@]}")")" \
  "${LOG_DIR#"${ROOT}"/}"

[[ ${failures} -eq 0 ]]
