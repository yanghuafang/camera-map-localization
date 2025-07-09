#!/usr/bin/env bash
# remote_ubuntu.sh — run any of these scripts on the Ubuntu box.
#
# This project claims Linux and macOS, and one of those claims cannot be
# checked from the other: CUDA has no macOS toolchain, so `nvcc`, the GPU
# kernels and every `CudaTest` are unreachable from a Mac. `--cuda-host` gets
# the host-side code compiled anywhere, but whether the kernels are *correct*
# can only be answered on a machine with a GPU. Rather than push a branch and
# wait for someone else's runner to disagree, this runs a command on the Linux
# host, optionally mirroring the working tree -- uncommitted edits included --
# first.
#
#   ./scripts/remote_ubuntu.sh ./scripts/ci.sh --cuda           # run what is there
#   ./scripts/remote_ubuntu.sh --sync ./scripts/ci.sh --cuda    # copy first, then run
#   ./scripts/remote_ubuntu.sh --sync                           # copy and stop
#   ./scripts/remote_ubuntu.sh --clone ./scripts/ci.sh          # clone, then run
#   ./scripts/remote_ubuntu.sh --shell 'nvidia-smi | head -5'   # a one-off probe
#
# ...or the same from inside scripts/, which is how these are usually run:
#
#   ./remote_ubuntu.sh --sync
#   ./remote_ubuntu.sh ./build_ros.sh
#
# Copying is opt-in rather than the default because it is the only step that
# destroys anything: it is rsync --delete against the remote checkout, so
# whatever is there is made to match this machine exactly. A command that only
# builds or reads should not have to think about that.
#
# --sync and --clone answer the same question -- where does the remote tree
# come from -- with different answers, so asking for both is a contradiction
# rather than a sequence, and is refused. --sync sends what is on this machine,
# uncommitted work and all; --clone fetches what is pushed to GitHub, which is
# the honest way to check that what was committed is what actually builds.
#
# The command runs from wherever you are, mirrored: the remote working
# directory is the same path relative to the repository root as the local one.
# Run `./remote_ubuntu.sh ./build_ros.sh` from scripts/ and it runs
# scripts/build_ros.sh over there. Anchoring at the repository root instead
# would mean the same command line meaning two different things depending on
# which side you typed it, and `./build_ros.sh` would simply not be found. A
# cwd outside the repository falls back to the remote root.
#
# --sync is also the answer when the host cannot reach GitHub: it moves the tree
# over ssh from this machine, so nothing on the host has to clone anything. Pair
# it with install_deps_ubuntu.sh, which takes the C++ libraries from apt, and the
# host builds without touching the network at all.
set -euo pipefail

REMOTE_HOST="${CAMLOC_REMOTE_HOST:-yanghuafang@192.168.10.13}"
REMOTE_DIR="${CAMLOC_REMOTE_DIR:-study-projects/camera-map-localization}"
REPO_URL="${CAMLOC_REPO_URL:-https://github.com/yanghuafang/camera-map-localization.git}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

usage() {
  cat <<'EOF'
Usage: remote_ubuntu.sh [--sync | --clone] [--shell] [command ...]

Run a command on the Ubuntu host, optionally putting a tree there first. The
reason this exists is CUDA: it cannot be built or tested on macOS at all.

  remote_ubuntu.sh ./scripts/ci.sh --cuda         Run; copy nothing.
  remote_ubuntu.sh --sync ./scripts/ci.sh --cuda  Copy this tree over, then run.
  remote_ubuntu.sh --sync                         Copy this tree over and stop.
  remote_ubuntu.sh --clone ./scripts/ci.sh        Clone from GitHub, then run.
  remote_ubuntu.sh --shell 'nvidia-smi'           Run shell text, not an argv.

The remote command runs in the directory matching this one, so from inside
scripts/ the paths are the ones you would type locally:

  ./remote_ubuntu.sh --sync
  ./remote_ubuntu.sh ./build_ros.sh

Options:
  --sync      Mirror this working tree to the host before running. This is
              rsync --delete: the remote checkout is made to match this one, so
              anything edited only on the host is lost. Build directories and
              data/ are never sent -- see the header comment.
  --clone     git clone the repository onto the host instead. Refuses to
              overwrite an existing checkout. Mutually exclusive with --sync.
  --shell     Treat the argument as shell text rather than a list of
              arguments, so pipes and semicolons work.
  -h, --help  Show this help.

Environment:
  CAMLOC_REMOTE_HOST  user@host to reach
                      (default: yanghuafang@192.168.10.13).
  CAMLOC_REMOTE_DIR   Checkout path on the host, relative to its home directory
                      (default: study-projects/camera-map-localization,
                      mirroring the macOS layout). --sync deletes whatever else
                      lives there, so give it a path of its own.
  CAMLOC_REPO_URL     What --clone clones. Set it to
                      git@github.com:yanghuafang/camera-map-localization.git to
                      clone over ssh, which uses the forwarded agent from this
                      machine.
EOF
}

do_sync=false
do_clone=false
as_shell=false

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sync) do_sync=true; shift ;;
    --clone) do_clone=true; shift ;;
    --shell) as_shell=true; shift ;;
    -h|--help) usage; exit 0 ;;
    --) shift; break ;;
    *) break ;;
  esac
done

if [[ "${do_sync}" == true && "${do_clone}" == true ]]; then
  echo "--sync and --clone are mutually exclusive: one sends this working" >&2
  echo "tree, the other fetches what is pushed to GitHub. Pick one." >&2
  exit 1
fi

# Nothing to run and nothing to put there is a mistake worth naming rather than
# a silent success. Checked before any transfer, so a typo costs nothing.
if [[ $# -eq 0 && "${do_sync}" == false && "${do_clone}" == false ]]; then
  echo "Nothing to do: give a command, or --sync/--clone to place a tree." >&2
  usage >&2
  exit 1
fi

if [[ "${do_clone}" == true ]]; then
  # git clone into an existing directory fails anyway, but it fails after the
  # connection with a message about the destination not being empty. Checking
  # first says the useful thing: which directory, and what to do about it. It is
  # deliberately not resolved by deleting anything -- that checkout may be the
  # only copy of something.
  if ssh "${REMOTE_HOST}" "[ -e ${REMOTE_DIR} ]"; then
    echo "${REMOTE_HOST}:${REMOTE_DIR} already exists; refusing to clone over it." >&2
    echo "Remove it on the host, or point CAMLOC_REMOTE_DIR somewhere else." >&2
    exit 1
  fi
  echo "Cloning ${REPO_URL} -> ${REMOTE_HOST}:${REMOTE_DIR}/"
  # -A forwards this machine's SSH agent, so an ssh-form CAMLOC_REPO_URL
  # authenticates with the key that already reaches GitHub from here and the
  # host needs none of its own. It is scoped to the clone rather than the whole
  # script because while connected, the host can use that agent.
  if ! ssh -A "${REMOTE_HOST}" \
      "mkdir -p \"\$(dirname '${REMOTE_DIR}')\" && git clone '${REPO_URL}' '${REMOTE_DIR}'"; then
    echo "" >&2
    echo "Clone failed. The three causes seen here, with different fixes:" >&2
    echo "  * No route to github.com from the host. Use --sync instead: it" >&2
    echo "    sends this working tree over ssh and never touches GitHub." >&2
    echo "  * 'Permission denied (publickey)' with an ssh CAMLOC_REPO_URL --" >&2
    echo "    no key in the forwarded agent. Run 'ssh-add' here, check with" >&2
    echo "    'ssh-add -l'." >&2
    echo "  * git not installed on the host. Run scripts/install_deps_ubuntu.sh" >&2
    echo "    there first." >&2
    exit 1
  fi
fi

if [[ "${do_sync}" == true ]]; then
  echo "Syncing ${ROOT}/ -> ${REMOTE_HOST}:${REMOTE_DIR}/"
  ssh "${REMOTE_HOST}" "mkdir -p ${REMOTE_DIR}"
  # --delete so a file deleted here does not linger and keep building there.
  #
  # The exclusions are the things that must not cross. .git/ because this
  # machine is the source of truth for history. build*/ because a CMake cache
  # records absolute paths and compiler identities from the host that wrote it,
  # so sending one produces a configure that disagrees with the machine it lands
  # on. The colcon workspace holds the same and lives beside the repository
  # too, so it never enters this transfer at all.
  #
  # Datasets need no exclusion: they live in ../<repo>-data, outside the tree
  # this syncs, which is one of the reasons they are there. /data/ is still named
  # in case an app was run by hand from the repository root.
  #
  # The leading slash anchors each pattern to the transfer root, and it is load
  # bearing: an unanchored 'data/' matches a directory of that name at any
  # depth, so it also excludes tests/data/ -- the committed fixtures every smoke
  # run reads. rsync protects excluded files from --delete, so the far side
  # keeps whatever it had and the breakage only shows on a host that never
  # received them: prepare_smoke_kitti.sh fails on a missing calib_minimal.txt.
  rsync -az --delete \
    --exclude '/.git/' \
    --exclude '/.claude/' \
    --exclude '/build/' \
    --exclude '/build-*/' \
    --exclude '/data/' \
    --exclude '.DS_Store' \
    --exclude '._*' \
    "${ROOT}/" "${REMOTE_HOST}:${REMOTE_DIR}/"
fi

# --sync or --clone with no command is the "just put it there" case.
if [[ $# -eq 0 ]]; then
  exit 0
fi

# Two different things are being sent, and they need opposite treatment. A
# command is a list of arguments: %q escapes each one so a path with a space
# survives the two shells this crosses. --shell is a snippet the caller wrote to
# be interpreted -- its pipes and semicolons are the point -- so quoting it
# would turn the whole line into one nonexistent filename.
if [[ "${as_shell}" == true ]]; then
  remote_cmd="$*"
else
  remote_cmd="$(printf '%q ' "$@")"
fi

# Ask for a remote TTY only when there is a local one to mirror, so a build run
# from a terminal keeps its progress output live, and one run from a script does
# not open with ssh complaining that stdin is not a terminal.
tty_flag=()
if [[ -t 0 ]]; then
  tty_flag=(-t)
fi

# A login shell, so the command starts from the PATH the host's profile builds.
# `ssh host cmd` runs a shell that is neither login nor interactive: the profile
# is never read, and Ubuntu's stock ~/.bashrc returns on its first line for
# exactly that case. That matters here because CUDA installs to /usr/local/cuda
# and puts nvcc on PATH from a profile snippet, so without this the whole
# purpose of the script fails as "nvcc: command not found".
#
# The +"..." guard is for the empty case: macOS still ships bash 3.2, where
# expanding an empty array under `set -u` is an unbound-variable error rather
# than nothing at all.
# The remote working directory mirrors this one; see the header. `pwd -P` so a
# symlinked path still compares against ROOT, which is also physical.
remote_wd="${REMOTE_DIR}"
local_wd="$(pwd -P)"
case "${local_wd}" in
  "${ROOT}") ;;
  "${ROOT}"/*) remote_wd="${REMOTE_DIR}/${local_wd#"${ROOT}"/}" ;;
  *) ;;  # outside the repository: nothing to mirror, use the remote root
esac

# `cd X || exit 1` on its own line, not `cd X && cmd`. With && and a --shell
# snippet holding more than one statement, only the first is guarded: the rest
# would run in the home directory if the cd failed.
exec ssh ${tty_flag[@]+"${tty_flag[@]}"} "${REMOTE_HOST}" \
  "bash -lc 'cd ${remote_wd} || exit 1
${remote_cmd}'"
