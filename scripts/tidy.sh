#!/usr/bin/env bash
# tidy.sh — run clang-tidy over cam_loc's own translation units.
#
# Needs a compile database; any configure writes build/compile_commands.json.
#
# The file list is the database entries under src/, apps/ and tests/. Both
# halves of that matter:
#   - from the database, because src/cuda/distance_transform_gpu.cc is compiled
#     only when CAMLOC_BUILD_CUDA finds nvcc. Handed a file it has no command
#     for, clang-tidy guesses one from a sibling entry and reports a missing
#     cuda_runtime.h plus a cascade of unreal parse errors.
#   - under the source roots, to keep the list to code this repository is
#     responsible for, whatever else a build directory happens to compile.
#
# ros/cam_loc_ros is built by colcon against an installed cam_loc_core, so it
# never appears here; analyzing it needs its own compile database.
#
# Exits 1 on findings, and separately on a run that could not analyze what it
# was asked to — findings go to stdout, so "clean" and "analyzed nothing" look
# identical there, and the second is what matters in CI.
#
# Usage:
#   ./scripts/tidy.sh
#   ./scripts/tidy.sh --fix                         # apply fixes, then reformat
#   ./scripts/tidy.sh src/core/cost_aggregator.cc   # limit to specific files
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
BUILD="$(camloc_build_dir "${ROOT}")"

usage() {
  cat <<'EOF'
Usage: tidy.sh [--fix] [file...]

Run clang-tidy against the curated check list in .clang-tidy. With no files,
analyzes every cam_loc translation unit in the compile database.

Options:
  --fix       Apply clang-tidy's automatic fixes, then re-run format.sh.
  -h, --help  Show this help.

Environment:
  CAMLOC_BUILD_DIR   Build directory holding compile_commands.json (default:
                     ../<repo>-build, beside the repository).
  CAMLOC_CLANG_TIDY  clang-tidy binary to use (default: the Homebrew llvm keg
                     on macOS, else PATH).
EOF
}

fix=false
files=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --fix) fix=true ;;
    -h|--help) usage; exit 0 ;;
    -*) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
    *) files+=("$1") ;;
  esac
  shift
done

CLANG_TIDY="$(camloc_resolve_clang_tool clang-tidy)"

COMPILE_DB="${BUILD}/compile_commands.json"
if [[ ! -f "${COMPILE_DB}" ]]; then
  echo "No compile database at ${COMPILE_DB}." >&2
  echo "Configure first:  ./scripts/ci.sh --no-style" >&2
  exit 1
fi

# python3 rather than grep: the entries are JSON and the paths need
# canonicalizing before they can be matched against the roots.
db_files() {
  python3 - "${COMPILE_DB}" "${ROOT}" <<'PY'
import json, os, sys
db_path, repo = sys.argv[1], os.path.realpath(sys.argv[2])
roots = tuple(os.path.join(repo, d) + os.sep for d in ("src", "apps", "tests"))
for entry in json.load(open(db_path)):
    path = os.path.realpath(os.path.join(entry.get("directory", ""), entry["file"]))
    if path.startswith(roots):
        print(path)
PY
}

analyzable="$(db_files | sort -u)"
if [[ -z "${analyzable}" ]]; then
  echo "No cam_loc sources in ${COMPILE_DB}; is it from another project?" >&2
  exit 1
fi

if [[ ${#files[@]} -eq 0 ]]; then
  while IFS= read -r f; do files+=("$f"); done <<<"${analyzable}"
else
  # An explicitly named file still has to be in the database, for the reason in
  # the header comment. Reporting it is the difference between a clear error and
  # a page of invented diagnostics.
  unanalyzable=()
  resolved=()
  for file in "${files[@]}"; do
    if [[ ! -f "${file}" ]]; then
      unanalyzable+=("${file} — no such file")
      continue
    fi
    abs="$(cd "$(dirname "${file}")" && pwd)/$(basename "${file}")"
    if ! grep -qxF "${abs}" <<<"${analyzable}"; then
      unanalyzable+=("${abs} — not in the compile database")
      continue
    fi
    resolved+=("${abs}")
  done
  if [[ ${#unanalyzable[@]} -gt 0 ]]; then
    echo "clang-tidy cannot analyze ${#unanalyzable[@]} of ${#files[@]} file(s):" >&2
    printf '  %s\n' "${unanalyzable[@]}" >&2
    echo "Re-configure to refresh ${COMPILE_DB}, or check the CUDA build option." >&2
    exit 1
  fi
  files=("${resolved[@]}")
fi

# Both platforms need help finding the standard library, for different reasons.
# On macOS clang-tidy comes from the Homebrew LLVM keg and does not know where
# the SDK is; on Linux clang may select a GCC whose headers are not installed.
# Left alone, either one makes every file in the tree fail to parse.
extra_args=()
if [[ "$(uname -s)" == Darwin ]]; then
  sdk_path="$(xcrun --show-sdk-path 2>/dev/null)" || sdk_path=""
  [[ -n "${sdk_path}" ]] && extra_args+=("--extra-arg=-isysroot${sdk_path}")
else
  gcc_flag="$(camloc_clang_gcc_flag)"
  [[ -n "${gcc_flag}" ]] && extra_args+=("--extra-arg=${gcc_flag}")
fi

tidy_args=(-p "${BUILD}" "${extra_args[@]}" --quiet)
if [[ "${fix}" == true ]]; then
  tidy_args+=(--fix --fix-errors)
fi

work_dir="$(mktemp -d "${TMPDIR:-/tmp}/camloc-tidy-XXXXXX")"
trap 'rm -rf "${work_dir}"' EXIT

# clang-tidy splits findings (stdout) from progress and the reasons a file could
# not be checked at all (stderr). Keep them apart so a broken run is
# distinguishable from a clean one.
# `|| tidy_status=$?` rather than a bare call followed by $?: a non-zero
# clang-tidy is an expected outcome here, and set -e would abort on it before
# the status could be read.
tidy_status=0
if [[ "${fix}" == true ]]; then
  echo "Running clang-tidy over ${#files[@]} file(s), serially for --fix..."
  "${CLANG_TIDY}" "${tidy_args[@]}" "${files[@]}" \
    >"${work_dir}/stdout" 2>"${work_dir}/stderr" || tidy_status=$?
else
  jobs="$(camloc_nproc)"
  echo "Running clang-tidy over ${#files[@]} file(s) on ${jobs} core(s)..."
  # -n1 so each process gets one file and the pool stays busy; NUL-delimited so
  # a path with a space survives. xargs reports 123 when any child failed, which
  # is folded into the same "something went wrong" signal as a direct non-zero.
  printf '%s\0' "${files[@]}" \
    | xargs -0 -P "${jobs}" -n1 "${CLANG_TIDY}" "${tidy_args[@]}" \
      >"${work_dir}/stdout" 2>"${work_dir}/stderr" || tidy_status=$?
fi

# Findings are expected output in --fix mode (they are what just got fixed), so
# they are tracked separately from the failures that make a run untrustworthy.
findings=0
if [[ -s "${work_dir}/stdout" ]]; then
  cat "${work_dir}/stdout"
  findings=1
fi

broken=0

# Everything on stderr except the known-benign progress and tally lines is worth
# showing; keeping the filter narrow means an unfamiliar message surfaces rather
# than being swallowed the way all of stderr would be by a 2>/dev/null.
# `|| true` because grep exits 1 when it prints nothing, and printing nothing is
# the ordinary outcome here.
grep -vE '^(\[[0-9]+/[0-9]+\] Processing file |[0-9]+ warnings? (and [0-9]+ errors? )?generated\.|Suppressed [0-9]+ warnings? )' \
  "${work_dir}/stderr" >"${work_dir}/stderr-notable" || true

if [[ -s "${work_dir}/stderr-notable" ]]; then
  echo "clang-tidy wrote to stderr:" >&2
  cat "${work_dir}/stderr-notable" >&2
fi

if grep -qE 'Compile command not found|Error while processing|Error while trying to load a compilation database|unable to handle compilation' \
     "${work_dir}/stderr"; then
  echo "clang-tidy could not analyze one or more files (see stderr above)." >&2
  broken=1
fi

if [[ "${tidy_status}" -ne 0 ]]; then
  echo "clang-tidy exited ${tidy_status}." >&2
  broken=1
fi

if [[ "${fix}" == true ]]; then
  # clang-tidy's rewrites do not respect .clang-format line breaking.
  "${ROOT}/scripts/format.sh" >/dev/null
  echo "Applied fixes and re-formatted."
  exit "${broken}"
fi

if [[ "${findings}" -eq 0 && "${broken}" -eq 0 ]]; then
  echo "clang-tidy: no findings (${#files[@]} file(s) analyzed)."
fi

if [[ "${findings}" -ne 0 || "${broken}" -ne 0 ]]; then
  exit 1
fi
exit 0
