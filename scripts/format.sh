#!/usr/bin/env bash
# format.sh — apply the repo's formatting rules in place.
#
# Two passes over src/, include/, apps/, tests/ and ros/:
#   1. clang-format against the root .clang-format (Google, 80 columns). CUDA
#      sources included; clang-format reads .cu as C++.
#   2. trailing-whitespace strip, which also reaches what clang-format does not
#      touch: the scripts, the CMakeLists, the docs.
#
# No clang-format version is pinned: CI installs the distro's on ubuntu-latest,
# a macOS developer gets Homebrew's, and the two run several majors apart. The
# tree formats identically under 18 through 22, and .clang-format restates the
# one Google default that changed across that range (DerivePointerAlignment,
# flipped in LLVM 21) so both sides agree. lib.sh prints which binary ran, which
# is the first thing to check if --check disagrees between two machines.
#
# Usage:
#   ./scripts/format.sh
#   ./scripts/format.sh --check   # report and exit 1; writes nothing. For CI.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"

usage() {
  cat <<'EOF'
Usage: format.sh [--check]

Apply clang-format and strip trailing whitespace across the hand-written
sources.

Options:
  --check     Report files that would change and exit 1; writes nothing.
  -h, --help  Show this help.

Environment:
  CAMLOC_CLANG_FORMAT  clang-format binary to use (default: the Homebrew llvm
                       keg on macOS, else PATH).
EOF
}

check_only=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --check) check_only=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
  shift
done

CLANG_FORMAT="$(camloc_resolve_clang_tool clang-format)"

# GNU sed takes -i; BSD sed (macOS) requires an explicit empty suffix.
if sed --version >/dev/null 2>&1; then
  sed_inplace=(sed -i)
else
  sed_inplace=(sed -i '')
fi

# Trailing whitespace is stripped from more than clang-format covers, because
# the scripts and docs are read as often as the sources are.
list_whitespace_files() {
  camloc_source_files "${ROOT}"
  find "${ROOT}/scripts" -type f -name '*.sh' | sort
  find "${ROOT}/docs" -type f -name '*.md' | sort
  find "${ROOT}" -maxdepth 1 -type f -name '*.md' | sort
  find "${ROOT}/src" "${ROOT}/apps" "${ROOT}/tests" "${ROOT}/ros" \
    -type f -name 'CMakeLists.txt' | sort
  echo "${ROOT}/CMakeLists.txt"
}

status=0

if [[ "${check_only}" == true ]]; then
  while IFS= read -r file; do
    if ! "${CLANG_FORMAT}" "${file}" | diff -q - "${file}" >/dev/null 2>&1; then
      echo "needs clang-format: ${file#"${ROOT}"/}"
      status=1
    fi
  done < <(camloc_source_files "${ROOT}")

  while IFS= read -r file; do
    if grep -qE '[[:blank:]]+$' "${file}"; then
      echo "trailing whitespace:  ${file#"${ROOT}"/}"
      status=1
    fi
  done < <(list_whitespace_files)

  if [[ "${status}" -eq 0 ]]; then
    echo "All files are formatted."
  fi
  exit "${status}"
fi

formatted=0
while IFS= read -r file; do
  "${CLANG_FORMAT}" -i "${file}"
  formatted=$((formatted + 1))
done < <(camloc_source_files "${ROOT}")

# After clang-format, so this pass has the final say.
stripped=0
while IFS= read -r file; do
  if grep -qE '[[:blank:]]+$' "${file}"; then
    "${sed_inplace[@]}" -E 's/[[:blank:]]+$//' "${file}"
    stripped=$((stripped + 1))
  fi
done < <(list_whitespace_files)

echo "clang-format applied to ${formatted} file(s); trailing whitespace stripped from ${stripped}."
