#!/usr/bin/env bash
# docs.sh — generate the Doxygen API reference for the public headers.
#
# Reads docs/doxygen/Doxyfile, which documents only what carries a /// comment:
# the headers under include/. Output lands in the build directory, which sits
# beside the repository rather than inside it -- see camloc_build_dir.
#
# This deliberately does not go through CMake. Doxygen and dot are the only
# tools involved, so requiring a configure and a set of C++ libraries just to
# render documentation would be a worse trade -- and it is what lets the docs
# workflow install two apt packages and nothing else.
#
# Pass/fail comes from the warning log, not the exit status. Doxygen exits 0
# after complaining about an unresolved reference or a malformed \param, so a
# green run means nothing on its own; this is the same reasoning behind tidy.sh
# reading clang-tidy's stdout separately from its exit code.
#
# .github/workflows/docs.yml runs this on a push to main and publishes the
# result to GitHub Pages. Deliberately not part of ci.yml: the warning check
# below is strict, and Doxygen's warning set moves between releases, so folding
# it into the build gate would turn pull requests red on a Doxygen upgrade
# rather than on a change to this repository. Failing here stops the site being
# republished and nothing else.
#
# Usage:
#   ./scripts/docs.sh
#   ./scripts/docs.sh --open     # also open the generated index in a browser
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=scripts/lib.sh
source "${ROOT}/scripts/lib.sh"
DOCS_DIR="$(camloc_build_dir "${ROOT}")/docs"

usage() {
  cat <<'EOF'
Usage: docs.sh [--open]

Generate the Doxygen API reference for the public headers under include/, into
<build dir>/docs/html. Fails if Doxygen writes anything to its warning log.

Options:
  --open      Open the generated index in a browser afterwards.
  -h, --help  Show this help.

Environment:
  CAMLOC_BUILD_DIR  Where docs/ is written (default: ../<repo>-build,
                    beside the repository).
EOF
}

open_after=false
while [[ $# -gt 0 ]]; do
  case "$1" in
    --open) open_after=true ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage >&2; exit 1 ;;
  esac
  shift
done

if ! command -v doxygen >/dev/null 2>&1; then
  echo "doxygen not found. Install the build environment:" >&2
  echo "  macOS:  ./scripts/install_deps_macos.sh" >&2
  echo "  Ubuntu: ./scripts/install_deps_ubuntu.sh" >&2
  exit 1
fi

# HAVE_DOT is on, so a missing dot would silently drop every diagram rather
# than fail.
if ! command -v dot >/dev/null 2>&1; then
  echo "graphviz's dot not found; the Doxyfile sets HAVE_DOT=YES." >&2
  echo "  macOS:  brew install graphviz" >&2
  echo "  Ubuntu: sudo apt install graphviz" >&2
  exit 1
fi

# Same reasoning as format.sh and tidy.sh: name the binary that produced the
# output, because Doxygen's warning set moves between releases and that is the
# first thing to check when two machines disagree.
echo "Using $(command -v doxygen) — Doxygen $(doxygen --version)"

# WARN_LOGFILE is opened before Doxygen creates OUTPUT_DIRECTORY.
mkdir -p "${DOCS_DIR}"
LOG="${DOCS_DIR}/doxygen-warnings.log"
rm -f "${LOG}"

# Doxygen writes into html/ without clearing it, so a page that stops being
# generated stays on disk and keeps being served locally -- a header that was
# renamed still has its old page. Only html/ goes; the warning log above lives
# beside it and is written before Doxygen runs.
rm -rf "${DOCS_DIR}/html"

# The Doxyfile's paths are repo-relative, so that `doxygen docs/doxygen/Doxyfile`
# works by hand from the root.
cd "${ROOT}"

# The Doxyfile names its own output directory, which is right for that by-hand
# run. Appending the two keys on stdin lets CAMLOC_BUILD_DIR win here without
# the Doxyfile having to depend on it being set: doxygen reads a config from
# "-", and a later assignment overrides an earlier one.
{
  cat docs/doxygen/Doxyfile
  echo "OUTPUT_DIRECTORY = ${DOCS_DIR}"
  echo "WARN_LOGFILE = ${LOG}"
} | doxygen -

if [[ -s "${LOG}" ]]; then
  echo "Doxygen reported problems:" >&2
  cat "${LOG}" >&2
  exit 1
fi

echo "Documentation written to ${DOCS_DIR}/html/index.html"

if [[ "${open_after}" == true ]]; then
  if command -v open >/dev/null 2>&1; then
    open "${DOCS_DIR}/html/index.html"
  elif command -v xdg-open >/dev/null 2>&1; then
    xdg-open "${DOCS_DIR}/html/index.html"
  fi
fi
