#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

rm -rf "${REPO_ROOT}/docs/html"
doxygen "${REPO_ROOT}/Doxyfile"
