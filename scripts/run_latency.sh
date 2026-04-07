#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${1:-build}"
CSV_PATH="${2:-artefacts/latency.csv}"
APP="${3:-solar_tracker}"

mkdir -p "$(dirname "${CSV_PATH}")"

SOLAR_LATENCY_CSV="${CSV_PATH}" "./${BUILD_DIR}/${APP}"