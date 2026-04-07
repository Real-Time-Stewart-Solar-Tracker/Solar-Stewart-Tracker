#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${1:-build}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
GENERATOR="${CMAKE_GENERATOR:-Ninja}"

cmake -S "${REPO_ROOT}" -B "${REPO_ROOT}/${BUILD_DIR}" \
  -G "${GENERATOR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DSOLAR_ENABLE_TESTS=ON \
  -DSOLAR_ENABLE_HW_TESTS=OFF \
  -DSOLAR_ENABLE_QT=OFF \
  -DSOLAR_TRY_LIBCAMERA=OFF \
  -DSOLAR_TRY_OPENCV=OFF

cmake --build "${REPO_ROOT}/${BUILD_DIR}" --parallel
