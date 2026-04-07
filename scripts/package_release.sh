#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${1:-build}"
OUT_ZIP="${2:-solar-stewart-tracker-release.zip}"
STAGE_DIR="${REPO_ROOT}/release_package"

rm -rf "${STAGE_DIR}"
mkdir -p "${STAGE_DIR}/bin"

cp "${REPO_ROOT}/${BUILD_DIR}/solar_tracker" "${STAGE_DIR}/bin/"

if [[ -f "${REPO_ROOT}/${BUILD_DIR}/solar_tracker_qt" ]]; then
  cp "${REPO_ROOT}/${BUILD_DIR}/solar_tracker_qt" "${STAGE_DIR}/bin/"
fi

for path in CMakeLists.txt README.md LICENSE Doxyfile .gitmodules; do
  if [[ -f "${REPO_ROOT}/${path}" ]]; then
    cp "${REPO_ROOT}/${path}" "${STAGE_DIR}/"
  fi
done

for dir in src docs scripts diagrams media artefacts external; do
  if [[ -d "${REPO_ROOT}/${dir}" ]]; then
    cp -R "${REPO_ROOT}/${dir}" "${STAGE_DIR}/"
  fi
done

rm -f "${REPO_ROOT}/${OUT_ZIP}"
(
  cd "${REPO_ROOT}"
  zip -r "${OUT_ZIP}" "$(basename "${STAGE_DIR}")"
)
