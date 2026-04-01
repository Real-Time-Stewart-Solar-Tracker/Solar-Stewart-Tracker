#!/usr/bin/env bash
set -euo pipefail

# Collect reproducible latency evidence on Raspberry Pi / Linux.
#
# Produces:
# - raw latency CSV
# - full runtime log
# - platform description
# - git commit hash
# - exact command used
# - brief README for the artefact folder
#
# Requirements:
# - built solar_tracker executable
# - timeout command
# - git available if commit hash is desired
#
# Usage:
#   bash scripts/collect_latency_pi.sh
#   bash scripts/collect_latency_pi.sh --duration 30
#   bash scripts/collect_latency_pi.sh --build-dir build --out-dir artifacts/latency_run_01

BUILD_DIR="build"
OUT_DIR=""
DURATION_S=30
APP_PATH=""
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --build-dir)
            BUILD_DIR="$2"
            shift 2
            ;;
        --out-dir)
            OUT_DIR="$2"
            shift 2
            ;;
        --duration)
            DURATION_S="$2"
            shift 2
            ;;
        --app)
            APP_PATH="$2"
            shift 2
            ;;
        -h|--help)
            cat <<'EOF'
Usage:
  bash scripts/collect_latency_pi.sh [options]

Options:
  --build-dir <dir>   Build directory containing solar_tracker (default: build)
  --out-dir <dir>     Output artefact directory
  --duration <sec>    Capture duration in seconds (default: 30)
  --app <path>        Explicit solar_tracker executable path
  -h, --help          Show this help
EOF
            exit 0
            ;;
        *)
            echo "Unknown argument: $1" >&2
            exit 1
            ;;
    esac
done

if [[ -z "${OUT_DIR}" ]]; then
    OUT_DIR="artifacts/latency_${TIMESTAMP}"
fi

mkdir -p "${OUT_DIR}"

if [[ -z "${APP_PATH}" ]]; then
    APP_PATH="${BUILD_DIR}/solar_tracker"
fi

if [[ ! -x "${APP_PATH}" ]]; then
    echo "Error: executable not found or not executable: ${APP_PATH}" >&2
    exit 1
fi

RAW_CSV="${OUT_DIR}/latency_raw.csv"
RUN_LOG="${OUT_DIR}/run.log"
PLATFORM_TXT="${OUT_DIR}/platform.txt"
GIT_TXT="${OUT_DIR}/git_commit.txt"
COMMAND_TXT="${OUT_DIR}/command.txt"
README_TXT="${OUT_DIR}/README.txt"

echo "Collecting latency evidence into: ${OUT_DIR}"

# Platform description
{
    echo "Timestamp: $(date --iso-8601=seconds)"
    echo "Hostname: $(hostname)"
    echo "Kernel: $(uname -a)"
    echo
    echo "OS release:"
    if [[ -f /etc/os-release ]]; then
        cat /etc/os-release
    else
        echo "Unavailable"
    fi
    echo
    echo "CPU info:"
    if command -v lscpu >/dev/null 2>&1; then
        lscpu
    else
        echo "lscpu unavailable"
    fi
    echo
    echo "libcamera version:"
    if command -v libcamera-hello >/dev/null 2>&1; then
        libcamera-hello --version || true
    else
        echo "libcamera-hello unavailable"
    fi
} > "${PLATFORM_TXT}"

# Commit hash / repo state
{
    if git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
        echo "Commit: $(git rev-parse HEAD)"
        echo "Short commit: $(git rev-parse --short HEAD)"
        echo
        echo "Status:"
        git status --short
    else
        echo "Not inside a git work tree"
    fi
} > "${GIT_TXT}"

# Exact command used
{
    echo "Working directory: $(pwd)"
    echo "Executable: ${APP_PATH}"
    echo "Duration seconds: ${DURATION_S}"
    echo "Raw CSV path: ${RAW_CSV}"
    echo
    echo "Command:"
    echo "SOLAR_LATENCY_CSV=\"${RAW_CSV}\" timeout --signal=INT ${DURATION_S}s \"${APP_PATH}\""
} > "${COMMAND_TXT}"

# Artefact README
cat > "${README_TXT}" <<EOF
Latency evidence artefact folder

Files:
- latency_raw.csv : raw per-frame latency evidence emitted by LatencyMonitor
- run.log         : full application stdout/stderr during capture
- platform.txt    : hardware / OS / platform information
- git_commit.txt  : commit hash and repo status
- command.txt     : exact command used to collect the data

Recommended workflow:
1. Build the project in Release mode.
2. Run this script on the Raspberry Pi hardware target.
3. Use latency_raw.csv as the raw evidence referenced by docs.
4. Copy the exact commit hash from git_commit.txt into docs/latency_measurement.md.
EOF

echo "Running application for ${DURATION_S}s ..."
set +e
SOLAR_LATENCY_CSV="${RAW_CSV}" \
timeout --signal=INT "${DURATION_S}s" "${APP_PATH}" > "${RUN_LOG}" 2>&1
APP_STATUS=$?
set -e

# timeout returns 124 on timeout; a graceful SIGINT-handled shutdown may also return 130.
if [[ "${APP_STATUS}" -ne 0 && "${APP_STATUS}" -ne 124 && "${APP_STATUS}" -ne 130 ]]; then
    echo "Application exited with unexpected status: ${APP_STATUS}" >&2
    echo "Check ${RUN_LOG}" >&2
    exit "${APP_STATUS}"
fi

if [[ ! -f "${RAW_CSV}" ]]; then
    echo "Warning: raw latency CSV was not produced at ${RAW_CSV}" >&2
else
    echo "Raw CSV written to: ${RAW_CSV}"
    echo "CSV line count: $(wc -l < "${RAW_CSV}")"
fi

echo "Runtime log written to: ${RUN_LOG}"
echo "Platform info written to: ${PLATFORM_TXT}"
echo "Git info written to: ${GIT_TXT}"
echo "Command record written to: ${COMMAND_TXT}"
echo "Done."