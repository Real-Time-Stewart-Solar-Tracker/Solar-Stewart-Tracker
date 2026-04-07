#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${1:-build-pi}"

export SOLAR_RUN_I2C_HW_TESTS="${SOLAR_RUN_I2C_HW_TESTS:-0}"
export SOLAR_I2C_DEV="${SOLAR_I2C_DEV:-/dev/i2c-1}"

CTEST_REGEX='^(PCA9685_|ServoDriver_|Mpu6050Publisher_|LinuxI2C_)'

ctest --test-dir "${BUILD_DIR}" --output-on-failure -R "${CTEST_REGEX}"
