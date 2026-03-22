#!/usr/bin/env bash
set -euo pipefail

cmake -S . -B build-pi -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build build-pi -j

export SOLAR_RUN_CAMERA_HW_TESTS=1
export SOLAR_RUN_I2C_HW_TESTS=1
export SOLAR_I2C_DEV=/dev/i2c-1

ctest --test-dir build-pi -L hw --output-on-failure | tee build-pi/hw_ctest.txt