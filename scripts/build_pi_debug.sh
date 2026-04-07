#!/usr/bin/env bash
set -euo pipefail

BUILD_DIR="${1:-build-pi}"
GENERATOR="${CMAKE_GENERATOR:-Ninja}"

cmake -S . -B "${BUILD_DIR}" \
  -G "${GENERATOR}" \
  -DCMAKE_BUILD_TYPE=Debug \
  -DSOLAR_ENABLE_TESTS=ON \
  -DSOLAR_ENABLE_HW_TESTS=ON \
  -DSOLAR_ENABLE_QT=OFF \
  -DSOLAR_TRY_LIBCAMERA=OFF \
  -DSOLAR_TRY_OPENCV=OFF

cmake --build "${BUILD_DIR}" --parallel --target \
  test_pca9685 \
  test_servodriver \
  test_mpu6050_publisher \
  test_linux_i2c_hw
