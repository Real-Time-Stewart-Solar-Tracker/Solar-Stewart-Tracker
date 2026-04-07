# Reproducibility

## Purpose

This document explains how to reproduce the  repository  from source using the files: `CMakeLists.txt`, `.github/workflows/`, `scripts/`, `.gitmodules`, `external/`, `src/`, and `docs/`.

---

## 0. Reference Platform

All latency measurements and hardware-smoke tests were captured on the following exact platform:

| Parameter | Value |
|---|---|
| Hardware | Raspberry Pi 5 Model B, 4 GB RAM |
| CPU | Broadcom BCM2712, Cortex-A76, 4-core, 2.4 GHz |
| Operating system | Raspberry Pi OS Bookworm (Debian 12), 64-bit (aarch64) |
| Kernel | Linux 6.6.51+rpt-rpi-2712 (aarch64) |
| Compiler | GCC 12.2.0 (aarch64-linux-gnu) |
| libgpiod | 2.2.2 (built from source) |
| OpenCV | 4.10.0 (`libopencv-dev` via apt) |
| libcamera | 0.3.2 (`libcamera-dev` via apt) |
| Qt | Qt 5.15.8 (`qtbase5-dev libqt5charts5-dev` via apt) |

---

## 1. Source of Truth

The reproducible source of truth in this repository is:

| Path | Role |
|---|---|
| `src/` | Application and library source code |
| `CMakeLists.txt` | Top-level build definition |
| `.gitmodules` | External submodule declarations |
| `external/` | External support code checked out via submodules |
| `scripts/` | Build, test, documentation, latency, and packaging helper scripts |
| `.github/workflows/` | CI, documentation, hardware-smoke, and release automation |
| `artefacts/latency.csv` | Captured software latency measurements |
| `Doxyfile` | API documentation configuration |

---

## 2. External Dependencies

### Git Submodules

Two external repositories are declared in `.gitmodules`:

| Submodule | Path | URL |
|---|---|---|
| `libgpiod_event_demo` | `external/libgpiod_event_demo` | `https://github.com/berndporr/libgpiod_event_demo` |
| `libcamera2opencv` | `external/libcamera2opencv` | `https://github.com/berndporr/libcamera2opencv` |

These should be fetched with:

```bash
git submodule update --init --recursive
```

### System Packages

The Ubuntu CI workflow installs these packages directly:

```bash
sudo apt-get update
sudo apt-get install -y \
  autoconf \
  autoconf-archive \
  automake \
  cmake \
  g++ \
  libjsoncpp-dev \
  libtool \
  make \
  ninja-build \
  pkg-config \
  wget \
  xz-utils
```

The CI workflow then builds and installs **libgpiod v2.2.2** from source so that the C++ bindings are available.

For local Linux reproduction, the following additional packages may be needed depending on which optional features are enabled:

- `libopencv-dev` for OpenCV-dependent paths
- `libcamera-dev` for the Raspberry Pi libcamera backend
- `qtbase5-dev` for the Qt GUI target
- `graphviz` and `doxygen` for API documentation generation
- `i2c-tools` for Raspberry Pi hardware-smoke workflows

---

## 3. CMake Build Options

The top-level build defines the following options:

| Option | Default | Effect |
|---|---|---|
| `SOLAR_ENABLE_TESTS` | `ON` | Build and register test targets |
| `SOLAR_ENABLE_HW_TESTS` | `OFF` | Register real hardware smoke tests |
| `SOLAR_ENABLE_QT` | `ON` | Build the Qt GUI target if Qt5 is available |
| `SOLAR_TRY_LIBCAMERA` | `ON` | Probe for the libcamera backend |
| `SOLAR_TRY_OPENCV` | `ON` | Probe for OpenCV support |
| `SOLAR_ENABLE_COVERAGE` | `OFF` | Enable coverage compile/link flags where supported |

Feature probing behaves as follows:

- if OpenCV is not found, OpenCV-dependent functionality is disabled
- if libcamera is found but OpenCV is not found, the libcamera backend is disabled
- if Qt5 is not found, the Qt GUI target is skipped
- if `external/libgpiod_event_demo` is missing, GPIO demo support is disabled

---

## 4. Clean Reproduction Procedure

```bash
# Step 1 — clone
git clone <repo-url>
cd Solar-Stewart-Tracker

# Step 2 — initialise submodules
git submodule update --init --recursive

# Step 3 — install build dependencies
sudo apt-get update
sudo apt-get install -y \
  cmake \
  ninja-build \
  g++ \
  pkg-config \
  libjsoncpp-dev

# Optional packages for optional features
# sudo apt-get install -y libopencv-dev libcamera-dev qtbase5-dev doxygen graphviz i2c-tools

# Step 4 — configure
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release

# Step 5 — build
cmake --build build --parallel

# Step 6 — run software tests
ctest --test-dir build --output-on-failure --verbose

# Step 7 — run the headless application
./build/solar_tracker

# Step 8 — run the Qt GUI if Qt was found at configure time
./build/solar_tracker_qt
```

---

## 5. Helper Scripts in `scripts/`

The repository  contains these helper scripts:

| Script | Purpose |
|---|---|
| `scripts/build_core.sh` | Configure and build the main software targets |
| `scripts/build_docs.sh` | Generate Doxygen documentation |
| `scripts/build_pi_debug.sh` | Configure a Pi-oriented debug build |
| `scripts/test_core.sh` | Run the software test suite |
| `scripts/test_pi_hw.sh` | Run hardware-labelled smoke tests on Raspberry Pi |
| `scripts/run_latency.sh` | Run the application with latency CSV export enabled |
| `scripts/package_release.sh` | Package a release zip |


---

## 6. CI Automation

Four workflows are defined in `.github/workflows/`:

| Workflow | Trigger | Runner |
|---|---|---|
| `ci.yml` | push to `main`, pull request to `main`, manual dispatch | `ubuntu-latest` |
| `doxygen.yml` | push to `main` or `master`, manual dispatch | `ubuntu-latest` |
| `release.yml` | release created, manual dispatch | `ubuntu-latest` |
| `pi-hardware-tests.yml` | manual dispatch | self-hosted `linux/arm64` |

### Standard CI (`ci.yml`)

The standard CI workflow performs the following steps:

1. checks out the repository with submodules
2. installs build dependencies
3. builds and installs `libgpiod` v2 with C++ bindings from source
4. configures the project with CMake and Ninja
5. builds the repository
6. runs `ctest --test-dir build --output-on-failure --verbose`

### Doxygen workflow (`doxygen.yml`)

The documentation workflow:

1. checks out the repository with submodules
2. installs `doxygen` and `graphviz`
3. runs `bash ./scripts/build_docs.sh`
4. uploads `docs/html` as the Pages artifact
5. deploys to GitHub Pages

### Release workflow (`release.yml`)

The release workflow:

1. checks out the repository with submodules
2. installs build dependencies
3. runs `bash ./scripts/build_core.sh build`
4. runs `bash ./scripts/package_release.sh build solar-stewart-tracker-release.zip`
5. uploads the packaged release artifact

### Raspberry Pi hardware-smoke workflow (`pi-hardware-tests.yml`)

The Pi workflow:

1. checks out the repository with submodules
2. installs Pi-side dependencies including `i2c-tools`
3. configures a debug build with `SOLAR_ENABLE_HW_TESTS=ON`
4. builds curated hardware-smoke targets
5. runs `bash ./scripts/test_pi_hw.sh "${BUILD_DIR}"`
6. uploads CTest logs and cache files as artifacts

---

## 7. Hardware Reproduction

The repository assumes a Raspberry Pi Linux target with the following hardware path available for full reproduction:

- Raspberry Pi running Linux
- I2C bus exposed as `/dev/i2c-1`
- PCA9685 PWM controller
- ADS1115 ADC for manual input sampling
- MPU6050 IMU
- GPIO event support through `libgpiod`
- Raspberry Pi camera path compatible with libcamera if the libcamera backend is used

### Hardware Test Environment Variables

The hardware-smoke script uses these environment variables:

| Variable | Default | Meaning |
|---|---|---|
| `SOLAR_RUN_CAMERA_HW_TESTS` | `1` | Enable camera hardware checks |
| `SOLAR_RUN_I2C_HW_TESTS` | `1` | Enable I2C hardware checks |
| `SOLAR_I2C_DEV` | `/dev/i2c-1` | I2C device path |

### Enabling Hardware Tests

```bash
cmake -S . -B build \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Debug \
  -DSOLAR_ENABLE_TESTS=ON \
  -DSOLAR_ENABLE_HW_TESTS=ON \
  -DSOLAR_ENABLE_QT=OFF \
  -DSOLAR_TRY_LIBCAMERA=OFF \
  -DSOLAR_TRY_OPENCV=OFF

cmake --build build --parallel
./scripts/test_pi_hw.sh build
```

---

## 8. Latency Data Reproduction

The repository includes `artefacts/latency.csv` as captured software latency evidence.

The same output path can be regenerated with:

```bash
./scripts/run_latency.sh build artefacts/latency.csv solar_tracker
```

This script sets the `SOLAR_LATENCY_CSV` environment variable before launching the application. At shutdown, the runtime `LatencyMonitor` writes per-frame measurements to the specified CSV path.

---

## 9. Non-Hardware Reproduction

If physical hardware is unavailable:

- the software test suite still runs
- hardware-smoke tests remain disabled by default because `SOLAR_ENABLE_HW_TESTS` defaults to `OFF`
- the application can run through the simulated camera backend path when hardware backends are not available

This allows code-level reproduction of the processing pipeline without requiring the full physical platform.

---

## 10. API Documentation Reproduction

A `Doxyfile` is  at the repository root.

Local generation:

```bash
doxygen Doxyfile
```

The repository also contains `scripts/build_docs.sh`, which is used by the GitHub documentation workflow.

---

## 11. Summary

This repository can be reproduced from source because it contains:

- the top-level build definition
- the declared submodules
- the build, test, latency, documentation, and packaging scripts
- the CI workflows
- the source tree
- the API documentation configuration
- the captured software latency artifact

A software-only reproduction path is available on a normal Linux system, and a fuller Raspberry Pi hardware-smoke path is defined separately for the target platform.
