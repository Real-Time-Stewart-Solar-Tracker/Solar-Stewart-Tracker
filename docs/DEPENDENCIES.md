# Dependencies

This document lists the software dependencies that match the current repository and build system.

The project uses **CMake** and **C++17**. Optional features are enabled by dependency detection in `CMakeLists.txt`.

---

## 1) Mandatory requirements

### 1.1 Build system

- **CMake 3.16 or newer**

Check version:

```bash
cmake --version
```
### 1.2 Compiler

The current build enforces:

* **C++17**
* `CMAKE_CXX_STANDARD_REQUIRED ON`
* `CMAKE_CXX_EXTENSIONS OFF`

A standards-compliant C++17 compiler is therefore required.

Supported compiler families include:

* **GCC**
* **Clang**
* **MSVC**

### 1.3 Standard/system support

Required for the core build:

* standard C++ library
* thread support from the host OS/runtime

No third-party C++ library is required for the **core headless build**.

---

## 2) Optional dependencies

### 2.1 `pkg-config` (Linux only)

`pkg-config` is used for optional dependency detection on Linux, including `libcamera`.

Install on Debian / Raspberry Pi OS / Ubuntu:

```bash
sudo apt update
sudo apt install -y pkg-config
```

If `pkg-config` is not present, the repository still builds; only automatic detection of some optional Linux dependencies is affected.

---

### 2.2 `libcamera` (Linux / Raspberry Pi only)

Purpose in the current repository:

* enables compilation of `src/sensors/LibcameraPublisher.cpp`
* enables the Raspberry Pi camera backend on supported Linux systems

If `libcamera` is not found:

* the build still succeeds
* the project uses non-libcamera paths such as `SimulatedPublisher`

Install:

```bash
sudo apt update
sudo apt install -y libcamera-dev pkg-config
```

Detection is performed automatically on supported Linux systems.

---

### 2.3 Qt5 (optional GUI target)

Purpose in the current repository:

* builds the optional Qt GUI target `solar_tracker_qt` when Qt5 is found

Install on Debian / Raspberry Pi OS / Ubuntu:

```bash
sudo apt install -y qtbase5-dev qtcharts5-dev qt5-qmake
```

Detection is automatic.

Important notes:

* Qt support is auto-detected
* the repository does not rely on a separate mandatory Qt-only build path
* if Qt is not found, the core CLI build still works

---

### 2.4 OpenCV (optional CLI viewer support)

Purpose in the current repository:

* when OpenCV is found, viewer support can be compiled into the CLI application path
* this enables `UiViewer` support in the current codebase

Install on Debian / Raspberry Pi OS / Ubuntu:

```bash
sudo apt install -y libopencv-dev
```

Disable OpenCV detection explicitly:

```bash
cmake -S . -B build -DSOLAR_TRY_OPENCV=OFF
```

Important notes:

* OpenCV support is optional
* OpenCV does not create a separate main executable in the current repository
* the Qt GUI target does not depend on OpenCV

---

## 3) Platform setup examples

### 3.1 Linux / Raspberry Pi OS

Required tools:

```bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config
```

Optional packages depending on features needed:

```bash
sudo apt install -y libcamera-dev
sudo apt install -y qtbase5-dev qtcharts5-dev qt5-qmake
sudo apt install -y libopencv-dev
```

### 3.2 Windows

Install:

* Visual Studio 2019 or 2022
* Desktop development with C++
* CMake

Optional:

* Qt5 if `solar_tracker_qt` is required
* OpenCV if viewer support is required

---

## 4) Build and test tools used by the current repository

### 4.1 CTest

The repository uses **CMake/CTest** directly.

No external unit-test framework is required.

Run automated tests with:

```bash
ctest --test-dir build --output-on-failure
```

### 4.2 Doxygen

The repository contains a `Doxyfile` and supports Doxygen-based documentation generation.

Generate locally with:

```bash
doxygen Doxyfile
```

Current configured HTML output path:

* `docs/html/`
* main page: `docs/html/index.html`

---

## 5) Current build target summary

Always built:

* `solar_tracker`
* `solar_tracker_core`
* `test_core`
* `test_pca9685`
* `test_servodriver`
* `servo_manual_smoketest` (manual hardware executable, not part of `CTest`)

Conditionally built:

* `solar_tracker_qt` — only when Qt5 is found
* `test_linux_i2c_hw` — on supported Linux systems
* `test_libcamera_hw` — on supported Linux systems when libcamera support is available

Conditionally compiled into the core library:

* `src/sensors/LibcameraPublisher.cpp` — when `libcamera` is found
* `src/ui/UiViewer.cpp` — when OpenCV is found and `SOLAR_TRY_OPENCV=ON`

```
```
