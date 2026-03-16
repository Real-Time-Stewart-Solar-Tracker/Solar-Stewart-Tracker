# Dependencies

This document lists all required and optional software packages
needed to build, test, and run the Solar Stewart Tracker project.

The project uses **CMake** and **C++17** and follows a modular,
optionally-extended architecture.

---

# 1. Mandatory Requirements

## 1.1 CMake

Minimum required version:

CMake >= 3.16

Check version:

```bash
cmake --version
```

---

## 1.2 C++ Compiler (C++17 Required)

The project enforces:

- C++17 standard
- Standard-compliant compiler
- No compiler-specific extensions

Supported compilers:

- GCC >= 9
- Clang >= 10
- MSVC (Visual Studio 2019 or newer)

Check GCC version:

```bash
g++ --version
```

---

## 1.3 Standard System Libraries

Required:

- Standard C++ library
- Threading support (provided by OS)
- POSIX support on Linux

No third-party C++ libraries are required for the **core system**.

---

# 2. Optional Dependencies (Feature-Based)

The project builds modular targets depending on detected libraries.

---

## 2.1 libcamera (Linux / Raspberry Pi Only)

Enables hardware camera support.

Automatically detected using `pkg-config`.

If installed:
- Camera backend is enabled.
- `SOLAR_HAVE_LIBCAMERA=1` is defined.

If not installed:
- Build succeeds normally.
- Camera functionality is disabled.
- `SOLAR_HAVE_LIBCAMERA=0` is defined.

Install on Raspberry Pi OS / Ubuntu:

```bash
sudo apt update
sudo apt install -y libcamera-dev pkg-config
```

Note:
libcamera is optional. The project builds without it.

---

## 2.2 Qt5 (GUI Application)

Required only for the Qt GUI target `solar_tracker_qt`.

CMake option:

```
-DSOLAR_HAVE_QT=ON/OFF
```

If Qt5 is installed and `SOLAR_HAVE_QT=ON`:
- The Qt GUI target is built.

If Qt5 is not installed:
- The core system still builds.
- GUI target is disabled.

Install on Raspberry Pi OS / Ubuntu:

```bash
sudo apt install -y qtbase5-dev qtcharts5-dev qt5-qmake
```

---

## 2.3 OpenCV (Legacy Viewer Target Only)

Only required for the optional legacy executable:

```
-DSOLAR_BUILD_LEGACY_OPENCV_APP=ON
```

If OpenCV is found:
- `SOLAR_HAVE_OPENCV=1` is defined.

If not found:
- Legacy viewer target is not built.
- Core and Qt targets still build normally.

Install:

```bash
sudo apt install -y libopencv-dev
```

Note:
The Qt GUI does **not** depend on OpenCV.

---

# 3. Platform Setup

## 3.1 Ubuntu / Raspberry Pi OS

Install required tools:

```bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config
```

Optional:

```bash
sudo apt install -y libcamera-dev
sudo apt install -y qtbase5-dev qtcharts5-dev
sudo apt install -y libopencv-dev
```

---

## 3.2 Windows (Visual Studio)

Install:

- Visual Studio 2019 or 2022
- “Desktop development with C++” workload
- CMake (bundled or standalone)

Qt and OpenCV are optional on Windows depending on targets built.

---

## 3.3 macOS

Install CMake:

```bash
brew install cmake
```

Note:

- libcamera is not supported on macOS.
- Camera functionality will be disabled automatically.
- Qt5 must be installed separately if GUI is required.

---

# 4. Testing Requirements

The project uses CMake’s built-in testing system.

No external testing framework is required.

Run tests using:

```bash
ctest --test-dir build --output-on-failure
```

---

# 5. Verified Environments

The project has been tested successfully on:

- Ubuntu 22.04 (GCC 11+)
- Raspberry Pi OS (Bookworm)
- Windows 11 (MSVC 2022)
- CMake 3.22+

---

# Dependency Summary

Mandatory:
- CMake >= 3.16
- C++17 compatible compiler

Optional:
- pkg-config (Linux)
- libcamera-dev (camera support)
- Qt5 (GUI application)
- OpenCV (legacy viewer only)

No additional third-party libraries are required.