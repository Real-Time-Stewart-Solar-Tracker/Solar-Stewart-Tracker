# Dependencies

This document lists all required software packages and tools necessary to build, test, and run the Solar Stewart Tracker project.

The project uses CMake and C++17 and has no hidden third-party C++ dependencies.

---

## 1. Build Requirements (Mandatory)

### 1.1 CMake

Minimum required version:

CMake >= 3.16

Check version:

cmake --version

---

### 1.2 C++ Compiler (C++17 Required)

The project enforces:

- C++17 standard
- Standard-compliant compiler
- No compiler-specific extensions

Supported compilers:

GCC >= 9  
Clang >= 10  
MSVC (Visual Studio 2019 or newer)

Check GCC version:

g++ --version

---

### 1.3 System Libraries

Required:

- Standard C++ library
- Thread support (provided by OS)

No additional third-party libraries are required for the core system.

---

## 2. Optional Dependency (Linux Only)

### libcamera (Optional Camera Support)

Camera support is automatically enabled if libcamera is detected using pkg-config.

If installed:
- Camera support is enabled.
- SOLAR_HAVE_LIBCAMERA=1 is defined.

If not installed:
- Build succeeds normally.
- Camera functionality is disabled.
- SOLAR_HAVE_LIBCAMERA=0 is defined.

Install on Ubuntu / Raspberry Pi OS:

sudo apt update
sudo apt install build-essential cmake pkg-config
sudo apt install libcamera-dev

libcamera is optional. The project builds without it.

---

## 3. Platform Setup

### 3.1 Ubuntu / Raspberry Pi OS

Install required tools:

sudo apt update
sudo apt install build-essential cmake pkg-config

(Optional camera support)

sudo apt install libcamera-dev

---

### 3.2 Windows (Visual Studio)

Install:

- Visual Studio 2019 or 2022
- "Desktop development with C++" workload
- CMake (bundled with Visual Studio or standalone)

---

### 3.3 macOS

Install CMake using Homebrew:

brew install cmake

Note:
- libcamera is not supported on macOS.
- Camera functionality will be disabled automatically.

---

## 4. Testing Requirements

The project uses CMake's built-in testing framework.

No external testing framework is required.

Run tests using:

ctest --test-dir build --output-on-failure

---

## 5. Verified Environments

The project has been tested successfully on:

- Ubuntu 22.04 (GCC 11+)
- Raspberry Pi OS (Bookworm)
- Windows 11 (MSVC 2022)
- CMake 3.22+

---

## Dependency Summary

Mandatory:
- CMake >= 3.16
- C++17 compatible compiler

Optional (Linux only):
- pkg-config
- libcamera-dev

No other packages are required.