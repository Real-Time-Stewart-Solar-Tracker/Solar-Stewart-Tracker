# Reproducibility

This document defines the exact procedure required to reproduce the build, tests, and documentation for the Solar Stewart Tracker project.

The project is designed to build deterministically on any clean machine that satisfies the documented dependencies.

---

## 1. Software Requirements

Minimum requirements:

- CMake >= 3.16
- C++17 compatible compiler
- Standard C++ library
- Thread support (system provided)

Optional (Linux only):

- libcamera
- pkg-config

See docs/DEPENDENCIES.md for full installation instructions.

---

## 2. Clean Build Procedure

Clone repository:

git clone https://github.com/<your-username>/SolarStewartTracker.git
cd SolarStewartTracker

Configure build:

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

Compile:

cmake --build build -j

Run unit tests:

ctest --test-dir build --output-on-failure

Successful reproduction must result in:

100% tests passed

---

## 3. Optional Camera Support

Camera functionality is enabled automatically if libcamera is detected on Linux systems.

If libcamera is installed:
- Camera support is enabled.
- Compile definition SOLAR_HAVE_LIBCAMERA=1 is set.

If libcamera is not installed:
- The build succeeds normally.
- Camera functionality is disabled.
- SOLAR_HAVE_LIBCAMERA=0 is set.

No manual configuration is required.

---

## 4. Deterministic Build Configuration

The project enforces:

- C++17 standard
- No compiler-specific extensions
- Strict warning flags
- Static core library separation
- Explicit test targets

No random seeds or nondeterministic build steps are used.

---

## 5. Documentation Reproduction

Generate Doxygen documentation:

doxygen Doxyfile

Generated documentation will be placed in:

docs/doxygen/

All public class members must be documented in header files.

---

## 6. Verified Environments

The project has been verified on:

- Ubuntu 22.04 (GCC 11+)
- Raspberry Pi OS (Bookworm)
- Windows 11 (MSVC 2022)
- CMake 3.22+

---

## 7. Hardware Reproducibility

Hardware configuration is defined in:

hardware/BOM.md

This includes:

- Component models
- Quantities
- Power requirements
- Calibration notes

Following the documented BOM and calibration procedure ensures mechanical reproducibility.

---

## 8. Reproducibility Guarantee

A clean system satisfying the documented software dependencies and hardware BOM will:

- Build without modification
- Pass all unit tests
- Generate documentation successfully
- Enable optional features automatically when available

No undocumented packages, hidden dependencies, or manual patches are required.