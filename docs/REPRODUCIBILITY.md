# Reproducibility

This document defines the exact procedure required to reproduce the build,
tests, and documentation for the Solar Stewart Tracker project.

The project is designed to build consistently on a clean system that satisfies
the documented software dependencies.

---

# 1. Software Requirements

Minimum requirements:

- CMake >= 3.16
- C++17 compatible compiler
- Standard C++ library
- Operating system with thread support

Supported compilers:

- GCC ≥ 9
- Clang ≥ 10
- MSVC (Visual Studio 2019 or newer)

Optional (Linux only):

- pkg-config
- libcamera-dev (camera backend support)

Full installation instructions are provided in:

docs/DEPENDENCIES.md

---

# 2. Clean Build Procedure

Clone repository:

```bash
git clone https://github.com/<your-username>/SolarStewartTracker.git
cd SolarStewartTracker
```

Configure build:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
```

Build:

```bash
cmake --build build -j
```

Run unit tests:

```bash
ctest --test-dir build --output-on-failure
```

Successful reproduction must result in:

- All tests passing
- No compilation errors
- No missing dependency errors

Expected test executables:

- test_core
- test_pca9685
- test_servodriver

---

# 3. Optional Camera Support (Linux Only)

Camera support is enabled automatically when libcamera is detected.

If `libcamera-dev` is installed:

- SOLAR_HAVE_LIBCAMERA=1 is defined
- The libcamera backend is compiled
- Hardware camera streaming is enabled

If not installed:

- SOLAR_HAVE_LIBCAMERA=0 is defined
- The build succeeds normally
- Camera functionality is disabled
- Simulation mode remains available

No manual code modification is required.

---

# 4. Deterministic Build Configuration

The build system enforces:

- C++17 standard
- No compiler-specific extensions
- Explicit compile flags (warnings enabled)
- Static core library (`solar_tracker_core`)
- Explicit test targets
- No hidden runtime dependencies

The build process does not use:

- Random code generation
- Network downloads
- External package managers at build time

All dependencies are system-level packages.

---

# 5. Documentation Reproduction

Generate Doxygen documentation:

```bash
doxygen Doxyfile
```

Output directory:

```
docs/doxygen/
```

All public headers in `include/` are documented.

Documentation generation requires only Doxygen.

---

# 6. Verified Environments

The project has been verified on:

- Ubuntu 22.04 (GCC 11+)
- Raspberry Pi OS (Bookworm)
- Windows 11 (MSVC 2022)
- CMake 3.22+

---

# 7. Hardware Reproducibility

The hardware configuration is documented in:

```
hardware/BOM.md
```

The BOM specifies:

- Component models
- Quantities
- Power requirements
- Interface details
- Calibration assumptions

Following the documented BOM ensures hardware reproducibility.

---

# 8. Reproducibility Statement

A clean system satisfying the documented software dependencies will:

- Configure successfully with CMake
- Build without modification
- Pass all unit tests
- Generate documentation successfully
- Enable optional features automatically when available

The repository contains no hidden dependencies,
no undocumented configuration steps,
and no manual patch requirements.