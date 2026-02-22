# Build and Run Guide

This guide explains how to build, test, and run the Solar Stewart Tracker project.

Target deployment platform: Raspberry Pi OS (Linux)  
Development platform: Windows or Linux (CMake + C++17)

---

## 1. Repository Structure

- src/        C++ source files
- include/    Public headers
- docs/       Documentation (Doxygen, reproducibility, etc.)
- hardware/   Bill of Materials and hardware notes
- tests/      Unit tests

---

## 2. Prerequisites

### 2.1 Raspberry Pi (Target Platform)

OS:
- Raspberry Pi OS (Debian-based)

Install required tools:

sudo apt update
sudo apt install -y build-essential cmake git pkg-config

Optional (camera support via libcamera):

sudo apt install -y libcamera-dev

Verify CMake version:

cmake --version

Minimum required:
CMake >= 3.16

---

### 2.2 Windows (Development Only)

Install:

- Visual Studio 2019 or 2022
- "Desktop development with C++" workload
- CMake (bundled or standalone)

Verify:

cmake --version

---

## 3. Clone Repository

git clone <your-repository-url>
cd SolarStewartTracker

---

## 4. Configure Build

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

---

## 5. Build Project

cmake --build build -j

This produces:

- Static library: solar_tracker_core
- Executable: solar_tracker
- Test executable: solar_tracker_tests

---

## 6. Run Unit Tests

ctest --test-dir build --output-on-failure

Expected result:

100% tests passed

---

## 7. Run Application

Linux:

./build/solar_tracker

Windows:

.\build\Release\solar_tracker.exe

---

## 8. Optional Features

### libcamera (Raspberry Pi Only)

If libcamera is installed:
- Camera backend is enabled automatically.

If not installed:
- The project builds normally.
- Camera functionality is disabled.

No manual configuration is required.

---

## 9. Documentation (Doxygen)

Generate API documentation:

doxygen Doxyfile

Open:

docs/doxygen/html/index.html

---

## 10. Clean Build (Optional)

To rebuild from scratch:

rm -rf build
cmake -S . -B build
cmake --build build