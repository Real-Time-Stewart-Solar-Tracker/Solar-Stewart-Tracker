# Reproducibility Guide

This document explains how to reproduce the current repository build, tests, executables, and documentation.

------------------------------------------------------------------------

## Clone repository

```bash
git clone https://github.com/Real-Time-Stewart-Solar-Tracker/Solar-Stewart-Tracker.git
cd Solar-Stewart-Tracker
````

---

## Install dependencies

### Minimal

```bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config
```

### Optional

Camera backend:

```bash
sudo apt install -y libcamera-dev
```

Qt GUI:

```bash
sudo apt install -y qtbase5-dev qtcharts5-dev qt5-qmake
```

OpenCV viewer support:

```bash
sudo apt install -y libopencv-dev
```

Documentation tools:

```bash
sudo apt install -y doxygen graphviz
```

---

## Configure build

Standard Linux / Raspberry Pi OS configure:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
```

Disable OpenCV detection explicitly:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSOLAR_TRY_OPENCV=OFF
```

---

## Build

```bash
cmake --build build -j$(nproc)
```

---

## Run tests

Linux / single-config generators:

```bash
ctest --test-dir build --output-on-failure
```

Windows / multi-config generators such as Visual Studio:

```powershell
ctest --test-dir build -C Release --output-on-failure
```

---

## Run programs

Core CLI application:

```bash
./build/solar_tracker
```

Qt GUI application (only if the `solar_tracker_qt` target was built):

```bash
./build/solar_tracker_qt
```

Manual hardware executable (not part of `CTest`):

```bash
./build/tests/servo_manual_smoketest
```

Typical Windows executable locations for Visual Studio generators:

```text
build\Release\solar_tracker.exe
build\Release\solar_tracker_qt.exe
```

The exact executable subdirectory may vary by generator.

---

## Generate documentation

```bash
doxygen Doxyfile
```

Open:

```text
docs/html/index.html
```

---

## Hardware BOM

Located at:

```text
docs/BOM.md
```

---

## Clean rebuild

### Linux

```bash
rm -rf build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
ctest --test-dir build --output-on-failure
```

### Windows

```powershell
Remove-Item -Recurse -Force build
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

