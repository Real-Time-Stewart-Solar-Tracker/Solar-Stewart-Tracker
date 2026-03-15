# Build and Run

Target platform: Raspberry Pi OS (Linux)
Development platforms: Linux / Windows (CMake + C++17)

---

## Repository structure

- `include/` - public headers
- `src/` - implementation
- `src/app/` - bootstrap modules (config + factory + event-loop)
- `src/qt/` - Qt UI application
- `docs/` - assessment documentation
- `diagrams/` - PNG + Mermaid diagrams
- `tests/` - unit tests

---

## Prerequisites

### 2.1 Raspberry Pi (Target Platform)

OS:
- Raspberry Pi OS (Debian-based)

Install required tools:

```bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config
```

Optional (Raspberry Pi camera via libcamera):

```bash
sudo apt install -y libcamera-dev
```

Optional (Qt UI build on Linux):

```bash
sudo apt install -y qtbase5-dev qtcharts5-dev qt5-qmake
```

Optional (legacy OpenCV viewer target):

```bash
sudo apt install -y libopencv-dev
```

Verify CMake version:

```bash
cmake --version
```

Minimum required:
CMake >= 3.16

---

### 2.2 Windows (Development Only)

Install:

- Visual Studio 2019 or 2022
- "Desktop development with C++" workload
- CMake (bundled or standalone)

Verify:

```bash
cmake --version
```

---

## Clone

```bash
git clone <https://github.com/Real-Time-Stewart-Solar-Tracker/Solar-Stewart-Tracker>
```

---

## Configure

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
```

Common options:
- `-DSOLAR_TRY_OPENCV=ON/OFF` (default: ON, enables the OpenCV viewer when OpenCV is found)

---

## Build

```bash
cmake --build build -j
```

Outputs (typical):
- `solar_tracker_core` (core library)
- `solar_tracker` (CLI / headless app)
- `solar_tracker_qt` (Qt UI app, if enabled)
- unit tests: `test_core`, `test_pca9685`, `test_servodriver`

---

## Run unit tests

```bash
ctest --test-dir build --output-on-failure
```

---

## Run

CLI app:

```bash
./build/solar_tracker
```

Qt app:

```bash
./build/solar_tracker_qt
```

---

## Notes on optional features

- **libcamera (Pi only):** enabled automatically when `libcamera-dev` is present.
- **Qt UI:** built automatically when Qt5 Widgets is detected.
- **UiViewer (OpenCV):** a legacy diagnostic viewer target; it is not used by the Qt application.

---

## Doxygen

```bash
doxygen Doxyfile
```

Output:
- `docs/doxygen/html/index.html`

---

## Clean build

```bash
rm -rf build
cmake -S . -B build
cmake --build build -j
```
