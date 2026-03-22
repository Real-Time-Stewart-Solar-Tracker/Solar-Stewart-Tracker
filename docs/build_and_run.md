# Build and Run

Target platform: **Raspberry Pi OS (Linux)**  
Development platforms: **Linux / Windows** (CMake + C++17)

This project is designed to be reproducible:

- one CMake build for all targets
- optional features are auto-detected where available
- automated tests integrate with **CTest**

------------------------------------------------------------------------

## 1) Repository structure

- `include/` --- public headers
- `src/` --- implementation
- `src/app/` --- bootstrap modules (configuration, factory, event loop)
- `src/qt/` --- optional Qt UI application
- `src/ui/` --- optional OpenCV viewer support compiled into the CLI build when OpenCV is found
- `docs/` --- assessment documentation
- `diagrams/` --- PNG and Mermaid diagrams
- `tests/` --- automated tests plus one manual hardware executable

------------------------------------------------------------------------

## 2) Dependencies (quick overview)

**Always required**
- CMake ≥ 3.16
- a C++17 compiler (GCC / Clang / MSVC)
- Make or Ninja on Linux, or Visual Studio build tools on Windows

**Optional**
- **Qt5** --- builds the optional Qt GUI target when found
- **OpenCV** --- enables viewer support in the CLI application when found
- **libcamera** --- enables the Raspberry Pi camera backend on Linux when found

For full details see: `docs/DEPENDENCIES.md`

------------------------------------------------------------------------

## 3) Clone

```bash
git clone https://github.com/Real-Time-Stewart-Solar-Tracker/Solar-Stewart-Tracker.git
cd Solar-Stewart-Tracker
````

---

## 4) Raspberry Pi OS / Linux build

### Install required tools

```bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config
```

### Optional: libcamera

```bash
sudo apt install -y libcamera-dev
```

### Optional: Qt5 GUI

```bash
sudo apt install -y qtbase5-dev qtcharts5-dev qt5-qmake
```

### Optional: OpenCV

```bash
sudo apt install -y libopencv-dev
```

---

## 5) Configure and build (Linux / Raspberry Pi)

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

After building, executables are placed in the `build/` directory.

Run automated tests:

```bash
ctest --test-dir build --output-on-failure
```

---

## 6) Run executables

Core CLI application:

```bash
./build/solar_tracker
```

Qt GUI application (MAIN FULL APP)
(only if Qt5 was found during configuration):

```bash
./build/solar_tracker_qt
```

Manual hardware executable
(not part of `ctest`):

```bash
./build/tests/servo_manual_smoketest
```

---

## 7) Windows build

Configure:

```powershell
cmake -S . -B build
```

Build:

```powershell
cmake --build build --config Release
```

Run tests:

```powershell
ctest --test-dir build -C Release --output-on-failure
```


