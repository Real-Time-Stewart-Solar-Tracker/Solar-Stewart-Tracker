# Build and Run

This file gives the exact repository setup, submodule checkout, build, test, and run commands for the  project .

---

## Cloning

To clone with all submodules in one step, run:

```bash
git clone --recurse-submodules https://github.com/Real-Time-Stewart-Solar-Tracker/Solar-Stewart-Tracker.git
cd Solar-Stewart-Tracker
```

If you already cloned without submodules, run:

```bash
git submodule update --init --recursive
```

---

## External repositories used by this project

The project uses these external repositories as git submodules under `external/`, use these commands if you dont have them:

```bash
git submodule add https://github.com/berndporr/libcamera2opencv.git external/libcamera2opencv
git submodule add https://github.com/berndporr/libgpiod_event_demo.git external/libgpiod_event_demo
```

After adding them, fetch their contents with:

```bash
git submodule update --init --recursive
```

To verify they are present, run:

```bash
git submodule status
ls external
```

If you ever need to refresh them to the commits recorded by the main repository, run:

```bash
git submodule sync --recursive
git submodule update --init --recursive --checkout
```

To pull the latest remote changes for the submodules:

```bash
git submodule update --remote --recursive
```

---

## If the external folders are missing

If the repository was copied manually and the `external/` repositories are not populated, use:

```bash
git submodule update --init --recursive
```

That will place the repositories at these exact paths:

```text
external/libcamera2opencv
external/libgpiod_event_demo
```

---

## Linux dependencies

### libgpiod v2 — required, must be built from source

The project requires the C++ bindings introduced in libgpiod v2. The `apt` package on Raspberry Pi OS provides v1.x only. Build v2.2.2 from source:

```bash
sudo apt install -y autoconf autoconf-archive automake libtool wget xz-utils

cd /tmp
wget https://mirrors.edge.kernel.org/pub/software/libs/libgpiod/libgpiod-2.2.2.tar.xz
tar -xf libgpiod-2.2.2.tar.xz
cd libgpiod-2.2.2
./configure --prefix=/usr/local --enable-tools --enable-bindings-cxx
make -j$(nproc)
sudo make install
sudo ldconfig
```

Verify:
```bash
pkg-config --modversion libgpiod   # should print 2.2.2
```


### Core build dependencies

```bash
sudo apt update
sudo apt install -y build-essential cmake ninja-build git pkg-config
# Note: libgpiod-dev via apt provides v1.x only. This project requires v2.
# See the libgpiod v2 build-from-source instructions below.
```

### Optional Qt GUI dependencies

```bash
sudo apt install -y qtbase5-dev libqt5charts5-dev qt5-qmake
```

### Optional OpenCV and libcamera dependencies

```bash
sudo apt install -y libopencv-dev libcamera-dev
```

### Optional hardware tools

```bash
sudo apt install -y i2c-tools
```

### Optional documentation tools

```bash
sudo apt install -y doxygen graphviz
```

---

## CMake options used by this project

```text
SOLAR_ENABLE_TESTS=ON        default
SOLAR_ENABLE_HW_TESTS=OFF    default
SOLAR_ENABLE_QT=ON           default
SOLAR_TRY_LIBCAMERA=ON       default
SOLAR_TRY_OPENCV=ON          default
SOLAR_ENABLE_COVERAGE=OFF    default
```

---

## Standard release build

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

If you do not want to use Ninja:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
```

---

## Minimal headless build

This disables the optional GUI, libcamera, and OpenCV paths:

```bash
cmake -S . -B build   -G Ninja   -DCMAKE_BUILD_TYPE=Release   -DSOLAR_ENABLE_QT=OFF   -DSOLAR_TRY_LIBCAMERA=OFF   -DSOLAR_TRY_OPENCV=OFF

cmake --build build --parallel
```

---

## Build with hardware tests enabled

```bash
cmake -S . -B build   -G Ninja   -DCMAKE_BUILD_TYPE=Debug   -DSOLAR_ENABLE_TESTS=ON   -DSOLAR_ENABLE_HW_TESTS=ON   -DSOLAR_ENABLE_QT=OFF   -DSOLAR_TRY_LIBCAMERA=OFF   -DSOLAR_TRY_OPENCV=OFF

cmake --build build --parallel
```

This same configuration is also wrapped by the repository helper script:

```bash
./scripts/build_pi_debug.sh build
```

---

## Main executables

After a successful build, the main runtime targets are:

```text
build/solar_tracker
build/src/qt/solar_tracker_qt   (only if Qt5 Widgets and Qt5 Charts were found)
```

---

## Run the application

### Headless application

```bash
./build/solar_tracker
```

### Qt GUI application

```bash
./build/solar_tracker_qt
```

---

## Run the software tests

```bash
ctest --test-dir build --output-on-failure
```

Repository helper script:

```bash
./scripts/test_core.sh build
```

---

## Run hardware-labelled tests on Raspberry Pi

The helper script sets default hardware environment variables and then runs CTest with the `hw` label.

```bash
./scripts/test_pi_hw.sh build
```

Equivalent explicit form:

```bash
export SOLAR_RUN_CAMERA_HW_TESTS=1
export SOLAR_RUN_I2C_HW_TESTS=1
export SOLAR_I2C_DEV=/dev/i2c-1
ctest --test-dir build -L hw --output-on-failure
```

---

## Manual hardware smoke tests

These are for controlled hardware checks and should only be run on the target machine with the hardware connected safely.

```bash
./build/src/actuators/tests/test_pca9685
./build/tests/test_servodriver
```

---

## Typical Raspberry Pi workflow

```bash
git clone --recurse-submodules https://github.com/Real-Time-Stewart-Solar-Tracker/Solar-Stewart-Tracker.git
cd Solar-Stewart-Tracker

git submodule update --init --recursive

cmake -S . -B build   -G Ninja   -DCMAKE_BUILD_TYPE=Release   -DSOLAR_ENABLE_QT=OFF   -DSOLAR_ENABLE_HW_TESTS=ON

cmake --build build --parallel

ctest --test-dir build --output-on-failure
./build/solar_tracker
```

---

## Clean rebuild

If the build directory becomes stale, rebuild from scratch:

```bash
rm -rf build build
git submodule sync --recursive
git submodule update --init --recursive
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

---

## Notes

- Use `--recurse-submodules` on the first clone so the external repositories are checked out immediately.
- If someone cloned the repository without submodules, `git submodule update --init --recursive` is the required recovery step.
- Keep the external repositories at the paths under `external/` so the build system can find them consistently.
