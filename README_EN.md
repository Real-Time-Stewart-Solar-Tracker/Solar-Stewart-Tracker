# 3RRS Visual Tracking System - C++ Version

A 3RRS parallel mechanism visual tracking system based on Raspberry Pi 5 and OpenCV.

---

## 📋 Project Overview

This project implements a complete visual tracking control system using C++17:
- **PCA9685 Servo Driver**: Controls three servos via I2C (RAII resource management)
- **3RRS Inverse Kinematics**: Fully aligned with MATLAB sun211.m algorithm
- **PID Controller**: Platform pose control
- **OpenCV Vision System**: HSV color-space red target detection (callback-push mode)
- **TrackerController**: Event-driven control core (dependency injection + callback chain)

---

## 🏗️ SOLID Design Principles

This project strictly follows SOLID object-oriented design principles:

### Single Responsibility
Each class is responsible for one independent function:

| Class | Responsibility |
|-------|---------------|
| `PCA9685` | I2C communication and PWM servo control |
| `RRSKinematics` | 3RRS parallel mechanism inverse kinematics solver |
| `PIDController` | PID control algorithm computation |
| `VisionSystem` | Camera capture and color detection |
| `TrackerController` | Component coordination and control flow |

### Open/Closed Principle
- Extend functionality through abstract interfaces without modifying existing code
- Example: Implement a new `IServoDriver` to support different servo driver boards

### Liskov Substitution
- All interface implementations can be seamlessly swapped
- Example: Replace PCA9685 with MockServoDriver for unit testing

### Interface Segregation
- `IServoDriver`: Exposes only `setServoAngle()` and `close()`
- `IKinematicsSolver`: Exposes only `solve()`
- `IVisionSource`: Exposes only `start()`, `stop()`, `setCallback()`, `cleanup()`

### Dependency Inversion
- `TrackerController` depends on abstract interfaces, not concrete implementations
- Dependencies injected via constructor using `unique_ptr<IServoDriver>`, etc.

---

## ⚡ Real-Time Architecture

### Event-Driven Architecture (Non-Polling)

```
┌─────────────────┐    Callback         ┌───────────────────────┐
│   VisionSystem  │ ──────────────────→ │   TrackerController   │
│ (Background     │    onVisionUpdate   │ (condition_variable   │
│  capture thread)│                     │  timer thread @ 50Hz) │
│ libcamera I/O   │                     │                       │
└─────────────────┘                     │  PID → IK → Servo    │
                                        └───────────────────────┘

Main thread: sigwait(SIGINT/SIGTERM) — zero-polling exit wait
```

### Key Real-Time Techniques
- **Callback Chain**: Vision → `onVisionUpdate()` → PID → IK → Servo
- **Condition Variable Timer**: `condition_variable::wait_for` achieves precise 50Hz control frequency (instantly wakeable by stop signal)
- **Signal Wait**: Main thread uses `sigwait()` to block-wait for exit signals, zero CPU consumption
- **Thread Safety**: `std::mutex` + `std::atomic` protect all shared state

### Latency Analysis

| Stage | Duration | Description |
|-------|----------|-------------|
| Camera frame interval | ~33ms | 30fps libcamera pipeline |
| Vision processing | ~5ms | HSV conversion + morphology + contour detection |
| Callback delivery | <0.1ms | mutex lock/unlock |
| PID computation | <0.1ms | Floating-point arithmetic |
| IK solving | ~0.5ms | 3 sets of trigonometric functions + Eigen matrix ops |
| I2C servo communication | ~1ms | 3 channels × 5-byte writes |
| **End-to-end latency** | **~40ms** | **Meets real-time tracking requirements** |

---

## 💾 Memory Management

- All components managed with `std::unique_ptr` (RAII)
- Zero raw `new`/`delete` calls
- PCA9685 supports move semantics (move-only type)
- Completely leak-free

---

## 🚀 Quick Start

### 1. Install Dependencies

```bash
# Update system
sudo apt-get update

# Install build tools and libraries
sudo apt-get install -y \
    build-essential \
    cmake \
    libi2c-dev \
    libopencv-dev \
    libeigen3-dev

# Install Google Test (optional, for unit tests)
sudo apt-get install -y libgtest-dev
cd /usr/src/gtest && sudo cmake . && sudo make && sudo cp lib/*.a /usr/lib/

# Enable I2C (if not already enabled)
sudo raspi-config
# Select: Interfacing Options > I2C > Enable
```

### 2. Build the Project

```bash
cd cpp_tracker
mkdir -p build && cd build
cmake ..
make
```

Build outputs:
- `main_tracker` — Main program
- `test_servos` — Servo hardware test
- `test_circular` — Circular trajectory test
- `test_pid` — PID unit tests (requires GTest)
- `test_kinematics_unit` — IK unit tests (requires GTest)

### 3. Run Unit Tests

```bash
cd build
ctest --verbose
```

### 4. Run the Main Program

```bash
./main_tracker
# Press Ctrl+C for graceful shutdown
```

---

## 📁 Project Structure

```
cpp_tracker/
├── CMakeLists.txt              # CMake config (C++17 + GTest)
├── README.md                   # Documentation (Chinese)
├── README_EN.md                # Documentation (English)
├── include/                    # Header files
│   ├── interfaces.hpp          # SOLID abstract interface definitions
│   ├── pca9685.hpp             # PCA9685 servo driver (IServoDriver)
│   ├── kinematics.hpp          # 3RRS inverse kinematics (IKinematicsSolver)
│   ├── pid_controller.hpp      # PID controller
│   ├── vision_system.hpp       # OpenCV vision system (IVisionSource)
│   └── tracker_controller.hpp  # Event-driven controller
├── src/                        # Source files
│   ├── pca9685.cpp
│   ├── kinematics.cpp
│   ├── pid_controller.cpp
│   ├── vision_system.cpp
│   ├── tracker_controller.cpp
│   └── main_tracker.cpp        # Main program entry point
└── tests/                      # Tests
    ├── test_pid.cpp             # PID unit tests (GTest)
    ├── test_kinematics.cpp      # IK unit tests (GTest)
    ├── test_servos.cpp          # Servo hardware test
    └── test_circular.cpp        # Circular trajectory hardware test
```

---

## 🔧 Hardware Connections

### I2C Wiring (PCA9685)
| Raspberry Pi Pin | PCA9685 |
|-----------------|---------|
| GPIO2 (SDA) | SDA |
| GPIO3 (SCL) | SCL |
| GND | GND |

### Servo Power Supply
- Connect PCA9685 V+ to 5–6V power supply positive terminal
- Connect GND to power supply negative terminal and Raspberry Pi GND

### Servo Channels
- Channel 0 → Servo 1
- Channel 1 → Servo 2
- Channel 2 → Servo 3

### Camera
- Connect CSI camera to the Raspberry Pi CSI interface

---

## ⚙️ System Parameters

### Geometric Parameters
- Base platform radius Rb: 0.20m
- Moving platform radius Rp: 0.12m
- Platform height h: 0.18m
- Active arm length L1: 0.10m
- Passive arm length L2: 0.18m

### Control Parameters
- **PID Gains**: Kp=0.3, Ki=0.0, Kd=0.0
- **Max Tilt**: 35°
- **Control Frequency**: 50Hz
- **Servo Smoothing Factor**: 0.15

### Vision Parameters
- **Resolution**: 640x480
- **HSV Red Detection**:
  - Range 1: H[0,10], S[50,255], V[50,255]
  - Range 2: H[160,180], S[50,255], V[50,255]
- **Minimum Contour Area**: 100 pixels
- **Smoothing Window**: 5 frames

---

## 🧪 Testing Strategy

### Unit Tests (Google Test)

**PIDController Tests** (`tests/test_pid.cpp`):
- Initial state verification
- Proportional/Integral/Derivative control response
- Output clamping (positive & negative)
- Reset functionality
- Parameter storage validation
- Boundary conditions (dt=0)

**RRSKinematics Tests** (`tests/test_kinematics.cpp`):
- Zero-angle neutral position verification
- Output range safety
- Continuity verification (no sudden jumps)
- Pitch/Roll symmetry
- Extreme angle stability
- Convergence verification

### Hardware Integration Tests

- `test_servos` — Verify servo I2C communication and motion
- `test_circular` — Verify IK + servo coordinated circular trajectory

---

## 🐛 Troubleshooting

### 1. Build Error: OpenCV Not Found
```bash
sudo apt-get install libopencv-dev
```

### 2. Build Error: Eigen3 Not Found
```bash
sudo apt-get install libeigen3-dev
```

### 3. GTest Not Found
```bash
sudo apt-get install libgtest-dev
cd /usr/src/gtest && sudo cmake . && sudo make && sudo cp lib/*.a /usr/lib/
```

### 4. Runtime Error: Failed to open I2C bus
```bash
# Verify I2C is enabled
ls /dev/i2c-*
sudo raspi-config  # Enable I2C
```

### 5. Runtime Error: Cannot open camera
```bash
libcamera-hello  # Detect camera
```

---

## 📄 License

MIT License

---

## 👨‍💻 Development Info

- **C++ Standard**: C++17
- **Target Platform**: Raspberry Pi 5 (ARM64)
- **OpenCV Version**: 4.x
- **CMake Version**: >= 3.10
- **Test Framework**: Google Test
