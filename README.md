# SunLock Lab Solar Stewart Tracker

<p align="center">
  <img src="media/logo.jpeg" alt="SunLock Lab Solar Stewart Tracker" width="750">
</p>


Real-time embedded **C++17** software for solar tracking using a **3-RRS Stewart-type parallel mechanism** on **Raspberry Pi / Linux**.

This project implements an event-driven pipeline in which camera frames are delivered through callbacks, processed by vision and control modules, converted into platform motion through inverse kinematics, and safely applied to three actuators through a hardware abstraction layer. The runtime is structured around blocking waits, callback-driven stage transitions, bounded queues, and modular components so the software remains responsive, maintainable, testable, and reproducible.

<p align="center">
  <img src="media/system_overview.jpeg" alt="System Overview" width="750">
</p>
<p align="center">
  <img src="media/system_section.jpeg" alt="System Section" width="750">
</p>
<p align="center">
  <img src="media/model.jpeg" alt="System Model" width="750">
</p>

---
<p align="center">
  <strong>Explore the full generated developer reference, architecture pages, and API documentation.</strong><br>
  📘 <a href="https://real-time-stewart-solar-tracker.github.io/Solar-Stewart-Tracker/">
    Doxygen Documentation
  </a>
</p>

---
## Table of Contents

- [Social Media](#social-media)
- [Project Overview](#project-overview)
- [Project Management](#project-management)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
- [Sequence Diagram](#sequence-diagram)
- [Circuit Diagram](#circuit-diagram)
- [Repository Structure](#repository-structure)
- [Bill of Materials](#bill-of-materials)
- [Dependencies](#dependencies)
- [Cloning](#cloning)
- [Building](#building)
- [Running](#running)
- [Running Tests](#running-tests)
- [Realtime Evidence](#realtime-evidence)
- [Documentation](#documentation)
- [Authors and Contributions](#authors-and-contributions)
- [Acknowledgements](#acknowledgements)
- [License](#license)
- [Future Work](#future-work)
- [Last Updated](#last-updated)

---

## Social Media

We actively document the development, testing, and realtime performance of the **Solar Stewart Tracker** to promote transparency, reproducibility, and engagement with the engineering community.

📌 **TikTok (Primary Platform)**  
https://www.tiktok.com/@sunlock.lab_2

Content includes:

- realtime tracking demonstrations
- hardware setup and wiring
- development progress updates
- testing and debugging clips

---

## Project Overview

The **Solar Stewart Tracker** is a Linux userspace realtime embedded system that:

- acquires camera frames from a camera backend
- detects the sun position using image processing
- computes tracking corrections
- converts desired motion into **3-RRS inverse kinematics**
- applies safety shaping before actuation
- drives three servo outputs through a PCA9685 PWM controller

The design goal is not only functional tracking, but tracking implemented in a way that is:

- **event-driven**
- **responsive**
- **modular**
- **testable**
- **reproducible**
- **safe for hardware control**

The main software path is:

**Camera → SunTracker → Controller → ManualImuCoordinator → Kinematics3RRS → ActuatorManager → ServoDriver**

---

## Project Management

Development was tracked through GitHub issues, milestones, branches, pull requests, and tagged releases.

See:
- Issues for feature planning and bug tracking
- Milestones for delivery stages
- Pull requests for reviewed changes
- Releases 
---

## Key Features

- **Realtime event-driven processing**
  - callback-based frame delivery
  - blocking worker threads
  - no busy-wait control loop in the processing pipeline
  - no sleep-based timing in the control path

- **Modular C++17 architecture**
  - `SystemManager` orchestrates the runtime pipeline
  - hardware-facing and processing modules are separated by clear interfaces
  - optional backends and UI layers do not change the core architecture

- **Multiple build and use modes**
  - Linux / Raspberry Pi execution
  - software-only path with simulated camera input
  - headless CLI application
  - optional Qt GUI target

- **Safety-oriented actuation path**
  - actuator clamping
  - optional slew/rate limiting
  - neutral / park behaviour on start and stop
  - explicit handling of low-confidence tracking conditions
  - fault-driven suppression of invalid actuation commands

- **Engineering evidence built into the repository**
  - CMake-based build
  - CTest-integrated automated tests
  - latency measurement exported to artefacts
  - Doxygen-ready source tree
  - structured repository layout by subsystem

---

## System Architecture

<p align="center">
  <img src="diagrams/Threaded Event-Driven System Architecture Diagram.png" alt="System Architecture" width="900">
</p>

The repository is organised around a staged runtime pipeline.

### Core Modules

- **ICamera**  
  Abstract camera interface used by the system.

- **LibcameraPublisher**  
  Raspberry Pi / Linux camera backend when `libcamera` is available.

- **SimulatedPublisher**  
  Fallback or simulation camera backend.

- **SystemManager**  
  Top-level orchestrator for state, queues, callbacks, and worker threads.

- **SunTracker**  
  Vision module that finds the bright target and estimates sun position.

- **Controller**  
  Converts tracking error into platform tilt/pan setpoints.

- **ManualImuCoordinator**  
  Coordinates manual input ownership and optional IMU-based correction.

- **Kinematics3RRS**  
  Converts platform setpoints into actuator-space commands.

- **ActuatorManager**  
  Safety shaping layer for command limiting.

- **ServoDriver**  
  Final actuator output layer.

- **LatencyMonitor**  
  Measures timing across the userspace pipeline.

### Threading Model

The runtime is structured as a **multi-threaded, event-driven system** where **blocking I/O is used to wake up threads**, and callbacks propagate data through the processing pipeline.

Separate execution contexts are used for:

- **camera / backend context**: frame acquisition and callback delivery
- **control thread**: vision, control, coordination, and kinematics
- **actuator thread**: safety filtering and output application
- **main / event loop**: application lifecycle and optional UI / event handling

### Queue Policy

Inter-stage communication uses bounded queues with a freshest-data policy:

- **frame queue capacity:** 2
- **command queue capacity:** 8

Both queues use `push_latest(...)`. When a queue is full, the oldest item is discarded before the newest item is inserted. This bounds memory use, prevents unbounded backlog, and keeps the control path biased toward current data rather than stale queued work.

---

## Sequence Diagram

```mermaid
sequenceDiagram
    actor User
    participant Cam as Camera
    participant SM as SystemManager
    participant BC as BackendCoordinator
    participant GMD as GuiManualDispatcher
    participant FQ as FrameQueue
    participant CT as Control thread
    participant ST as SunTracker
    participant C as Controller
    participant MIC as ManualImuCoordinator
    participant K as Kinematics3RRS
    participant CQ as CommandQueue
    participant AT as Actuator thread
    participant AM as ActuatorManager
    participant SD as ServoDriver
    participant ADS as ADS1115
    participant IMU as MPU6050
    participant LM as LatencyMonitor

    User->>SM: start()
    SM->>SD: start()
    SM->>GMD: start()
    SM->>BC: start()
    Note over BC: starts ADS1115 and IMU backends
    SM->>Cam: start()
    SM->>SM: state = SEARCHING / MANUAL

    IMU-->>SM: imu sample callback
    SM->>MIC: updateImuSample(sample)

    rect rgb(245,245,255)
    Note over Cam,SD: Automatic path — control thread only
    Cam-->>SM: FrameEvent callback
    SM->>LM: onCapture(...)
    SM->>FQ: push_latest(frame)

    CT->>FQ: wait_pop()
    FQ-->>CT: FrameEvent
    CT->>ST: onFrame(frame)
    ST-->>CT: SunEstimate callback
    CT->>LM: onEstimate(...)
    CT->>C: onEstimate(estimate)
    C-->>CT: PlatformSetpoint callback
    CT->>LM: onControl(...)
    CT->>MIC: applyImuCorrection(setpoint)
    MIC-->>CT: corrected setpoint
    CT->>K: onSetpoint(setpoint)
    K-->>CT: ActuatorCommand callback
    CT->>CQ: push_latest(command)
    end

    rect rgb(245,255,245)
    Note over ADS,K: Pot manual path — ADS1115 callback thread, independent of camera frames
    ADS-->>SM: ManualPotSample callback
    SM->>MIC: buildManualSetpointFromPot(...)
    MIC-->>SM: PlatformSetpoint
    SM->>MIC: applyImuCorrection(sp)
    MIC-->>SM: corrected setpoint
    SM->>LM: onControl(...)
    SM->>K: onSetpoint(setpoint)
    K-->>SM: ActuatorCommand callback
    SM->>CQ: push_latest(command)
    end

    rect rgb(255,250,240)
    Note over User,K: GUI manual path — GuiManualDispatcher thread, independent of camera frames
    User->>SM: enterManual()
    SM->>SM: state = MANUAL

    User->>SM: setManualSetpoint(tilt, pan)
    SM->>GMD: setSetpoint(tilt, pan)
    Note over GMD: push_latest wakes thread immediately
    GMD->>MIC: buildManualSetpointFromGui(...)
    MIC-->>GMD: PlatformSetpoint
    GMD->>MIC: applyImuCorrection(sp)
    MIC-->>GMD: corrected setpoint
    GMD->>LM: onControl(...)
    GMD->>K: onSetpoint(setpoint)
    K-->>GMD: ActuatorCommand callback
    GMD->>CQ: push_latest(command)

    User->>SM: exitManual()
    SM->>SM: state = SEARCHING
    end

    rect rgb(240,240,240)
    Note over CQ,SD: Actuator path — actuator thread
    AT->>CQ: wait_pop()
    CQ-->>AT: ActuatorCommand
    AT->>AM: onCommand(command)
    AM-->>AT: safe command callback
    AT->>LM: onActuate(...)
    AT->>SD: apply(command)
    end

    User->>SM: stop()
    SM->>Cam: stop()
    SM->>BC: stop()
    SM->>GMD: stop()
    SM->>SD: stop()
    SM->>SM: state = IDLE
```

---

## Circuit Diagram

<p align="center">
  <img src="diagrams/circuit_diagram.jpeg" alt="Circuit Diagram" width="900">
</p>

The hardware setup connects:

- camera through a libcamera-compatible interface
- PCA9685 PWM driver over I2C
- ADS1115 over I2C for manual input
- MPU6050 over I2C for feedback
- three servo motors
- Raspberry Pi acting as the central controller

The PCA9685 generates PWM signals for the servos, while I2C provides communication between the Raspberry Pi, the manual-input ADC, the IMU, and the actuator driver layer.

> **Exact connection mapping:** see [`docs/hardware_connections.md`](docs/hardware_connections.md).  
> This document is the source of truth for physical Raspberry Pi pin numbers, I2C wiring, GPIO interrupt pins, servo channel assignments, potentiometer wiring, and shared-ground requirements.

---

## Repository Structure

```text
Solar-Stewart-Tracker/
├── .github/
│   └── workflows/
├── artefacts/
├── external/
│   ├── libcamera2opencv/
│   └── libgpiod_event_demo/
├── scripts/
├── src/
│   ├── actuators/
│   │   └── tests/
│   ├── app/
│   ├── common/
│   │   └── tests/
│   ├── control/
│   │   └── tests/
│   ├── hal/
│   │   └── tests/
│   ├── qt/
│   ├── sensors/
│   │   └── tests/
│   ├── system/
│   │   └── tests/
│   ├── tests/
│   │   └── support/
│   └── vision/
│       └── tests/
├── CMakeLists.txt
├── CONTRIBUTING.md
├── Doxyfile
└── LICENSE
```

---

## Bill of Materials

### Controller

| Component | Quantity | Cost (£) |
|---|---:|---:|
| Raspberry Pi 5 (8GB) | 1 | 80.00 |

### Sensors and Vision

| Component | Quantity | Cost (£) |
|---|---:|---:|
| IMX219 Camera Module | 1 | 25.00 |

### Actuation and Drive

| Component | Quantity | Cost (£) |
|---|---:|---:|
| PCA9685 PWM Driver | 1 | 12.00 |
| High-Torque Servo (RDS3230 or equivalent) | 3 | 45.00 |
| External 5–6V High-Current Supply | 1 | 20.00 |

### Mechanical and Supporting Components

| Component | Quantity | Cost (£) |
|---|---:|---:|
| Breadboard and Wiring Set | 1 | 10.00 |
| Structural Frame / 3D Printed Parts | 1 | 15.00 |
| Fasteners and Mounts | Assorted | 10.00 |

**Grand Total: £217.00**

---

## Dependencies

### Mandatory

- CMake 3.16 or newer
- C++17 compiler
  - GCC
  - Clang
  - MSVC

### Optional

- **libcamera**  
  Enables the Raspberry Pi camera backend.

- **Qt5 Widgets / Charts**  
  Enables the optional Qt GUI target.

- **OpenCV**  
  Enables optional image-conversion and viewer support where available.

### Linux Packages

#### Minimal build tools

```bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config
```

#### Optional packages

```bash
sudo apt install -y libcamera-dev
sudo apt install -y qtbase5-dev qtcharts5-dev qt5-qmake
sudo apt install -y libopencv-dev
sudo apt install -y doxygen graphviz
```

---

## Cloning

Clone the repository with all submodules:

```bash
git clone --recurse-submodules https://github.com/Real-Time-Stewart-Solar-Tracker/Solar-Stewart-Tracker.git
cd Solar-Stewart-Tracker
```

If you already cloned without submodules, initialise them with:

```bash
git submodule update --init --recursive
```

### External Repositories Used by This Project

The repository includes these submodules under `external/`:

```text
external/libcamera2opencv
external/libgpiod_event_demo
```

Their configured sources are:

- `external/libcamera2opencv` → `https://github.com/berndporr/libcamera2opencv.git`
- `external/libgpiod_event_demo` → `https://github.com/berndporr/libgpiod_event_demo.git`
---

## Building

### Linux / Raspberry Pi OS

#### Configure

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
```

#### Build

```bash
cmake --build build -j
```

### Windows

#### Configure

```bash
cmake -S . -B build
```

#### Build

```bash
cmake --build build --config Release
```

### Optional: Disable OpenCV Auto-Detection

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DSOLAR_TRY_OPENCV=OFF
```

### Optional: Disable libcamera Auto-Detection

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DSOLAR_TRY_LIBCAMERA=OFF
```

### Optional: Enable Hardware-Adjacent Tests

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DSOLAR_ENABLE_HW_TESTS=ON
```

---

## Running

### Core CLI Application

#### Linux

```bash
./build/solar_tracker
```

#### Typical Windows Location

```text
build\Release\solar_tracker.exe
```

### Software-Only Mode

The software-only path uses a simulated camera backend and a non-hardware actuator path for development and testing without the physical platform.

### Optional Qt GUI Application

Built only when Qt support is enabled and found:

```bash
./build/src/qt/solar_tracker_qt
```

#### Typical Windows Location

```text
build\Release\solar_tracker_qt.exe
```

### Hardware Mode

Hardware execution requires:

- libcamera support for the live camera path
- I2C enabled on the host
- PCA9685 connected correctly
- servo power and wiring connected correctly

The system enters `FAULT` if required startup steps fail or mandatory hardware is unavailable.

### Runtime Latency Capture

```bash
./scripts/run_latency.sh
```

This writes latency data to:

```text
artefacts/latency.csv
```

For a fuller command reference, see [`docs/build_and_run.md`](docs/build_and_run.md).

---

## Running Tests

This project integrates tests with **CTest**.

### Run All Registered Tests

#### Linux

```bash
ctest --test-dir build --output-on-failure -LE hw
```

#### Windows

```bash
ctest --test-dir build -C Release --output-on-failure
```

### Convenience Script

```bash
./scripts/test_core.sh
```

### Hardware-Adjacent Test Script

```bash
./scripts/test_pi_hw.sh
```

### Included Automated Test Areas

- SunTracker
- Controller
- ManualInputMapper
- ImuFeedbackMapper
- ImuTiltEstimator
- ActuatorManager
- ThreadSafeQueue
- Kinematics3RRS
- LatencyMonitor
- SystemManager state handling
- PCA9685
- ServoDriver
- MPU6050 publisher
- Linux I2C hardware smoke path

---

## Realtime Evidence

The measured software-side pipeline is:

**Camera → FrameQueue → SunTracker → Controller → ManualImuCoordinator → Kinematics3RRS → ActuatorManager → ServoDriver**

All measurements are taken using monotonic timestamps recorded inside the software pipeline and exported to `artefacts/latency.csv`.

### Latency Results

| Metric | Average (ms) | Minimum (ms) | Maximum (ms) | Jitter (ms) |
|---|---:|---:|---:|---:|
| L_total | 8.369570 | 6.829599 | 14.565364 | 7.735765 |
| L_vision | 8.242496 | 6.755912 | 14.530086 | 7.774174 |
| L_control | 0.014822 | 0.006038 | 0.363637 | 0.357599 |
| L_actuation | 0.112253 | 0.019426 | 3.095969 | 3.076543 |

### Interpretation

- average end-to-end software latency is approximately **8.37 ms**
- worst-case measured software latency is approximately **14.57 ms**
- measured timing remains below a typical **30 Hz** frame period of approximately **33 ms**
- measurements represent the userspace software path only, not full physical actuator motion or mechanical settling time

---

## Documentation

The repository includes a Doxygen configuration file for API documentation generation.

### Generate Doxygen Documentation

```bash
doxygen Doxyfile
```

Open locally at:

```text
docs/html/index.html
```
## Generated Doxygen documentation for the codebase is available here:

- **Project Documentation Website:**  
  https://real-time-stewart-solar-tracker.github.io/Solar-Stewart-Tracker/

### Source Areas Covered by the Documentation Build

- `src/actuators`
- `src/app`
- `src/common`
- `src/control`
- `src/hal`
- `src/sensors`
- `src/system`
- `src/vision`
- `src/qt`

Additional project documentation is provided under `docs/`, including:

- `docs/project_overview.md`
- `docs/system_architecture.md`
- `docs/state_machine.md`
- `docs/realtime_analysis.md`
- `docs/latency_measurement.md`
- `docs/testing.md`
- `docs/solid_justification.md`
- `docs/requirements.md`
- `docs/user_stories_use_cases.md`
- `docs/DEPENDENCIES.md`
- `docs/REPRODUCIBILITY.md`
- `docs/build_and_run.md`
- `docs/hardware_connections.md`

---

## Authors and Contributions

**Fadi Halteh (3127931H)**  
Designed and implemented the event-driven system architecture, including runtime orchestration, state handling, and the bounded queue pipeline. Responsible for integrating the major software stages into the full runtime path.  
Implemented the MPU6050 (IMU) and ADS1115 (manual input) integration within the system, including their event-driven data handling and coordination with the control pipeline.  Also responsible for the system architecture design and creation of the technical diagrams (pipeline, threading, and system architecture).

**Jichao Wang (3137140W)**  
Developed the 3-RRS inverse kinematics model and core application setup, including configuration, factory creation, and application entry structure. Responsible for translating platform setpoints into actuator-space commands.

**Ziming Yan (2429452Y)**  
Developed the vision subsystem and user interface components, including the SunTracker detection pipeline and Qt-based control panel. Integrated visual feedback, overlays, and runtime interaction into the system.

**Tareq A M Almzanin (3139787A)**  
Implemented the control layer translating vision estimates into platform motion, including closed-loop control logic and manual override behaviour. Contributed to the definition of shared data types and the control-side software path.

**Zhenyu Zhu (3099498Z)**  
Implemented the low-level actuator interface including PCA9685 integration and servo control, along with latency measurement instrumentation. Responsible for hardware abstraction and timing analysis across the system pipeline.

*Source: project members document.*

---

## Acknowledgements

We would like to thank:

- Dr. Bernd Porr for guidance in realtime embedded systems and architecture design
- Dr. Chongfeng Wei for software engineering support and project supervision
- the University of Glasgow
- the laboratory, workshop, and technical support staff involved in supporting the project

Their guidance and infrastructure helped shape both the realtime architecture and the engineering process behind this repository.

---

## License

This project is released under the license included in this repository:

```text
LICENSE
```

### External Components and Attribution

The repository also includes or depends on external open-source components. Please credit and use them according to their original licenses.

#### libcamera2opencv
- The libcamera-to-OpenCV wrapper is sourced from Bernd Porr.
- Repository: https://github.com/berndporr/libcamera2opencv

#### libgpiod_event_demo
- GPIO event-driven reference code is sourced from Bernd Porr.
- Repository: https://github.com/berndporr/libgpiod_event_demo

---

## Future Work

Planned or natural next extensions include:

- richer telemetry and live plotting
- enhanced fault handling and recovery strategies
- broader hardware-backed integration testing
- further public-facing project media and demonstration material

---

## Last Updated

April 2026