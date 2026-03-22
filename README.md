# SunLock Lab Solar Stewart Tracker

<p align="center">
  <img src="media/logo.jpeg" alt="System Overview" width="750">
</p>

Real-time embedded C++17 software for solar tracking using a **3-RRS Stewart-type parallel mechanism** on **Raspberry Pi / Linux**.

This project implements an event-driven pipeline in which camera frames are delivered through callbacks, processed by vision and control modules, converted into platform motion through inverse kinematics, and safely applied to three actuators through a hardware abstraction layer. The architecture is designed to follow the course expectations for **blocking I/O**, **callback-based communication**, **multi-threaded processing**, **bounded latency**, **modular structure**, and **CMake-based reproducibility**.

<p align="center">
  <img src="media/system_overview.jpeg" alt="System Overview" width="750">
</p>
<p align="center">
  <img src="media/system_section.jpeg" alt="System Overview" width="750">
</p>
<p align="center">
  <img src="media/model.jpeg" alt="System Overview" width="750">
</p>

## Table of Contents

- [Social Media](#social-media)
- [Project Overview](#project-overview)
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
- [Running Software Tests](#running-software-tests)
- [Realtime Evidence](#realtime-evidence)
- [Documentation](#documentation)
- [Authors and Contributions](#authors-and-contributions)
- [Acknowledgements](#acknowledgements)
- [License](#license)
- [Future Work](#future-work)

---

## Social Media

We actively document the development, testing, and real-time performance of the **Solar Stewart Tracker** to promote transparency, reproducibility, and engagement with the engineering community.

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
- converts desired motion into **3RRS inverse kinematics**
- applies safety shaping before actuation
- drives three servo outputs through a PCA9685 PWM controller

The design goal is not only functional tracking, but tracking implemented in a way that is:

- **event-driven**
- **responsive**
- **modular**
- **testable**
- **reproducible**
- **safe for hardware control**

The main realtime software path is:

**Camera -> SunTracker -> Controller -> Kinematics3RRS -> ActuatorManager -> ServoDriver**

---

## Key Features

- **Realtime event-driven processing**
  - callback-based frame delivery
  - blocking worker threads
  - no busy-wait control loop in the processing pipeline
  - no sleep-based fake realtime timing in the control path

- **Modular C++17 architecture**
  - `SystemManager` orchestrates the full pipeline
  - hardware-facing and processing modules are separated by clear interfaces
  - optional backends and UI layers do not change the core architecture

- **Multiple build/use modes**
  - Linux / Raspberry Pi execution
  - desktop simulation path
  - headless CLI application
  - optional Qt GUI target

- **Safety-oriented actuation path**
  - actuator clamping
  - optional slew/rate limiting
  - neutral/park behaviour on start and stop
  - explicit handling of low-confidence tracking conditions

- **Assessment-oriented engineering evidence**
  - CMake-based build
  - CTest-integrated automated tests
  - latency measurement
  - architecture and state-machine documentation
  - Doxygen-ready repository

---

## System Architecture

<p align="center">
  <img src="diagrams/Threaded Event-Driven System Architecture Diagram.png" alt="System Architecture" width="900">
</p>

The repository is organised around a staged runtime pipeline.

### Core modules

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

- **Kinematics3RRS**  
  Converts platform setpoints into actuator-space commands.

- **ActuatorManager**  
  Safety shaping layer for command limiting.

- **ServoDriver**  
  Final actuator output layer.

- **LatencyMonitor**  
  Measures timing across the userspace pipeline.

### Threading model

The runtime design uses separate responsibilities with blocking waits:

- **camera/backend context**: frame acquisition and callback delivery
- **control thread**: vision, control, and kinematics
- **actuator thread**: safety filtering and output application
- **main/event loop**: application lifecycle and optional UI/event handling

### Queue policy

Inter-stage communication uses bounded queues with a **freshest-data** policy:

- frame queue capacity: **1**
- command queue capacity: **1**

This avoids stale backlog and prioritises current data over historical frames.

---

## Sequence Diagram

```mermaid
sequenceDiagram
    actor User
    participant Cam as Camera Backend
    participant SM as SystemManager
    participant ST as SunTracker
    participant C as Controller
    participant K as Kinematics3RRS
    participant AM as ActuatorManager
    participant SD as ServoDriver

    User->>SM: Start tracker
    SM->>SD: start()
    SM->>Cam: start()
    SM->>SM: setState(STARTUP)
    SM->>SD: apply startup park
    SM->>SM: setState(NEUTRAL)
    SM->>SM: setState(SEARCHING)

    Cam-->>SM: FrameEvent callback
    SM->>ST: onFrame(FrameEvent)
    ST-->>SM: SunEstimate

    alt confidence >= threshold
        SM->>SM: setState(TRACKING)
        SM->>C: onEstimate(SunEstimate)
        C-->>SM: PlatformSetpoint
        SM->>K: onSetpoint(PlatformSetpoint)
        K-->>SM: ActuatorCommand
        SM->>AM: onCommand(ActuatorCommand)
        AM-->>SM: Safe ActuatorCommand
        SM->>SD: apply(Safe ActuatorCommand)
    else confidence < threshold
        SM->>SM: setState(SEARCHING)
    end

    opt Manual mode
        User->>SM: enterManual()
        SM->>SM: setState(MANUAL)
        User->>SM: setManualSetpoint(tilt, pan)
        SM->>K: onSetpoint(PlatformSetpoint)
        K-->>SM: ActuatorCommand
        SM->>AM: onCommand(ActuatorCommand)
        AM-->>SM: Safe ActuatorCommand
        SM->>SD: apply(Safe ActuatorCommand)
        User->>SM: exitManual()
        SM->>SM: setState(SEARCHING)
    end

    opt Stop system
        User->>SM: stop()
        SM->>SM: setState(STOPPING)
        SM->>Cam: stop()
        SM->>SD: stop()
        SM->>SM: setState(IDLE)
    end
````

---

## Circuit Diagram

<p align="center">
  <img src="diagrams/circuit_diagram.png" alt="Circuit Diagram" width="900">
</p>

The hardware setup connects:

* camera through a libcamera-compatible interface
* PCA9685 PWM driver over I2C
* three servo motors
* Raspberry Pi acting as the central controller

The PCA9685 generates stable PWM signals for the servos, while I2C provides communication between the Raspberry Pi and the actuator driver layer.

---

## Repository Structure

```text
Solar-Stewart-Tracker/
├── .github/
├── diagrams/
├── docs/
├── include/
│   ├── actuators/
│   ├── app/
│   ├── common/
│   ├── control/
│   ├── hal/
│   ├── sensors/
│   ├── system/
│   ├── ui/
│   └── vision/
├── media/
├── scripts/
├── src/
│   ├── actuators/
│   ├── app/
│   ├── common/
│   ├── control/
│   ├── hal/
│   ├── qt/
│   ├── sensors/
│   ├── system/
│   ├── ui/
│   └── vision/
├── tests/
├── CMakeLists.txt
├── Doxyfile
├── CONTRIBUTING.md
└── LICENSE
```

---

## Bill of Materials

### Controller

| Component            | Quantity | Cost (£) |
| -------------------- | -------: | -------: |
| Raspberry Pi 5 (8GB) |        1 |    80.00 |

### Sensors and Vision

| Component            | Quantity | Cost (£) |
| -------------------- | -------: | -------: |
| IMX219 Camera Module |        1 |    25.00 |

### Actuation and Drive

| Component                                 | Quantity | Cost (£) |
| ----------------------------------------- | -------: | -------: |
| PCA9685 PWM Driver                        |        1 |    12.00 |
| High-Torque Servo (RDS3230 or equivalent) |        3 |    45.00 |
| External 5–6V High-Current Supply         |        1 |    20.00 |

### Mechanical and Supporting Components

| Component                           | Quantity | Cost (£) |
| ----------------------------------- | -------: | -------: |
| Breadboard and Wiring Set           |        1 |    10.00 |
| Structural Frame / 3D Printed Parts |        1 |    15.00 |
| Fasteners and Mounts                | Assorted |    10.00 |

### Grand Total

**£217.00**

---

## Dependencies

### Mandatory

* **CMake 3.16 or newer**
* **C++17 compiler**

  * GCC
  * Clang
  * MSVC

### Optional

* **libcamera**
  Enables the Raspberry Pi camera backend.

* **Qt5 Widgets / Charts**
  Enables the optional Qt GUI target.

* **OpenCV**
  Enables viewer support in the CLI path and vision-related functionality when available.

### Linux packages

Minimal build tools:

```bash
sudo apt update
sudo apt install -y build-essential cmake git pkg-config
```

Optional packages:

```bash
sudo apt install -y libcamera-dev
sudo apt install -y qtbase5-dev qtcharts5-dev qt5-qmake
sudo apt install -y libopencv-dev
sudo apt install -y doxygen graphviz
```

---

## Cloning

Clone the repository:

```bash
git clone https://github.com/Real-Time-Stewart-Solar-Tracker/Solar-Stewart-Tracker.git
cd Solar-Stewart-Tracker
```

---

## Building

### Linux / Raspberry Pi OS

Configure:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
```

Build:

```bash
cmake --build build -j
```

### Windows

Configure:

```powershell
cmake -S . -B build
```

Build:

```powershell
cmake --build build --config Release
```

### Optional: disable OpenCV auto-detection

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DSOLAR_TRY_OPENCV=OFF
```

---

## Running

### Core CLI application

Linux:

```bash
./build/solar_tracker
```

Typical Windows location:

```powershell
build\Release\solar_tracker.exe
```

### Software-only mode

The software-only path uses a simulated camera backend and a non-hardware actuator path for development and testing without the physical platform.

### Optional Qt GUI application

Built only when Qt5 is found:

```bash
./build/solar_tracker_qt
```

Typical Windows location:

```powershell
build\Release\solar_tracker_qt.exe
```

### Hardware mode

Hardware execution requires:

* libcamera support
* I2C enabled on the host
* PCA9685 connected correctly
* servo power and wiring connected correctly

The system will enter **FAULT** if required hardware is unavailable or startup fails.

### Manual hardware smoke test

```bash
./build/tests/servo_manual_smoketest
```

---

## Running Software Tests

This project integrates tests with **CTest**.

### Linux

```bash
ctest --test-dir build --output-on-failure
```

### Windows

```powershell
ctest --test-dir build -C Release --output-on-failure
```

### Included automated test areas

* SunTracker
* Controller
* ActuatorManager
* ThreadSafeQueue
* Kinematics3RRS
* LatencyMonitor
* SystemManager state machine
* PCA9685
* ServoDriver

### Optional hardware-oriented integration tests

Where supported by platform and dependencies, the repository also contains hardware-related tests for:

* `libcamera`
* Linux I2C / PCA9685

---

## Realtime Evidence

The measured software-side userspace pipeline is:

**Camera -> SunTracker -> Controller -> Kinematics3RRS -> ActuatorManager -> ServoDriver**

All measurements are taken using monotonic timestamps recorded inside the software pipeline.

### Latency Results

| Metric      | Average (ms) | Minimum (ms) | Maximum (ms) | Jitter (ms) |
| ----------- | -----------: | -----------: | -----------: | ----------: |
| **L_total** |    **0.800** |        0.636 |        3.806 |       3.171 |
| L_vision    |        0.795 |        0.631 |        3.803 |           — |
| L_control   |        0.004 |        0.001 |        1.252 |           — |
| L_actuation |        0.001 |       0.0006 |        0.005 |           — |

Interpretation:

* average end-to-end software latency is below **1 ms**
* worst-case measured software latency is below **4 ms**
* observed jitter is below **4 ms**
* measurements represent the **userspace software path only**, not hard real-time guarantees or full mechanical settling time

---

## Documentation

Project documentation is stored in `docs/` and includes:

* `docs/BOM.md`
* `docs/build_and_run.md`
* `docs/DEPENDENCIES.md`
* `docs/REPRODUCIBILITY.md`
* `docs/requirements.md`
* `docs/solid_justification.md`
* `docs/state_machine.md`
* `docs/system_architecture.md`
* `docs/testing.md`
* `docs/user_stories_use_cases.md`
* `docs/latency_measurement.md`
* `docs/realtime_analysis.md`

### Generate Doxygen documentation

```bash
doxygen Doxyfile
```

Open locally at:

```text
docs/html/index.html
```

### Online Doxygen documentation

Full generated documentation is available at:

**[https://fadihalteh.github.io/sun-tracker/](https://fadihalteh.github.io/sun-tracker/)**

The documentation is automatically built and deployed using GitHub Actions on each push.

---

## Authors and Contributions

### Jichao Wang (3137140W)

Developed the 3RRS inverse kinematics model and core application setup, including configuration, factory creation, and application entry structure. Responsible for translating platform setpoints into actuator-space commands.

### Fadi Halteh (3127931H)

Designed and implemented the event-driven system architecture, including the SystemManager orchestration, state machine, and thread-safe queue pipeline. Responsible for enforcing realtime design principles such as blocking I/O wake-up behaviour, bounded queues, and overall system integration.

### Ziming Yan (2429452Y)

Developed the vision subsystem and user interface components, including the SunTracker detection pipeline and Qt-based control panel. Integrated visual feedback, overlays, and logging into the runtime system.

### Tareq A M Almzanin (3139787A)

Implemented the control layer translating vision estimates into platform motion, including closed-loop control logic and manual override behaviour. Contributed to the definition of shared data types and system-level interaction with the control path.

### Zhenyu Zhu (3099498Z)

Implemented the low-level actuator interface including PCA9685 integration and servo control, along with latency measurement instrumentation. Responsible for hardware abstraction and timing analysis across the system pipeline.

Source: project members document.

---

## Acknowledgements

We would like to thank:

* **Dr. Bernd Porr** for guidance in realtime embedded systems and architecture design
* **Dr. Chongfeng Wei** for software engineering support and project supervision
* the **University of Glasgow**
* the laboratory, workshop, and technical support staff involved in supporting the project

Their teaching, feedback, and infrastructure helped shape both the realtime architecture and the software-engineering process behind this repository.

---

## License

This project is released under the license included in this repository:

```text
LICENSE
```

Please also credit any external libraries, frameworks, or reused components according to their original licenses.

---

## Future Work

Planned or natural next extensions include:

* hardware-validated closed-loop sun tracking on the full physical platform
* improved camera backend integration on Raspberry Pi
* stronger manual and GUI operating modes
* richer telemetry and live plotting
* enhanced fault handling and recovery strategies
* more hardware-backed integration testing
* more polished project media, demo video, and public-facing documentation

---

## Last Updated

March 2026

```

