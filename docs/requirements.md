# Requirements

## Purpose

This document defines the requirements implemented in the  repository . All requirements are derived directly from the  source tree, `CMakeLists.txt`, `src/app/AppConfig.cpp`, and the hardware defaults in `src/app/AppConfig.hpp`.

---

## 1. Platform Requirements

| Requirement | Source |
|---|---|
| Linux userspace execution | `LinuxEventLoop`, `signalfd`, `timerfd`, `eventfd`, `poll()` |
| C++17 standard | `CMakeLists.txt`: `CMAKE_CXX_STANDARD 17` |
| CMake 3.16 or newer | `CMakeLists.txt`: `cmake_minimum_required(VERSION 3.16)` |
| Raspberry Pi (arm64) target for hardware mode | Pi hardware workflow / scripts |
| I2C bus 1 available (`/dev/i2c-1`) | `AppConfig.cpp`: hardware defaults use `i2c_bus = 1` |
| GPIO chip 0 available | `AppConfig.cpp`: `gpio_chip_index = 0` for ADS1115 and MPU6050 |
| `libgpiod` available | build / workflow dependency |
| Optional: Qt5 Widgets + Charts | `src/qt/CMakeLists.txt` |
| Optional: OpenCV + libcamera | top-level CMake option path |

---

## 2. Functional Requirements

### FR1 — Frame acquisition via abstract camera interface

The system shall acquire frames through `ICamera`, a pure virtual interface defining `registerFrameCallback()`, `start()`, `stop()`, and `isRunning()`.

Two concrete implementations shall be provided:

- `SimulatedPublisher` — always available; paces frames via `timerfd`-based `poll()`
- `LibcameraPublisher` — available when both libcamera and OpenCV are found at configure time

The active camera backend is selected by `SystemFactory`.

### FR2 — Bright-target detection and centroid estimation

`SunTracker` shall process one `FrameEvent` and produce one `SunEstimate` containing:

- weighted centroid position (`cx`, `cy`) in pixels
- confidence value in range `[0,1]`

Detection shall use a configurable brightness threshold and minimum bright-pixel count. Supported pixel formats are `Gray8`, `RGB888`, and `BGR888`, with padded-stride frames handled correctly.

### FR3 — Image-space error to platform setpoint conversion

`Controller` shall convert a `SunEstimate` into a `PlatformSetpoint` (tilt and pan in radians) using proportional control with configurable gains, deadband, and output clamps. Estimates below the configured confidence threshold suppress motion.

### FR4 — Platform setpoint to actuator command via 3-RRS kinematics

`Kinematics3RRS` shall map a `PlatformSetpoint` to an `ActuatorCommand` containing three actuator targets in degrees.

If the inverse-kinematics solution is geometrically invalid, the system shall either fall back to the last valid command or surface an invalid configuration state that triggers FAULT handling.

### FR5 — Per-channel output safety conditioning

`ActuatorManager` shall apply per-channel clamping and optional slew/rate limiting before hardware output.

### FR6 — Servo hardware output via PCA9685

`ServoDriver` shall output conditioned actuator commands through the PCA9685 PWM controller and support startup policies including `LogOnly`, `PreferHardware`, and `RequireHardware`.

### FR7 — Manual control via potentiometer input

`ADS1115ManualInput` shall provide a manual-input path using ADS1115 plus ALERT/RDY GPIO wakeup.

`ManualInputMapper` shall convert raw voltages to bounded setpoints with deadband and optional inversion.

The potentiometer callback shall update the latest manual sample, but downstream manual actuation shall be owned by the control thread rather than driven directly from the callback.

### FR8 — IMU tilt measurement and optional correction

`Mpu6050Publisher` shall provide an IMU backend using I2C and a GPIO data-ready interrupt.

`ImuTiltEstimator` and `ImuFeedbackMapper` shall compute and optionally apply correction offsets.

Modes include:

- Disabled
- Shadow
- Live

Failure handling depends on mode.

### FR9 — Headless CLI runtime

`LinuxEventLoop` shall provide a runtime using blocking `poll()` over:

- `signalfd`
- `timerfd`
- stdin

CLI commands include:

- `manual`
- `auto`
- `set <tilt> <pan>`
- `threshold <value>`
- `quit`
- `help`

### FR10 — Qt GUI runtime

`solar_tracker_qt` shall provide a GUI with:

- control buttons
- camera preview
- status display
- charts for actuator commands, ADC, and IMU
- manual sliders for GUI-set manual control

Qt timers are used for GUI refresh, not as the timing source for control.

GUI manual mode shall update the stored manual target continuously while the slider is dragged. Continuous manual actuation shall still be performed by the control thread.

### FR11 — Explicit runtime state machine

The system shall implement an explicit state machine with:

- IDLE
- STARTUP
- NEUTRAL
- SEARCHING
- TRACKING
- MANUAL
- STOPPING
- FAULT

Transitions shall be explicit and observable.

### FR12 — Per-frame latency measurement

`LatencyMonitor` shall record timestamps at key pipeline stages and compute per-frame latency metrics. Summary statistics shall be printed on shutdown. CSV export shall be supported via environment configuration.

### FR13 — Software test execution

The build system shall support a software test suite runnable via CTest without requiring physical hardware.

Test binaries shall be executable directly. Additional command-line interfaces may exist but are not assumed to be uniform across all tests.

### FR14 — Hardware smoke test execution

The build system shall support hardware-adjacent tests via `-DSOLAR_ENABLE_HW_TESTS=ON`, runnable on a Raspberry Pi.

Hardware-dependent tests shall support safe skipping when hardware is unavailable.

---

## 3. Architectural Requirements

### AR1 — Forward-only event-driven pipeline

The automatic processing path shall be:

`ICamera → SunTracker → Controller → ManualImuCoordinator → Kinematics3RRS → ActuatorManager → ServoDriver`

No stage shall call back into an earlier stage.

Stages may own worker threads where required for blocking I/O or event-driven execution. Threads shall have clear ownership, lifecycle, and no polling or sleep-based timing.

### AR2 — Blocking waits at all thread boundaries

All threads shall block on appropriate primitives until work is available:

| Thread | Mechanism |
|---|---|
| Control | `condition_variable` via queue |
| Actuator | `condition_variable` via queue |
| Event loop | `poll()` |
| Sensors | GPIO edge events |
| SimulatedPublisher | `timerfd` + `poll()` |

No polling loops or busy-waiting are permitted. Execution is driven by blocking I/O and event-triggered wake-ups.

### AR3 — Bounded freshest-data queues

Inter-thread communication shall use bounded queues with latest-wins behaviour:

- frame queue capacity: 2
- command queue capacity: 8

Oldest data is discarded when full.

### AR4 — Graceful optional feature degradation

Optional components shall not break the build:

- libcamera absent → fallback
- OpenCV absent → disable camera backend
- Qt absent → GUI skipped
- IMU failure handled by mode

### AR5 — Camera and backend substitution

Camera backend shall be substitutable via `ICamera` without modifying downstream stages.

### AR6 — Composition root isolation

`SystemFactory` shall act as the primary top-level assembly point for the application.

Where practical, major dependencies shall be injected into `SystemManager`. Internal runtime-owned support components may still be constructed where required for lifecycle control or startup/teardown coordination.

### AR7 — Centralised state control

All state transitions shall be controlled by `SystemManager` and observable.

### AR8 — Manual-mode timing ownership

Manual GUI and potentiometer callbacks may update desired manual state, but only the control thread shall turn that manual state into downstream `Kinematics3RRS` work.

### AR9 — GUI manual stability policy

GUI manual mode shall not apply live IMU correction by default. The operator-selected GUI target remains continuous and stable, while IMU-assisted correction remains available to other modes according to policy.

---

## 4. Non-Functional Requirements

### NFR1 — End-to-end latency

Measured runtime data demonstrates that total processing latency remains within one frame period at 30 Hz under tested conditions.

### NFR2 — Latency distribution

Vision processing is expected to dominate latency, with control and actuation remaining comparatively small.

### NFR3 — Reproducible build

The system shall build reproducibly using declared dependencies and documented steps.

---

## 5. Safety Requirements

### SR1 — Park on start and stop

Servos shall move to a safe park position during startup and shutdown.

### SR2 — Safe shutdown

Shutdown shall be safe from all states and complete all cleanup steps.

### SR3 — Kinematics fault handling

Invalid kinematics shall prevent actuator output and trigger FAULT.

### SR4 — Output safety enforcement

All actuator outputs shall be clamped and validated before reaching hardware.
