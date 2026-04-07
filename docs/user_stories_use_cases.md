# User Stories and Use Cases

## Purpose

This document captures the user-facing goals supported by the  repository . All stories and use cases are grounded in the actual runtime behaviour visible in `src/app/CliController.cpp`, `src/qt/MainWindow.cpp`, and `src/system/SystemManager.hpp`.

---

## 1. User Stories

### Story 1 — Automatic tracking

As a user, I want the platform to automatically align toward the brightest light source so that the system performs autonomous solar tracking without manual intervention.

Supported by: automatic SEARCHING → TRACKING state transition on confidence threshold; continuous closed-loop updates in TRACKING state.

### Story 2 — Manual override via physical potentiometers

As a user, I want to manually position the platform using physical potentiometer inputs so that I can test, calibrate, or override the automatic mode without a computer.

Supported by: ADS1115 manual input backend; `ManualCommandSource::Pot`; MANUAL state with actuator path still active.

### Story 3 — Manual override via GUI sliders

As a user, I want to send explicit tilt and pan setpoints from the GUI so that I can position the platform precisely from the application window.

Supported by: pan and tilt `QSlider` controls in `MainWindow`; "Send manual setpoint" and "Zero" buttons; `ManualCommandSource::Gui`; `setManualSetpoint(tilt_rad, pan_rad)`.

### Story 4 — Manual override via CLI

As a developer or operator, I want to send manual setpoints and control the runtime from a terminal so that I can operate the system without the GUI.

Supported by: `CliController` commands: `manual`, `auto`, `set <tilt_rad> <pan_rad>`, `threshold <0..255>`, `quit`.

### Story 5 — Runtime state visibility

As a user, I want to see whether the system is IDLE, STARTUP, NEUTRAL, SEARCHING, TRACKING, MANUAL, STOPPING, or FAULT so that I always know the current operating mode.

Supported by: `registerStateObserver()` callback; `modeLabel_` and `statusLabel_` in `MainWindow`; CLI status output.

### Story 6 — Observe live pipeline signals

As a developer, I want to observe frame data, target estimates, platform setpoints, actuator commands, potentiometer voltages, and IMU readings in real time so that I can verify and debug pipeline behaviour.

Supported by the following registered observers in `MainWindow`:

| Observer | Data Exposed |
|---|---|
| `registerFrameObserver` | Raw camera frames → live preview |
| `registerEstimateObserver` | `SunEstimate` (position + confidence) |
| `registerSetpointObserver` | `PlatformSetpoint` (tilt, pan) |
| `registerCommandObserver` | `ActuatorCommand` (3-channel targets) → servo command plot |
| `registerManualObserver` | `ManualPotSample` (tilt voltage, pan voltage, sequence) |
| `registerImuObserver` | `ImuSample` (ax, ay, az) + computed tilt in degrees |
| `registerLatencyObserver` | Per-frame latency breakdown |

### Story 7 — Adjust tracking sensitivity at runtime

As a user, I want to adjust the brightness threshold used for target detection without restarting the application so that I can tune tracking sensitivity for changing light conditions.

Supported by: threshold `+`/`-` buttons in `MainWindow`; `threshold <0..255>` CLI command; `setTrackerThreshold()` in `SystemManager`.

### Story 8 — Switch manual input source at runtime

As a user, I want to switch between potentiometer control and GUI slider control without leaving manual mode so that I can hand off between physical and software inputs seamlessly.

Supported by: "Use Pots" / "Use GUI" buttons in `MainWindow`; `setManualCommandSource(ManualCommandSource::Pot/Gui)`.

### Story 9 — Safe shutdown

As a user, I want the platform to park safely and all backends to stop cleanly when I close the application so that the hardware is left in a known state.

Supported by: STOPPING state sequence (camera stop → thread join → neutral command → actuator stop → driver stop → IDLE); `park_on_stop = true` in default config; `signalfd` SIGINT/SIGTERM handling in `LinuxEventLoop`.

### Story 10 — Run without full hardware

As a developer or assessor, I want the system to run and process frames without a real camera or servo hardware so that software testing and non-hardware validation remain possible.

Supported by: `SimulatedPublisher` (`CameraBackend::Simulated`); `ServoDriver::StartupPolicy::LogOnly`; full software test suite in `test_core`.

---

## 2. Use Cases

### UC-1: Start automatic tracking

**Actor:** user

**Precondition:** application launched

1. `SystemManager::start()` is called
2. runtime enters STARTUP — camera validity checked, driver started, queues reset, threads started
3. runtime enters NEUTRAL — startup park applied at 41°
4. runtime enters SEARCHING — frames begin processing
5. `SunTracker` produces an estimate with confidence ≥ 0.4
6. runtime enters TRACKING — closed-loop actuator updates begin

**Postcondition:** platform continuously updates to follow the brightest target

---

### UC-2: Switch to manual mode and position the platform

**Actor:** user

**Precondition:** system in SEARCHING or TRACKING

**Path A — potentiometers (headless or GUI):**
1. user enters manual mode (`manual` CLI command or AUTO/MANUAL button)
2. runtime enters MANUAL — automatic controller disabled
3. ADS1115 delivers potentiometer samples via ALERT/RDY GPIO edge
4. `ManualImuCoordinator` maps voltages to setpoints via `ManualInputMapper`
5. setpoints pass through `Kinematics3RRS` → `ActuatorManager` → `ServoDriver`

**Path B — GUI sliders:**
1. user presses "Use GUI" button → `ManualCommandSource::Gui`
2. user adjusts pan/tilt sliders and presses "Send manual setpoint"
3. `setManualSetpoint(tilt_rad, pan_rad)` is called on `SystemManager`
4. same downstream path as Path A from step 5

**Path C — CLI:**
1. user types `set <tilt_rad> <pan_rad>`
2. `CliController` calls `setManualSetpoint()` directly

**Postcondition:** platform positioned at requested setpoint; automatic tracking suspended

---

### UC-3: Adjust tracking threshold at runtime

**Actor:** user / developer

**Precondition:** system running in any active state

1. user presses `+`/`-` threshold button in GUI, or types `threshold <value>` in CLI
2. `setTrackerThreshold(value)` updates the brightness cutoff in `SunTracker`
3. subsequent frames use the new threshold immediately

**Postcondition:** tracking sensitivity changed without restart

---

### UC-4: Observe live pipeline data

**Actor:** developer

**Precondition:** Qt GUI running; observers registered in `MainWindow`

The GUI displays the following in real time:

| Panel | Content |
|---|---|
| Live preview | Camera frames converted to Qt image |
| Mode / status labels | Current state, manual source |
| Manual input label | Tilt voltage, pan voltage, sequence counter |
| IMU label | ax, ay, az in m/s², computed tilt in degrees, valid flag |
| Servo command plot | 3-channel actuator targets over time |
| ADS1115 voltage plot | Tilt and pan potentiometer voltages over time |
| MPU6050 plot | Accelerometer axes over time |

Plot refresh rate: 33 ms timer (~30 Hz). Camera preview refresh rate: 33 ms timer.

---

### UC-5: Stop safely

**Actor:** user

**Precondition:** system in any active state

1. user presses STOP button, types `quit`/`exit`, or sends SIGINT/SIGTERM
2. runtime enters STOPPING
3. camera stops → frame queue stops and thread joins → command queue clears → neutral kinematics command issued → actuator thread stops → driver stops
4. runtime enters IDLE

**Postcondition:** platform parked at neutral position; all threads joined; hardware released

---

### UC-6: Run without hardware (simulated path)

**Actor:** developer / assessor

**Precondition:** built without libcamera/OpenCV, or libcamera unavailable at runtime

1. `defaultConfig()` selects `CameraBackend::Simulated` and `ServoDriver::StartupPolicy::LogOnly`
2. `SimulatedPublisher` generates synthetic frames at 30 Hz via `timerfd`-paced `poll()` loop
3. full pipeline runs — `SunTracker`, `Controller`, `ManualImuCoordinator`, `Kinematics3RRS`, `ActuatorManager` all execute normally
4. `ServoDriver` logs commands without writing to hardware

**Postcondition:** end-to-end software pipeline exercised without physical hardware

---

## 3. Summary

| Story | Interface |
|---|---|
| Automatic tracking | Runtime state machine |
| Manual via potentiometers | ADS1115 backend + MANUAL state |
| Manual via GUI sliders | `MainWindow` pan/tilt sliders |
| Manual via CLI | `set`, `manual`, `auto` commands |
| State visibility | State observer + GUI labels |
| Live signal observation | 7 registered observers + plots |
| Runtime threshold tuning | `+`/`-` buttons or `threshold` CLI |
| Manual source switching | "Use Pots" / "Use GUI" buttons |
| Safe shutdown | STOPPING sequence + park on stop |
| Simulated fallback | `SimulatedPublisher` + `LogOnly` servo |