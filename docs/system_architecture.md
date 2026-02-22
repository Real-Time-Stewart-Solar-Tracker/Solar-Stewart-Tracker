# System Architecture — Solar Stewart Tracker

This system is implemented as an **event-driven, multi-threaded pipeline** on Linux
(Raspberry Pi) and Windows (desktop simulation). Threads **block while waiting for
events** and wake only when new data arrives. The realtime path uses **no polling
loops** and **no sleep-based timing**.

---

## Links to requirements and use-cases
- System requirements: `docs/requirements.md`
- User stories / use-cases: `docs/user_stories_use_cases.md`
- Realtime analysis: `docs/realtime_analysis.md`

---

## High-level concept

The software is structured as a **three-stage pipeline**:

1. **Acquire** (camera backend)
2. **Compute** (vision + control + kinematics)
3. **Actuate** (safety + servo output)

Each stage is separated by a **bounded blocking queue** to:
- prevent unbounded backlog,
- bound latency/jitter,
- ensure the controller uses **fresh** sensor data.

Queues are configured with a **freshest-data policy**:
- When full, the oldest item is dropped and the newest item is kept.

---

## Threads and wake-up events

| Thread | Purpose | Wakes up when (event) | Mechanism |
|---|---|---|---|
| **T1 Camera (backend-owned)** | Acquire frames | New frame available | Camera implementation invokes frame callback (event-driven) |
| **T2 Control (SystemManager)** | Vision + control + kinematics | New `FrameEvent` available | Blocks on `FrameQueue.wait_pop()` (condition variable) |
| **T3 Actuator (SystemManager)** | Safety + servo output | New `ActuatorCommand` available | Blocks on `CommandQueue.wait_pop()` (condition variable) |

> Notes:
> - On Raspberry Pi, **libcamera** delivers frame callbacks from its internal thread(s).
> - On desktop simulation, frames are still delivered as discrete events by the simulated camera publisher.
> - Both control and actuator stages are **blocking** and event-driven (no polling).

---

## Data flow (event payloads)

| From | To | Data payload | Transport |
|---|---|---|---|
| `ICamera` (Libcamera/Simulated) | `SystemManager::onFrame_()` | `FrameEvent` (image + `frame_id` + timestamp) | Frame callback |
| `SystemManager::onFrame_()` | **FrameQueue** | `FrameEvent` | `frame_q_.push_latest()` |
| **T2 Control** | `SunTracker` | `FrameEvent` → `SunEstimate` | Direct call within T2 |
| **T2 Control** | `Controller` | `SunEstimate` → `PlatformSetpoint` | Callback-driven inside T2 |
| **T2 Control** | `Kinematics3RRS` | `PlatformSetpoint` → `ActuatorCommand` | Callback-driven inside T2 |
| `Kinematics3RRS` | **CommandQueue** | `ActuatorCommand` | `cmd_q_.push_latest()` |
| **T3 Actuator** | `ActuatorManager` | `ActuatorCommand` → safe `ActuatorCommand` | Direct call within T3 |
| **T3 Actuator** | `ServoDriver` | safe `ActuatorCommand` | Direct call within T3 |

---

## Queue semantics (realtime behaviour)

Both queues are **bounded** to prevent pipeline backlog:

- **FrameQueue**: capacity = 1  
  - keeps only the newest `FrameEvent`
  - ensures control uses the **freshest** frame
- **CommandQueue**: capacity = 1  
  - keeps only the newest `ActuatorCommand`
  - ensures actuators receive the **latest** command

When a bounded queue is full, `push_latest()` drops the oldest element
and enqueues the newest one.

This design is appropriate for realtime control where **freshness**
matters more than processing every single frame.

---

## Component responsibilities (SOLID alignment)

- **`ICamera`**: frame acquisition interface (dependency inversion for test/simulation)
- **`SunTracker`**: vision / sun centroid + confidence estimation
- **`Controller`**: control law generating platform setpoints
- **`Kinematics3RRS`**: inverse kinematics mapping setpoint → actuator targets
- **`ActuatorManager`**: safety layer (limits, sanity checks, safe output)
- **`ServoDriver`**: actuator command application (hardware or simulated)
- **`LatencyMonitor`**: timestamps and reporting of end-to-end latency

---

## Latency and instrumentation

Timestamps are taken at key pipeline boundaries:
- capture (frame callback)
- estimate produced
- control output produced
- actuation applied

This allows reporting of:
- Capture → Estimate
- Estimate → Control
- Control → Actuation

These metrics, plus jitter, are discussed in `docs/realtime_analysis.md`.

---

## Requirement mapping (example)

| Requirement | Architecture element |
|---|---|
| FR1 Event-driven camera | Camera backend callback → `SystemManager::onFrame_()` |
| FR2 Sun detection | `SunTracker` (T2) |
| FR3 Control law | `Controller` (T2) |
| FR4 Kinematics | `Kinematics3RRS` (T2) |
| FR5 Safety | `ActuatorManager` (T3) |
| FR6 Actuation output | `ServoDriver` (T3) |
| FR7 Latency/logging | `LatencyMonitor` + logger timestamps |

---

## Diagram (PNG for marking reliability)

This PNG is the authoritative architecture diagram for assessment.

![Threaded Event Architecture](../diagrams/thread_event_architecture.png)

## UML / Architecture Diagrams

These diagrams describe the high-level, event-driven architecture of the Solar Stewart Tracker.

---

## 1) Class Diagram (High-Level)

```mermaid
classDiagram
direction LR

class SystemManager {
  +start() bool
  +stop() void
  +state() TrackerState
  +enterManual() void
  +exitManual() void
  +setManualSetpoint(tilt_rad, pan_rad) void
  +setTrackerThreshold(thr) void
  +registerFrameObserver(cb) void
  +registerEstimateObserver(cb) void
  +registerSetpointObserver(cb) void
  +registerCommandObserver(cb) void
  +registerLatencyObserver(cb) void
  -onFrame_(fe) void
  -controlLoop_() void
  -actuatorLoop_() void
  -applyNeutralOnce_() void
  -setState_(s) void
}

class ICamera {
  <<interface>>
  +registerFrameCallback(cb) void
  +start() bool
  +stop() void
  +isRunning() bool
}

class LibcameraPublisher
class SimulatedPublisher

ICamera <|.. LibcameraPublisher
ICamera <|.. SimulatedPublisher

class ThreadSafeQueue~FrameEvent~ {
  +wait_pop() optional~FrameEvent~
  +push_latest(item) bool
  +stop() void
  +reset() void
  +clear() void
}

class ThreadSafeQueue~ActuatorCommand~ {
  +wait_pop() optional~ActuatorCommand~
  +push_latest(item) bool
  +stop() void
  +reset() void
  +clear() void
}

class SunTracker {
  +registerEstimateCallback(cb) void
  +setThreshold(thr) void
  +onFrame(fe) void
}

class Controller {
  +registerSetpointCallback(cb) void
  +onEstimate(est) void
}

class Kinematics3RRS {
  +registerCommandCallback(cb) void
  +onSetpoint(sp) void
}

class ActuatorManager {
  +registerSafeCommandCallback(cb) void
  +onCommand(cmd) void
}

class ServoDriver {
  +start() bool
  +stop() void
  +apply(cmd) void
}

class LatencyMonitor {
  +onCapture(frame_id, t) void
  +onEstimate(frame_id, t) void
  +onControl(frame_id, t) void
  +onActuate(frame_id, t) void
  +printSummary() void
}

SystemManager o-- ICamera
SystemManager o-- SunTracker
SystemManager o-- Controller
SystemManager o-- Kinematics3RRS
SystemManager o-- ActuatorManager
SystemManager o-- ServoDriver
SystemManager o-- LatencyMonitor
SystemManager o-- ThreadSafeQueue~FrameEvent~
SystemManager o-- ThreadSafeQueue~ActuatorCommand~

ICamera ..> FrameEvent : emits via callback
SunTracker ..> SunEstimate
Controller ..> PlatformSetpoint
Kinematics3RRS ..> ActuatorCommand

sequenceDiagram
autonumber

participant Cam as T1 Camera (backend thread)
participant SM as SystemManager
participant FQ as FrameQueue (cap=1)
participant CT as T2 Control thread
participant ST as SunTracker
participant C as Controller
participant K as Kinematics3RRS
participant CQ as CommandQueue (cap=1)
participant AT as T3 Actuator thread
participant AM as ActuatorManager
participant SD as ServoDriver

Cam->>SM: FrameCallback(FrameEvent)
SM->>FQ: push_latest(FrameEvent)

CT->>FQ: wait_pop()
FQ-->>CT: FrameEvent
CT->>ST: onFrame(FrameEvent)
ST-->>CT: SunEstimate (via callback)
CT->>C: onEstimate(SunEstimate)
C-->>CT: PlatformSetpoint (via callback)
CT->>K: onSetpoint(PlatformSetpoint)
K-->>CT: ActuatorCommand (via callback)
CT->>CQ: push_latest(ActuatorCommand)

AT->>CQ: wait_pop()
CQ-->>AT: ActuatorCommand
AT->>AM: onCommand(ActuatorCommand)
AM-->>AT: Safe ActuatorCommand (via callback)
AT->>SD: apply(Safe ActuatorCommand)


stateDiagram-v2
[*] --> IDLE
IDLE --> STARTUP : start()
STARTUP --> NEUTRAL : subsystems ready
NEUTRAL --> SEARCHING : neutral applied
SEARCHING --> TRACKING : confidence >= threshold
TRACKING --> SEARCHING : confidence < threshold

SEARCHING --> MANUAL : enterManual()
TRACKING --> MANUAL : enterManual()
MANUAL --> SEARCHING : exitManual()

IDLE --> FAULT : start() fails
STARTUP --> FAULT : init error
SEARCHING --> FAULT : unrecoverable error
TRACKING --> FAULT : unrecoverable error
MANUAL --> FAULT : unrecoverable error

SEARCHING --> STOPPING : stop()
TRACKING --> STOPPING : stop()
MANUAL --> STOPPING : stop()
STOPPING --> IDLE : threads joined, driver stopped