# System Architecture

This system is implemented as an event-driven, multi-threaded pipeline on Linux
(Raspberry Pi). Threads block while waiting for events and wake only when new
data arrives. No polling loops or sleep-based timing are used.

---
## Links to requirements
- System requirements: `docs/requirements.md`
- User stories/use cases: `docs/user_stories_use_cases.md`
- Realtime analysis: `docs/realtime_analysis.md`

## Threads and wake-up events

| Thread | Purpose | Wakes up when (event) | Mechanism |
|---|---|---|---|
| T1 Camera | Acquire frames | New camera frame arrives | libcamera delivers a frame callback (event-driven) |
| T2 Control | Compute control + kinematics | New sun estimate available | Blocks on SunEstimate queue (condition variable) |
| T3 Actuator | Apply safe output | New actuator command available | Blocks on ActuatorCmd queue (condition variable) |

---

## Data flow (event payloads)

| From | To | Data payload |
|---|---|---|
| CameraPublisher | SunTracker | FrameEvent (image + timestamp) |
| SunTracker | SunEstimate Queue | SunEstimate (centroid + confidence + timestamp) |
| Controller | Kinematics3RRS | PlatformSetpoint (tilt/pan) |
| Kinematics3RRS | ActuatorCmd Queue | ActuatorCommand (3 actuator targets) |

---

## Diagram (PNG for marking reliability)

This PNG is the authoritative architecture diagram for assessment.

![Threaded Event Architecture](../diagrams/thread_event_architecture.png)

---
## Requirement mapping

| Requirement | Architecture element |
|---|---|
| FR1 Event-driven camera | T1 Camera thread + CameraPublisher callback |
| FR2 Sun detection | SunTracker module |
| FR3 Control law | Controller module (T2) |
| FR4 Kinematics | Kinematics3RRS module (T2) |
| FR5 Safety | ActuatorManager (T3) |
| FR6 Logging/latency | Logger timestamps across pipeline |


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
class CameraPublisher

ICamera <|.. LibcameraPublisher
ICamera <|.. SimulatedPublisher
ICamera <|.. CameraPublisher

class SunTracker {
  +registerEstimateCallback(cb) void
  +setThreshold(thr) void
  +onFrame(frame) void
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

class UiViewer {
  +onFrame(frame) void
  +onEstimate(est) void
  +onSetpoint(sp) void
  +onCommand(cmd) void
  +onLatency(sample) void
  +tick() bool
  +threshold() int
}

SystemManager o-- ICamera
SystemManager o-- SunTracker
SystemManager o-- Controller
SystemManager o-- Kinematics3RRS
SystemManager o-- ActuatorManager
SystemManager o-- ServoDriver
SystemManager o-- LatencyMonitor
SystemManager ..> UiViewer : observers (optional)

ICamera ..> FrameEvent
SunTracker ..> SunEstimate
Controller ..> PlatformSetpoint
Kinematics3RRS ..> ActuatorCommand
