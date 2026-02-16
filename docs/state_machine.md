# System State Machine

This document defines the runtime behaviour of the Solar Stewart Tracker as a state machine.
It ensures the implemented behaviour matches the documented requirements (use cases + user stories).

---

## 1. States

| State | Meaning | Outputs |
|------|---------|---------|
| IDLE | System not running | No motion |
| STARTUP | Initialisation in progress | No motion |
| NEUTRAL | Move to safe neutral pose | Command tilt=0, pan=0 (best-effort) |
| SEARCHING | Sun not confidently detected | Hold safe (controller outputs zero) |
| TRACKING | Sun detected with confidence | Normal closed-loop updates |
| MANUAL | User controls setpoint | Manual setpoint → kinematics → actuators (bounded by safety) |
| STOPPING | Shutdown in progress | Best-effort neutral then stop |
| FAULT | Failure state | Stop outputs safely |

---

## 2. Transition Rules

| From | To | Trigger |
|------|----|---------|
| IDLE | STARTUP | `SystemManager::start()` called |
| STARTUP | FAULT | Camera/driver start fails |
| STARTUP | NEUTRAL | Camera and driver successfully started |
| NEUTRAL | SEARCHING | After neutral command issued |
| SEARCHING | TRACKING | `SunEstimate.confidence >= min_confidence` |
| TRACKING | SEARCHING | `SunEstimate.confidence < min_confidence` |
| (any) | MANUAL | `enterManual()` |
| MANUAL | SEARCHING | `exitManual()` |
| (any running) | STOPPING | `stop()` called |
| STOPPING | IDLE | Camera stopped + driver stopped |
| (any) | FAULT | unrecoverable failure detected |

Reacquire behaviour is implemented as a simplified transition:
TRACKING ↔ SEARCHING based on confidence threshold.

---

## 3. Mermaid Diagram

```mermaid
stateDiagram-v2
    [*] --> IDLE

    IDLE --> STARTUP: start()
    STARTUP --> FAULT: init failure
    STARTUP --> NEUTRAL: init ok
    NEUTRAL --> SEARCHING: neutral issued

    SEARCHING --> TRACKING: confidence >= threshold
    TRACKING --> SEARCHING: confidence < threshold

    SEARCHING --> MANUAL: enterManual()
    TRACKING --> MANUAL: enterManual()
    MANUAL --> SEARCHING: exitManual()

    SEARCHING --> STOPPING: stop()
    TRACKING --> STOPPING: stop()
    MANUAL --> STOPPING: stop()
    STOPPING --> IDLE: shutdown done

    FAULT --> IDLE: reset/restart