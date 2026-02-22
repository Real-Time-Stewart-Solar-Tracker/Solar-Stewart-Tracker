# SOLID Justification (Design Rationale)

This document explains how SOLID principles were applied *intentionally* (not mechanically) in the Solar Stewart Tracker project. The goal is professional-grade structure: clear responsibilities, testability, maintainability, and safe real-time/event-driven behaviour.

## System boundary (what the system is)
An event-driven pipeline:

Camera (ICamera) → SunTracker → Controller → Kinematics3RRS → ActuatorManager → ServoDriver

`SystemManager` wires modules using callbacks. Modules are “pure logic” where possible, and hardware-specific code is isolated.

---

## ICamera (interface)
**Why it exists (SRP):** Defines a minimal contract for “a source of FrameEvent”, independent of hardware.  
**Depends on (DIP):** Nothing hardware-specific; consumers depend on the abstraction (ICamera), not concrete camera backends.  
**Does NOT do:** No libcamera calls, no frame processing, no threading policy decisions for the whole system.

**SOLID highlights:**  
- **DIP:** Higher-level code depends on `ICamera`, not libcamera.  
- **ISP:** Interface is small: start/stop/register callback/isRunning.

---

## LibcameraPublisher (ICamera implementation)
**Why it exists (SRP):** Implements camera acquisition using libcamera on Linux and emits `FrameEvent` via callback.  
**Depends on (DIP):** Depends on libcamera only inside this module; the rest of the system is unaware of libcamera.  
**Does NOT do:** No sun detection, no control law, no kinematics, no actuation.

**Professional evaluation:** libcamera is isolated because it is platform-specific (Linux-only). This prevents cross-platform build failures and keeps hardware dependencies from “leaking” upward.

---

## SimulatedPublisher (ICamera implementation)
**Why it exists (SRP):** Generates synthetic frames for development/testing when hardware is absent (Windows/CI).  
**Depends on (DIP):** Implements `ICamera`, allowing `SystemManager` and downstream processing to run unchanged.  
**Does NOT do:** No production camera I/O; no coupling to libcamera APIs.

**Professional evaluation:** Simulation is not a shortcut; it enables deterministic testing and continuous integration while keeping the production path intact.

---

## SunTracker
**Why it exists (SRP):** Converts `FrameEvent` → `SunEstimate` (centroid + confidence).  
**Depends on (DIP):** Exposes output via callback; does not depend on Controller/Kinematics.  
**Does NOT do:** Does not move hardware, does not schedule threads, does not compute actuator commands.

**Professional evaluation:** Keeping SunTracker pure makes it testable with synthetic images and avoids hardware coupling.

---

## Controller
**Why it exists (SRP):** Converts `SunEstimate` → `PlatformSetpoint` (tilt/pan) using a deterministic control law (deadband, limits, confidence gating).  
**Depends on (DIP):** Consumes `SunEstimate` only and emits setpoints via callback.  
**Does NOT do:** No kinematics math, no actuator safety, no camera acquisition.

**Professional evaluation:** Control logic is isolated so it can be tuned/validated without changing the kinematics or hardware layers.

---

## Kinematics3RRS
**Why it exists (SRP):** Converts `PlatformSetpoint` → `ActuatorCommand` using a calibrated mapping (currently a parameterised linearised model).  
**Depends on (DIP):** Receives setpoint and emits actuator command via callback; does not depend on hardware.  
**Does NOT do:** No camera processing, no control logic, no safety limiting beyond clamping to configured actuator bounds.

**Professional evaluation:** Kinematics is intentionally separated because it is geometry/calibration-specific. The interface remains stable if we later replace the linear mapping with full geometric kinematics.

---

## ActuatorManager
**Why it exists (SRP):** Safety layer: clamp + rate-limit actuator commands before hardware output.  
**Depends on (DIP):** Receives `ActuatorCommand`, emits safe `ActuatorCommand` via callback.  
**Does NOT do:** No kinematics, no control law, no hardware PWM.

**Professional evaluation:** Safety is independent because it is a cross-cutting concern and must remain correct even if control/kinematics change.

---

## ServoDriver
**Why it exists (SRP):** Hardware output boundary: takes safe targets and applies them to the actuator interface (currently logs; can be replaced with PWM/driver hardware).  
**Depends on (DIP):** Has no dependencies on upstream processing; only consumes safe commands.  
**Does NOT do:** No rate limiting, no kinematics, no decision making.

**Professional evaluation:** Separating driver allows hardware changes (pigpio/PCA9685/etc.) without affecting control logic.

---

## SystemManager
**Why it exists (SRP):** Lifecycle + wiring only. Owns modules and connects callbacks to create the pipeline.  
**Depends on (DIP):** Receives `std::unique_ptr<ICamera>` (abstraction).  
**Does NOT do:** No image processing, no control computations, no hardware logic; no “business logic” besides start/stop order.

**Professional evaluation:** This prevents a “god object”. The manager coordinates but does not implement algorithms.

---

## ThreadSafeQueue (if used)
**Why it exists (SRP):** Provides safe producer/consumer transfer between threads (blocking with condition variables).  
**Depends on (DIP):** Generic template; independent of application domain.  
**Does NOT do:** No logging policy, no ownership of system threads.

**Professional evaluation:** Separating concurrency primitives reduces risk and makes behaviour testable.

---

## Logger
**Why it exists (SRP):** Single responsibility: consistent logging API used across modules.  
**Depends on (DIP):** Modules depend on the logging abstraction rather than writing directly to stdout.  
**Does NOT do:** No control decisions; no system orchestration.

---

## Summary: why SOLID was chosen (not blindly applied)
- The project mixes **hardware**, **real-time event flow**, and **pure logic**. SOLID separation prevents hardware details from contaminating control/vision logic.
- **DIP + interfaces** enable:
  - real camera on Linux
  - simulated camera on Windows/CI
  - same pipeline, same tests
- **Callbacks** provide decoupled event flow and avoid tight coupling between pipeline stages.
- Each class has an explicit boundary, enabling focused testing and professional maintainability.