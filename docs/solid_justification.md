# SOLID Justification

This document describes the class structure of the solar-tracking system and explains how the main class boundaries support realtime behaviour, hardware isolation, testability, and maintenance.

The core runtime pipeline is:

**ICamera → SunTracker → Controller → ManualImuCoordinator → Kinematics3RRS → ActuatorManager → ServoDriver**

`SystemManager` orchestrates that pipeline at runtime. `BackendCoordinator` owns optional hardware backend lifecycle. `GuiManualDispatcher` provides the dedicated event-driven thread for GUI manual mode. `SystemFactory` assembles the runtime graph. `LinuxEventLoop` and the Qt GUI provide application-level control around the processing core.

The system is intentionally structured as a staged pipeline rather than a single tracker class. That separation keeps acquisition, estimation, control, policy coordination, mechanism mapping, safety conditioning, hardware output, and application control distinct. For this system, that improves traceability, simplifies testing, and reduces the risk of mixing timing-sensitive logic with platform-specific code.

---

## Design context

The class structure is shaped by four practical requirements.

First, hardware-facing code must remain isolated from pure logic. Camera backends, I2C access, GPIO-backed manual input, IMU publishing, and PWM output are platform-specific concerns. Vision, control, mapping, and kinematics should not depend directly on those details.

Second, the runtime must preserve an event-driven flow. Sensor and backend events enter the pipeline, are transformed by dedicated stages, and progress toward output through typed interfaces. Clear stage boundaries make that flow explicit.

Third, the system must support development and testing without requiring the full physical platform at all times. Narrow interfaces and focused classes allow software-only tests, simulated inputs, and hardware-smoke checks to coexist.

Fourth, the runtime must remain maintainable as policies, backends, and hardware details change. Each class therefore has a narrow responsibility and an explicit role in the overall data path.

---

## `ICamera`

### Purpose

`ICamera` defines the contract for frame-producing backends.

The rest of the runtime does not need to know whether frames come from a Raspberry Pi camera backend or a software simulator. It only needs a typed frame source with a callback registration mechanism.

### SOLID role

**Single Responsibility Principle**

`ICamera` defines one architectural contract: frame delivery. It does not acquire frames itself, perform tracking, or manage control behaviour.

**Interface Segregation Principle**

The interface remains narrow. Camera consumers depend only on frame-source operations and do not need backend-specific configuration or display logic.

**Dependency Inversion Principle**

Higher-level runtime code depends on `ICamera`, not on concrete camera implementations. That keeps backend choice separate from pipeline logic.

### Boundary value

This boundary prevents camera backend details from leaking into orchestration, vision, or control code. It also allows different frame sources to use the same downstream processing path.

---

## `LibcameraPublisher`

### Purpose

`LibcameraPublisher` provides the Raspberry Pi camera backend behind the `ICamera` contract.

Its job is to acquire frames from the real camera path and publish them into the runtime pipeline. It does not participate in tracking, control, or actuation policy.

### SOLID role

**Single Responsibility Principle**

Its responsibility is camera acquisition for the libcamera path.

**Liskov Substitution Principle**

It can be used anywhere an `ICamera` is required.

**Dependency Inversion Principle**

The rest of the system remains dependent on the camera abstraction while this class absorbs the backend-specific API.

### Boundary value

This class isolates libcamera lifecycle and integration details from the rest of the runtime. That keeps the pipeline independent of camera-stack implementation details.

---

## `SimulatedPublisher`

### Purpose

`SimulatedPublisher` provides a software-only frame source through the same `ICamera` contract.

It supports development, testing, and runtime execution when the real camera path is unavailable or unnecessary.

### SOLID role

**Single Responsibility Principle**

Its responsibility is synthetic frame generation and publication.

**Liskov Substitution Principle**

It can replace `LibcameraPublisher` anywhere `ICamera` is consumed.

**Open/Closed Principle**

Additional camera implementations can be introduced without redesigning the consumers of `ICamera`.

### Boundary value

Simulation remains a first-class backend rather than an internal conditional path buried in runtime orchestration.

---

## `SunTracker`

### Purpose

`SunTracker` is the vision stage. It converts a `FrameEvent` into a `SunEstimate`.

It validates frame layout assumptions, interprets image data, detects the target, and emits a typed estimate for downstream control.

### SOLID role

**Single Responsibility Principle**

Its reason to change is the target-detection method.

**Open/Closed Principle**

Tracking internals can evolve while preserving the `SunEstimate` contract used downstream.

**Encapsulation**

Frame validation and image-processing details remain contained within the tracker.

### Boundary value

This class separates image interpretation from runtime orchestration, control, and hardware output. That keeps vision faults and control faults easier to localise.

---

## `Controller`

### Purpose

`Controller` converts a tracking estimate into a platform setpoint.

It handles confidence gating, deadband behaviour, and command generation in platform coordinates.

### SOLID role

**Single Responsibility Principle**

Its reason to change is the control-law policy from estimate space to platform-command space.

**Open/Closed Principle**

Gains, deadband policy, saturation, and confidence handling can change without restructuring downstream stages.

**Dependency Inversion Principle**

The controller depends on typed estimate data rather than on camera backends or hardware output details.

### Boundary value

This stage separates control policy from both vision and mechanism mapping. That preserves a clear distinction between “what the platform should do” and “how the mechanism achieves it”.

---

## `ManualImuCoordinator`

### Purpose

`ManualImuCoordinator` owns command-policy coordination for manual control and IMU-based correction.

Its responsibilities include:

- mapping manual potentiometer samples to platform setpoints
- selecting whether potentiometer or GUI input owns manual commands
- storing the latest IMU sample and tilt estimate
- applying optional IMU correction to controller setpoints

### SOLID role

**Single Responsibility Principle**

Its responsibility is command-policy coordination for manual ownership and tilt feedback.

**Open/Closed Principle**

Manual-input ownership rules and IMU-correction rules can evolve within this policy layer without restructuring runtime orchestration.

**Dependency Inversion Principle**

It depends on dedicated helpers such as `ManualInputMapper`, `ImuTiltEstimator`, and `ImuFeedbackMapper` rather than embedding those algorithms inline.

### Boundary value

This class keeps manual-command policy and IMU-correction policy out of `SystemManager`, reducing orchestration complexity and preserving focused runtime responsibilities.

---

## `ManualInputMapper`

### Purpose

`ManualInputMapper` converts manual input values into platform commands.

This is pure mapping logic and does not belong in the ADC/input backend or in runtime orchestration.

### SOLID role

**Single Responsibility Principle**

Its reason to change is the mapping from manual input values to commanded motion.

**Dependency Inversion Principle**

Higher-level policy code depends on a dedicated mapper rather than repeating scaling logic inline.

### Boundary value

This class keeps command semantics separate from hardware acquisition details.

---

## `ImuTiltEstimator`

### Purpose

`ImuTiltEstimator` converts reduced IMU acceleration data into a tilt estimate.

It is a pure estimation component.

### SOLID role

**Single Responsibility Principle**

Its reason to change is the tilt-estimation method.

**Dependency Inversion Principle**

Policy code depends on a dedicated estimator rather than embedding estimation logic into runtime control code.

### Boundary value

This class separates sensor transport from signal interpretation.

---

## `ImuFeedbackMapper`

### Purpose

`ImuFeedbackMapper` converts measured tilt into a bounded correction term.

It is a control-support policy component.

### SOLID role

**Single Responsibility Principle**

Its reason to change is the correction policy: gain, deadband, and output limits.

**Open/Closed Principle**

Correction behaviour can be refined without redesigning orchestration, backend startup, or hardware output code.

### Boundary value

This class makes tilt-correction policy explicit and testable.

---

## `Kinematics3RRS`

### Purpose

`Kinematics3RRS` transforms a platform setpoint into actuator commands for the 3-RRS mechanism.

It is mechanism-specific mapping, separate from both control-law policy and hardware output.

### SOLID role

**Single Responsibility Principle**

Its responsibility is mechanism mapping, including geometry, calibration, and inverse-kinematics behaviour.

**Open/Closed Principle**

Mechanism details can change here while preserving the upstream platform-setpoint contract.

**Encapsulation**

Geometry and calibration parameters remain contained in a dedicated configuration structure.

### Boundary value

This class preserves the physical meaning of the intermediate platform setpoint and keeps the mechanism model explicit rather than implicit.

---

## `ActuatorManager`

### Purpose

`ActuatorManager` is the actuator safety-conditioning stage.

It clamps actuator outputs and applies slew-rate limiting before commands reach the servo driver.

### SOLID role

**Single Responsibility Principle**

Its responsibility is actuator-command conditioning.

**Open/Closed Principle**

Safety policy can change without redesigning low-level hardware access.

**Dependency Inversion Principle**

Downstream hardware code receives already-conditioned commands rather than raw control outputs.

### Boundary value

This class keeps safety policy separate from both mechanism mapping and final hardware output.

---

## `ServoDriver`

### Purpose

`ServoDriver` is the final servo-output stage.

Its responsibilities are:

- degree-to-pulse conversion
- PCA9685 access policy
- application of already-conditioned commands
- startup and stop parking behaviour

It does not own control policy, kinematics, or queueing.

### SOLID role

**Single Responsibility Principle**

Its responsibility is final servo output handling.

**Open/Closed Principle**

Calibration, startup policy, and parking behaviour can change without restructuring upstream control or kinematics code.

**Dependency Inversion Principle**

The rest of the pipeline operates in actuator-command units without depending directly on PWM register access.

### Boundary value

This class keeps electrical and channel-mapping behaviour separate from motion semantics and safety policy.

---

## `PCA9685`

### Purpose

`PCA9685` encapsulates the low-level PWM chip behaviour.

Register-level I2C interaction is isolated here rather than spread through higher-level output code.

### SOLID role

**Single Responsibility Principle**

Its responsibility is PCA9685 chip access.

**Dependency Inversion Principle**

Higher-level output stages use a dedicated chip wrapper rather than embedding low-level register access directly.

### Boundary value

This class prevents low-level bus and register operations from leaking upward into servo policy or control code.

---

## `BackendCoordinator`

### Purpose

`BackendCoordinator` owns the lifecycle of the two optional hardware backends:
the ADS1115 manual potentiometer input and the MPU-6050/ICM-20600 IMU.

This class was extracted from `SystemManager` to give backend resource ownership
an explicit, single-responsibility boundary. `SystemManager` no longer needs to
know about I2C device construction, backend start/stop sequencing, or the
`ImuCallbackForwarder` adaptor — those details are encapsulated here.

### SOLID role

**Single Responsibility Principle** — Responsibility is hardware backend lifecycle:
I2C device construction, start/stop sequencing, and sample delivery via callbacks.

**Interface Segregation Principle** — `SystemManager` depends only on
`BackendCoordinator::start()` and `stop()`. It has no visibility into which
backends are active or how their callbacks are routed.

### Boundary value

Separating backend lifecycle from pipeline orchestration means `SystemManager`
can be tested and reasoned about independently of hardware availability.

---

## `GuiManualDispatcher`

### Purpose

`GuiManualDispatcher` provides the dedicated event-driven thread for GUI manual
mode. When `setSetpoint()` is called (from the Qt UI thread), the setpoint is
pushed into a bounded freshest-data queue. The dispatcher's worker thread blocks
on that queue and wakes immediately, building a platform setpoint via
`ManualImuCoordinator` and dispatching directly to `Kinematics3RRS`.

This class was introduced to remove the asymmetry where GUI manual mode depended
on camera-frame wakeups while potentiometer manual mode was directly event-driven.
Both manual paths now have structurally identical, independently-timed event paths.

### SOLID role

**Single Responsibility Principle** — Responsibility is GUI setpoint delivery:
one queue, one thread, one dispatch path.

**Open/Closed Principle** — The dispatcher works with any `ManualImuCoordinator`
and any `Kinematics3RRS` instance without modification.

**Interface Segregation Principle** — `SystemManager` depends on only
`setSetpoint()`, `start()`, and `stop()`. The internal queue and thread are
fully encapsulated.

### Boundary value

`setManualSetpoint()` on `SystemManager` is now a one-line forwarding call.
The timing, queuing, and dispatch logic are entirely contained in this class.

---

## `SystemManager`

### Purpose

`SystemManager` is a thin runtime orchestrator. Its responsibilities are:
- composing pipeline stages
- owning two worker threads and two inter-thread queues
- managing the runtime state machine (IDLE / STARTUP / SEARCHING / TRACKING /
  MANUAL / FAULT / STOPPING)
- lifecycle management (start / stop)
- observer registration forwarding

Backend lifecycle is delegated to `BackendCoordinator`. GUI manual dispatch is
delegated to `GuiManualDispatcher`. Manual/IMU policy is delegated to
`ManualImuCoordinator`. `SystemManager` itself is approximately 380 lines.

### SOLID role

**Single Responsibility Principle** — Responsibility is pipeline orchestration
and runtime state coordination. It does not implement vision, control law,
kinematics, safety shaping, hardware access, or GUI manual timing.

**Dependency Inversion Principle** — Depends on `ICamera` (not a concrete
backend), `ManualImuCoordinator` (not manual hardware), and
`GuiManualDispatcher` (not Qt directly).

**Encapsulation** — Exposes explicit observer registration and control methods.
Internal queue and thread management is private.

### Honest assessment

`controlLoop_()` is now 10 lines with a single responsibility: drain camera
frames and forward them to the automatic processing path. The manual branches
have been fully removed. The class is substantially smaller than its previous
form and its responsibilities are clearly bounded.

---

## `ThreadSafeQueue`

### Purpose

`ThreadSafeQueue` provides blocking handoff between producer and consumer stages.

Its architectural value is not only storage. It also defines blocking wait behaviour and bounded freshest-data queue policy for the runtime pipeline.

### SOLID role

**Single Responsibility Principle**

Its responsibility is inter-thread queue handoff.

**Open/Closed Principle**

Queue policy remains encapsulated in one place instead of being reimplemented ad hoc across the runtime.

**Safe data management**

It uses STL-managed storage, explicit stop/reset behaviour, blocking waits, and bounded latest-only insertion.

### Boundary value

This class provides deterministic handoff points between worker threads and avoids shared-state polling designs.

---

## `LinuxEventLoop`

### Purpose

`LinuxEventLoop` isolates headless application-shell events from the realtime processing pipeline.

It keeps process-level input handling and shutdown behaviour outside the tracking stages.

### SOLID role

**Single Responsibility Principle**

Its responsibility is the headless application event loop.

### Boundary value

This class keeps shell-level process control separate from the tracking runtime.

---

## `SystemFactory`

### Purpose

`SystemFactory` is the composition root.

It selects and constructs the runtime graph and keeps assembly logic out of `main()`.

### SOLID role

**Single Responsibility Principle**

Its responsibility is runtime assembly.

**Dependency Inversion Principle**

It centralises concrete-type selection while allowing the running system to depend on narrower interfaces and stage contracts.

### Boundary value

This class keeps backend selection and object assembly separate from runtime execution.

---

## Callback structure

Callbacks are the primary inter-stage event mechanism in the processing pipeline.

That choice fits this runtime because sensor and backend events arrive asynchronously and must be pushed forward through the staged pipeline. Callback boundaries make event timing explicit, keep freshness aligned with stage handoff, and avoid pull-style designs in which downstream code repeatedly queries upstream state.

In this system, callback-based stage interfaces support:

- explicit event timing
- stage-local ownership of processing
- reduced coupling between pipeline stages
- direct integration with blocking/event-driven input paths
- clearer test boundaries between producers and consumers

---

## Output-side command flow

The output side follows a staged command progression.

- `Controller` produces platform setpoints
- `Kinematics3RRS` produces actuator commands
- `ActuatorManager` conditions those commands
- `ServoDriver` applies final hardware-oriented output behaviour

Each stage hands forward a more concrete representation of the command while preserving clear ownership of policy, mechanism mapping, safety conditioning, and hardware output.

---

## Architectural summary

The class structure separates the runtime into distinct layers:

- acquisition boundary
- estimation boundary
- control boundary
- manual/IMU policy boundary
- mechanism-mapping boundary
- safety-conditioning boundary
- final hardware-output boundary
- orchestration boundary
- application-shell boundary

That separation supports hardware isolation, event-driven processing, bounded inter-thread handoff, staged reasoning about faults, and focused testing of individual behaviours.

## Final judgement

The class structure is strongest where it keeps platform-specific code isolated, preserves clear event flow between stages, and separates policy, computation, orchestration, and hardware output into dedicated boundaries.

The key architectural decisions are:

- `ICamera` as the frame-source abstraction
- backend isolation in `LibcameraPublisher` and `SimulatedPublisher`
- dedicated logic stages in `SunTracker`, `Controller`, `ManualInputMapper`, `ImuTiltEstimator`, and `ImuFeedbackMapper`
- policy isolation in `ManualImuCoordinator`
- mechanism isolation in `Kinematics3RRS`
- safety conditioning in `ActuatorManager`
- final hardware mapping isolation in `ServoDriver` and `PCA9685`
- orchestration in `SystemManager`
- runtime assembly in `SystemFactory`

Together, these boundaries support a maintainable staged runtime rather than a tightly coupled monolithic tracker implementation.