# SOLID Justification (Design Rationale)

This document explains how SOLID principles were applied intentionally in the Solar Stewart Tracker project. The aim is not to force textbook patterns everywhere, but to use SOLID where it gives clear engineering value for this specific system: hardware separation, testability, maintainability, and safe event-driven real-time behaviour.

## Why SOLID matters in this project

This project is not a single algorithm. It combines:

- hardware-dependent camera and actuator interfaces
- event-driven thread coordination
- image-processing logic
- control logic
- kinematics
- safety shaping
- platform-specific Linux code
- software-only test and simulation paths

Without clear boundaries, these concerns would quickly collapse into a tightly coupled design that is hard to test, hard to port, and hard to defend academically.

In this project, SOLID is useful because it helps achieve the following:

- keep hardware-specific code isolated
- keep pure logic testable without physical hardware
- allow simulation and hardware paths to share the same higher-level pipeline
- prevent one module from becoming a “god class”
- make safety logic explicit instead of buried inside unrelated classes
- keep the architecture aligned with an event-driven callback-based design

The system boundary is:

**Camera (`ICamera`) → SunTracker → Controller → Kinematics3RRS → ActuatorManager → ServoDriver**

`SystemManager` wires these modules together and manages lifecycle and worker threads.

---

## `ICamera` (interface)

### Why SOLID is the right choice here

The camera is one of the clearest places where SOLID is the best design choice in this project.

The rest of the system should care only about this question:

**“Can I receive frames?”**

It should not care whether frames come from:

- libcamera on Raspberry Pi
- a simulated source
- a future USB or file-backed source

If higher-level code depended directly on libcamera, then:

- testing would require hardware or Linux-specific stubs
- Windows/software-only development would become harder
- camera-specific details would leak upward into orchestration logic

So in this case, using an interface is not an academic exercise. It is the most practical way to keep the full pipeline reusable and testable.

### Why it exists (SRP)

Defines a minimal contract for “a source of `FrameEvent`”, independent of hardware.

### Depends on (DIP)

Higher-level code depends on the abstraction `ICamera`, not on a concrete backend.

### Does NOT do

- no libcamera calls
- no frame processing
- no control logic
- no global thread/orchestration policy

### SOLID highlights

- **SRP:** one responsibility — provide camera-like frame delivery contract
- **ISP:** small interface with only essential operations
- **DIP:** higher-level code depends on the camera abstraction, not Linux camera APIs

### Why this is the best choice here

Because this project must support both:

- real camera operation on Linux
- software-only or simulated execution for testing and development

That requirement makes an abstraction boundary at the camera input not just desirable, but necessary.

---

## `LibcameraPublisher` (`ICamera` implementation)

### Why SOLID is the right choice here

libcamera is Linux-specific and platform-specific. That makes it exactly the kind of dependency that should be isolated.

If libcamera code were spread across `SystemManager`, `SunTracker`, or application setup logic, then:

- the codebase would be less portable
- build logic would be harder to manage
- platform-specific failures would affect unrelated modules
- testing would become much more difficult

So in this project, putting libcamera inside one dedicated implementation is the cleanest and safest design.

### Why it exists (SRP)

Implements camera acquisition using libcamera and emits `FrameEvent` via callback.

### Depends on (DIP)

Depends on libcamera internally, but presents only the `ICamera` contract to the rest of the system.

### Does NOT do

- no sun detection
- no control law
- no kinematics
- no actuation logic

### SOLID highlights

- **SRP:** only camera acquisition and delivery
- **LSP:** can substitute for any other valid `ICamera` implementation
- **DIP:** libcamera dependency is contained inside the hardware-facing boundary

### Why this is the best choice here

Because libcamera is exactly the sort of volatile, platform-specific dependency that should not contaminate the rest of the pipeline.

---

## `SimulatedPublisher` (`ICamera` implementation)

### Why SOLID is the right choice here

Simulation is essential in this project because hardware is not always available, and much of the pipeline should still be testable and runnable without it.

If simulation were bolted awkwardly into `SystemManager` with `if hardware else simulate` logic everywhere, then:

- orchestration code would become cluttered
- test paths would diverge from production paths
- simulation would stop being a real substitute for hardware input

Making simulation another `ICamera` implementation is therefore the cleanest and most robust design.

### Why it exists (SRP)

Generates synthetic frames for development, testing, and software-only execution.

### Depends on (DIP)

Implements `ICamera`, so higher-level code can use it without any changes.

### Does NOT do

- no real camera I/O
- no libcamera dependency
- no downstream processing logic

### SOLID highlights

- **SRP:** one responsibility — provide simulated frame input
- **LSP:** can stand in for a real camera source at the interface level
- **DIP:** keeps higher-level code independent of whether input is real or simulated

### Why this is the best choice here

Because this project explicitly benefits from being able to run the same pipeline on:

- Raspberry Pi hardware
- Linux development machines
- Windows/software-only environments

That only works cleanly if simulation and hardware share the same abstraction boundary.

---

## `SunTracker`

### Why SOLID is the right choice here

Vision logic should answer one question only:

**“Where is the bright solar target, and how confident are we?”**

If `SunTracker` also handled control, actuator decisions, or hardware output, then:

- testing image logic would become harder
- algorithm tuning would be entangled with unrelated parts of the system
- failures would be harder to localise

For this project, keeping `SunTracker` as pure logic is the best design because it allows synthetic image testing and keeps vision independent from the rest of the control stack.

### Why it exists (SRP)

Converts `FrameEvent` into `SunEstimate` containing centroid and confidence.

### Depends on (DIP)

Consumes frame data and exposes output without depending on controller, kinematics, or actuators.

### Does NOT do

- no thread orchestration
- no motion commands
- no hardware output

### SOLID highlights

- **SRP:** image interpretation only
- **OCP:** internal tracking algorithm can evolve without changing controller or hardware layers
- **DIP:** no dependency on downstream stages

### Why this is the best choice here

Because vision should be testable with synthetic images and should not depend on the real mechanism being connected.

---

## `Controller`

### Why SOLID is the right choice here

The controller should be responsible only for transforming image-space error into desired motion commands.

If the controller also performed kinematics or actuator safety shaping, then:

- controller tuning would be harder
- control reasoning would be mixed with mechanism-specific details
- it would become unclear whether bad motion came from control, kinematics, or actuator limits

For this project, isolating the controller is the best choice because it keeps the control law understandable, testable, and tunable.

### Why it exists (SRP)

Converts `SunEstimate` into `PlatformSetpoint` using deadband, confidence gating, and output limits.

### Depends on (DIP)

Consumes the estimation result only and emits setpoints independently of downstream implementation.

### Does NOT do

- no kinematics
- no actuator safety limiting
- no camera logic

### SOLID highlights

- **SRP:** control law only
- **OCP:** controller gains and policies can change without redesigning the rest of the pipeline
- **DIP:** does not depend on hardware-facing modules

### Why this is the best choice here

Because the controller is a logical stage in the pipeline with its own engineering meaning, and it should be possible to validate it separately from mechanism geometry and hardware output.

---

## `Kinematics3RRS`

### Why SOLID is the right choice here

Kinematics is mechanism-specific. It answers a different engineering question from control:

**“Given a desired platform orientation, what actuator commands are needed?”**

That is fundamentally different from:
- detecting the sun
- deciding desired motion
- enforcing safety limits
- driving hardware

In this project, kinematics is the part most likely to change if the platform geometry, calibration, or mathematical model changes. Therefore, isolating it is especially valuable.

### Why it exists (SRP)

Converts `PlatformSetpoint` into `ActuatorCommand`.

### Depends on (DIP)

Consumes setpoints and produces actuator-space outputs without hardware dependencies.

### Does NOT do

- no camera processing
- no control policy
- no PWM output
- no global orchestration

### SOLID highlights

- **SRP:** mechanism mapping only
- **OCP:** the internal model can evolve from a parameterised linear approximation to fuller geometry without forcing changes upstream
- **DIP:** independent from actuator hardware

### Why this is the best choice here

Because platform geometry and calibration are likely to evolve independently of the rest of the system. A clean kinematics boundary makes that possible.

---

## `ActuatorManager`

### Why SOLID is the right choice here

Safety shaping is a cross-cutting concern, but it should not be hidden inside kinematics or servo output code.

If clamping and rate limiting were scattered across multiple classes, then:

- the real safety boundary would be unclear
- testing the safety policy would be harder
- future changes in controller or kinematics might accidentally bypass safety behaviour

For this project, a dedicated actuator-safety stage is the most defensible design.

### Why it exists (SRP)

Applies safety shaping to actuator commands before they reach the output layer.

### Depends on (DIP)

Consumes and emits `ActuatorCommand` values without depending on the physical output mechanism.

### Does NOT do

- no kinematics computation
- no control law
- no PWM or I2C output

### SOLID highlights

- **SRP:** saturation and rate limiting only
- **OCP:** new shaping rules can be added without redesigning vision or kinematics
- **DIP:** safety policy is kept independent from hardware driver details

### Why this is the best choice here

Because safety policy deserves its own explicit boundary. In a real-time actuator chain, hidden safety logic is weak engineering; explicit safety logic is much easier to justify and test.

---

## `ServoDriver`

### Why SOLID is the right choice here

The hardware output boundary is another place where SOLID is clearly the right engineering choice.

Servo output involves hardware-specific behaviour such as:

- PWM mapping
- driver policy
- I2C-backed actuation
- log-only fallback or hardware modes

If this logic were mixed into `ActuatorManager`, `SystemManager`, or kinematics, then:

- hardware changes would ripple through unrelated code
- software-only testing would become harder
- debugging would become more confusing

In this project, keeping `ServoDriver` as the output boundary is the best choice because it isolates the final hardware actuation step.

### Why it exists (SRP)

Receives safe actuator targets and applies them to the actual actuator interface.

### Depends on (DIP)

Consumes already-safe commands and isolates the output implementation.

### Does NOT do

- no control decisions
- no kinematics
- no rate limiting policy
- no vision processing

### SOLID highlights

- **SRP:** final actuator output only
- **OCP:** output backend can evolve without affecting control or kinematics
- **DIP:** upstream logic does not depend on PWM/I2C implementation details

### Why this is the best choice here

Because actuator hardware is likely to be one of the most changeable parts of the system. Keeping it isolated protects the rest of the pipeline.

---

## `SystemManager`

### Why SOLID is the right choice here

Real-time embedded projects often fail architecturally when orchestration code turns into a “god object” that also contains algorithms, device logic, and safety rules.

This project avoids that by keeping `SystemManager` focused on coordination:

- start/stop order
- state transitions
- queue/thread ownership
- callback wiring

That is exactly the right place for orchestration, and exactly the wrong place for vision, control, or hardware algorithms.

### Why it exists (SRP)

Owns lifecycle and wires the modules into the full event-driven pipeline.

### Depends on (DIP)

Receives `std::unique_ptr<ICamera>` and coordinates modules through abstractions and callbacks.

### Does NOT do

- no image processing algorithm
- no control computation
- no kinematics model
- no low-level hardware output logic

### SOLID highlights

- **SRP:** orchestration and lifecycle only
- **DIP:** depends on interfaces such as `ICamera`
- **OCP:** pipeline composition can evolve while keeping module responsibilities clear

### Why this is the best choice here

Because this system genuinely needs a coordinator, but that coordinator must not absorb all the application logic. Keeping it orchestration-only is the most professional choice.

---

## `ThreadSafeQueue`

### Why SOLID is the right choice here

Concurrency support should not be mixed directly into application logic.

If queue behaviour were reimplemented ad hoc inside `SystemManager` or camera/actuator code, then:

- correctness would be harder to verify
- thread behaviour would be duplicated
- testing concurrency behaviour would be more difficult

A dedicated generic queue is therefore the best choice in this project.

### Why it exists (SRP)

Provides safe producer-consumer transfer between threads using blocking waits and stop semantics.

### Depends on (DIP)

Generic template independent of the solar-tracking domain.

### Does NOT do

- no application policy
- no control logic
- no logging ownership
- no thread ownership

### SOLID highlights

- **SRP:** queue behaviour only
- **ISP:** small focused interface for push/pop/stop operations
- **DIP:** application logic depends on a generic queue utility, not ad hoc thread-transfer code

### Why this is the best choice here

Because thread transfer semantics are important enough to deserve their own reusable and testable abstraction.

---

## `Logger`

### Why SOLID is the right choice here

Logging is useful across many modules, but logging policy should not be hard-coded separately inside each class.

A dedicated logger boundary keeps output behaviour more consistent and prevents unrelated code from being cluttered with direct output logic.

### Why it exists (SRP)

Provides a consistent logging API across the project.

### Depends on (DIP)

Modules depend on a logging abstraction instead of writing directly to raw output streams everywhere.

### Does NOT do

- no orchestration
- no control logic
- no hardware decision-making

### SOLID highlights

- **SRP:** logging only
- **DIP:** modules depend on logging boundary rather than hard-coded local output style

### Why this is the best choice here

Because consistent logging is especially important in a real-time event-driven system where debugging often depends on coherent timing and state output.

---

## Summary: why SOLID was chosen here

SOLID is useful in this project because the project has several clearly different engineering concerns:

- platform-specific input
- image processing
- control
- kinematics
- actuator safety
- hardware output
- orchestration
- concurrency support

Trying to merge these together would make the code harder to:

- test
- reason about
- port
- maintain
- justify against assessment criteria

In this specific project, SOLID is not just a software-engineering slogan. It is the most practical way to ensure that:

- hardware-specific code stays isolated
- software-only testing remains possible
- simulation and real hardware can share the same architecture
- safety logic is explicit
- modules keep stable responsibilities
- the event-driven pipeline remains understandable

The result is a structure where each class has a clear reason to exist, a clear boundary, and a role that matches the real architecture of the system.