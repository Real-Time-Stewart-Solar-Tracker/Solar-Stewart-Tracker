# System Requirements Specification

---

# 1. System Overview

The Solar Stewart Tracker is a realtime, event-driven embedded system
implemented in C++17 and executed on Linux (Raspberry Pi).

The system acquires camera frames, estimates the sun position using
image processing, computes control commands, and actuates a 3RRS
Stewart-platform mechanism to orient a solar panel toward the sun.

The system shall exhibit deterministic behaviour, bounded latency,
and measurable performance.

---

# 2. Functional Requirements

## FR1 – Event-Driven Camera Acquisition

- The system shall acquire frames using callback-based mechanisms.
- The system shall not use polling loops for frame acquisition.
- The system shall not use sleep-based timing to trigger frame processing.

Verification:
- Code inspection confirms callback-based camera interface.
- No sleep-based timing in control path.

---

## FR2 – Sun Detection

- The system shall identify the brightest region in the image.
- The system shall compute the centroid coordinates (cx, cy).
- The system shall output a confidence value in the range [0, 1].
- The system shall operate on the most recent available frame only.

Verification:
- Unit tests validate centroid calculation.
- Confidence output verified via test scenarios.

---

## FR3 – Control Law

- The system shall compute error between centroid and image centre.
- The system shall convert error into desired tilt and pan angles.
- The control computation shall complete within bounded time.

Verification:
- Unit tests validate error-to-angle mapping.
- Latency instrumentation confirms bounded execution.

---

## FR4 – Kinematic Conversion

- The system shall convert tilt and pan commands into three actuator targets.
- Actuator commands shall respect defined mechanical limits.
- Invalid configurations shall not produce unsafe outputs.

Verification:
- Unit tests validate kinematic mapping.
- Output clamping verified in actuator module.

---

## FR5 – Safety Behaviour

- Actuator outputs shall be clamped within safe bounds.
- Actuator commands shall be rate-limited where required.
- If confidence falls below threshold, motion shall reduce or stop.
- On system stop, actuators shall return to neutral or safe state.

Verification:
- Safety checks present in ServoDriver and ActuatorManager.
- Manual testing confirms safe stop behaviour.

---

## FR6 – Logging and Latency Measurement

The system shall record timestamps for:

- Frame capture (`t_capture`)
- Sun estimate computation (`t_estimate`)
- Control computation (`t_control`)
- Actuation command issue (`t_actuate`)

The system shall compute:

- Average latency
- Minimum latency
- Maximum latency
- Jitter

Verification:
- LatencyMonitor produces statistical summary.
- Measured data included in realtime analysis document.

---

# 3. Non-Functional Requirements

## NFR1 – Realtime Behaviour

- Threads shall block while waiting for events.
- No busy waiting shall be present.
- No polling loops shall be used.
- No sleep-based timing shall be used in control logic.
- End-to-end latency shall be bounded and measurable.

Verification:
- Code inspection confirms condition-variable usage.
- Latency document provides empirical measurements.

---

## NFR2 – Modular Architecture

- The system shall follow SOLID principles.
- Each module shall have a single responsibility.
- No global mutable state shall be used.
- Hardware interfaces shall be abstracted behind interfaces.

Verification:
- Architectural diagrams and code structure confirm separation.
- No global shared mutable state present.

---

## NFR3 – Reproducibility

- The system shall build using CMake.
- All dependencies shall be documented.
- The build shall succeed on a clean system with documented dependencies.
- Unit tests shall execute successfully after build.

Verification:
- Reproducibility document defines clean build procedure.
- Tests pass on verified environments.

---

## NFR4 – Revision Control

- Development shall use feature branches.
- Changes shall be reviewed via pull requests.
- Commit history shall demonstrate progressive development.

Verification:
- Repository history confirms branching model.

---

# 4. Success Criteria

The system shall be considered successful if:

- The sun position is detected in realtime.
- The Stewart platform responds smoothly to sun movement.
- End-to-end latency is measured and documented.
- No polling or sleep-based timing is present.
- The project builds reproducibly on Raspberry Pi.

---

# 5. Failure Modes and Handling

## FM1 – Sun Not Detected

Condition:
- Confidence < threshold.

Response:
- Motion shall reduce or stop.
- State transition shall be logged.

---

## FM2 – Camera Failure

Condition:
- Camera backend stops delivering frames.

Response:
- Actuation shall cease safely.
- System shall enter safe state.

---

## FM3 – Actuator Limit Reached

Condition:
- Command exceeds mechanical range.

Response:
- Command shall be clamped.
- Event shall be logged.

---

# 6. Scope Limitations

The system does not include:

- Weather prediction
- Battery management
- Wind compensation
- Industrial fault tolerance
- Outdoor environmental hardening

The focus is realtime tracking and event-driven embedded design.

---

# 7. Traceability

## Related Documents

- User Stories and Use Cases: `docs/user_stories_use_cases.md`
- System Architecture: `docs/system_architecture.md`
- Realtime Analysis: `docs/realtime_analysis.md`

## Traceability Matrix

| User Story | Functional Requirements Covered |
|------------|----------------------------------|
| US1 Start tracking | FR1, FR5 |
| US2 Maintain alignment | FR2, FR3, FR4, FR5 |
| US3 Manual mode | FR3, FR4, FR5 |
| US4 Safe stop | FR5 |
| US5 Logs and latency | FR6 |