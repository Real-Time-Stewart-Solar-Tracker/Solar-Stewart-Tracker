# System Requirements

## 1. Project Overview

The Solar Stewart Tracker is a realtime, event-driven embedded system
implemented in C++ on Linux (Raspberry Pi). The system detects the sun
position from camera frames and adjusts a 3RRS Stewart platform to
orient a solar panel toward the sun.

The system must operate with deterministic behaviour and measurable latency.

---

## 2. Functional Requirements

### FR1 – Event-Driven Camera Acquisition
- The system shall acquire frames via event-driven callbacks.
- The system shall not use polling loops or sleep-based timing.

### FR2 – Sun Detection
- The system shall detect the brightest region in the image.
- The system shall compute centroid (cx, cy).
- The system shall output a confidence metric (0–1).

### FR3 – Control Law
- The system shall compute error between centroid and image center.
- The system shall output desired tilt and pan angles.

### FR4 – Kinematics
- The system shall convert tilt/pan into three actuator commands.
- Actuator outputs must respect defined limits.

### FR5 – Safety
- Actuator outputs shall be rate-limited.
- Actuator commands shall be clamped within safe bounds.
- If sun confidence < threshold, motion shall reduce or stop.

### FR6 – Logging and Latency Measurement
- The system shall timestamp:
  - Frame capture
  - Sun estimate computation
  - Control computation
  - Actuation output
- The system shall compute:
  - Average latency
  - Worst-case latency
  - Jitter

---

## 3. Non-Functional Requirements

### NFR1 – Realtime Behaviour
- Threads must block while waiting for events.
- No busy waiting.
- No polling loops.
- No sleep-based timing for control logic.

### NFR2 – Code Structure
- Classes must follow SOLID principles.
- Each module must have a single responsibility.
- No global mutable state.

### NFR3 – Reproducibility
- The project must build using CMake.
- Build and run instructions must be documented.
- All dependencies must be clearly listed.

### NFR4 – Revision Control
- All changes must go through feature branches.
- Pull requests must be reviewed.
- Work must be distributed over project duration.

---

## 4. Success Criteria

The system is considered successful if:

- It detects sun direction in real time.
- The platform responds smoothly to changes.
- End-to-end latency is measured and documented.
- No polling loops are present.
- The project builds reproducibly on Raspberry Pi.

---

## 5. Failure Modes and Handling

### FM1 – Sun Not Detected
- Confidence below threshold.
- System shall reduce or stop motion.

### FM2 – Camera Failure
- System shall stop actuation safely.

### FM3 – Actuator Limit Reached
- Command shall be clamped.
- Event shall be logged.

---

## 6. Scope Limitations

The system does not include:
- Full weather prediction
- Battery management
- Multi-axis wind compensation
- Industrial-grade fault tolerance

The focus is realtime tracking and event-driven embedded design.


## Related documents
- User Stories and Use Cases: `docs/user_stories_use_cases.md`
- System Architecture: `docs/system_architecture.md`

## Traceability (Stories → Functional Requirements)

| User Story | Covers |
|---|---|
| US1 Start tracking | FR1, FR5 |
| US2 Maintain alignment | FR2, FR3, FR4, FR5 |
| US3 Manual mode | FR3, FR4, FR5 |
| US4 Safe stop | FR5 |
| US5 Logs + latency | FR6 |