# Requirements (User Stories + Use Cases)

## System goal

A camera-based solar tracking system using a 3RRS Stewart-style platform.  
The system estimates sun direction from camera frames and adjusts panel orientation in real time using event-driven C++ on Raspberry Pi Linux.

---

## User Stories (prioritised)

### US1 (MUST) Start tracking

As a user, I want to start the tracker so that the platform aligns the solar panel automatically.

Acceptance criteria:
- Starts by parking the servos to a safe calibrated startup position.
- Camera starts and frames are processed.
- If the sun is not detected, the system remains in a safe searching behaviour and reports `SEARCHING`.
- If startup hardware requirements are not met, the system enters `FAULT`.

---

### US2 (MUST) Maintain alignment (core real-time behaviour)

As a user, I want continuous alignment updates so that the panel stays facing the sun.

Acceptance criteria:
- Each new frame triggers an update through the event-driven pipeline.
- Outputs are bounded, rate-limited, and respect configured safety limits.
- If confidence drops, the system reduces motion and transitions from `TRACKING` back to `SEARCHING`.

---

### US3 (SHOULD) Manual mode for calibration

As a user, I want a manual mode so I can test and calibrate safely.

Acceptance criteria:
- Switch between `AUTO` and `MANUAL`.
- Manual commands respect limits and rate limits.
- Returning to automatic operation resumes normal event-driven tracking behaviour.

---

### US4 (MUST) Safe stop

As a user, I want to stop the system at any time so it returns to a safe state.

Acceptance criteria:
- Stops processing and actuation cleanly.
- Shuts down worker threads cleanly.
- Stops camera streaming and releases resources.
- Applies configured stop/park behaviour where enabled.

---

### US5 (SHOULD) Logs and latency metrics

As an assessor or developer, I want latency logs so real-time performance can be evaluated.

Acceptance criteria:
- Records timestamps across the software pipeline: frame, vision, control, and actuation.
- Reports summary statistics including average, minimum, maximum, and jitter.

---

## Use Cases

### UC1 Start and track (AUTO)

1. User starts the program.
2. System loads configuration and enters `STARTUP`.
3. System starts camera and worker threads.
4. System parks the servos to the configured startup position and enters `NEUTRAL`.
5. Camera delivers frames through the callback-based input path.
6. `SunTracker` produces a sun estimate from each frame.
7. `Controller` produces desired tilt/pan commands.
8. `Kinematics3RRS` produces actuator commands.
9. `ActuatorManager` shapes commands within configured limits.
10. `ServoDriver` applies output commands.
11. System transitions between `SEARCHING` and `TRACKING` according to confidence.

Alternative paths:
- Sun not found -> `SEARCHING`
- Camera startup failure -> `FAULT`
- Required actuator hardware unavailable at startup -> `FAULT`
- Invalid kinematics result surfaced to the system manager -> `FAULT`

---

### UC2 Continue operation when confidence drops

1. System is in `TRACKING`.
2. Tracking confidence drops below the configured threshold.
3. System transitions back to `SEARCHING`.
4. Frame-driven processing continues.
5. If confidence recovers, system returns to `TRACKING`.

---

### UC3 Manual mode

1. User switches to `MANUAL`.
2. System stops automatic tracking updates from driving motion commands.
3. User issues manual commands within bounded limits.
4. System moves platform through the normal actuator path.
5. User switches back to automatic operation.
6. Normal frame-driven tracking resumes.

---

### UC4 Safe stop

1. User requests stop.
2. System enters `STOPPING`.
3. System stops accepting normal processing flow.
4. Worker threads are signalled to stop and are joined cleanly.
5. Camera resources are released.
6. Configured stop/park behaviour is applied where enabled.
7. System returns to `IDLE`.

---

## System State Machine (high level)

`IDLE -> STARTUP -> NEUTRAL -> (SEARCHING <-> TRACKING) -> STOPPING -> IDLE`  
`                              \-> MANUAL`  
`                              \-> FAULT`