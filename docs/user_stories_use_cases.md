
# Requirements (User Stories + Use Cases)

## System goal
A camera-based solar tracking system using a 3RRS Stewart-style platform.
The system estimates sun direction from camera frames and adjusts panel
orientation in real time using event-driven C++ on Raspberry Pi Linux.

---

## User Stories (prioritised)

### US1 (MUST) Start tracking
As a user, I want to start the tracker so that the platform aligns the solar panel automatically.

Acceptance criteria:
- Starts from a safe neutral pose.
- Camera starts and frames are processed.
- If the sun is not detected, system stays safe and reports SEARCHING.

---

### US2 (MUST) Maintain alignment (core realtime behaviour)
As a user, I want continuous alignment updates so that the panel stays facing the sun.

Acceptance criteria:
- Each new frame triggers an update (event-driven).
- Outputs are rate-limited and respect safety limits.
- If confidence drops, system reduces motion and tries to reacquire.

---

### US3 (SHOULD) Manual mode for calibration
As a user, I want a manual mode so I can test and calibrate safely.

Acceptance criteria:
- Switch AUTO <-> MANUAL.
- Manual commands respect limits and rate limits.
- Returning to AUTO is smooth.

---

### US4 (MUST) Safe stop
As a user, I want to stop the system at any time so it returns to a safe state.

Acceptance criteria:
- Stops motion quickly.
- Shuts down threads cleanly.
- Returns to neutral pose if possible.

---

### US5 (SHOULD) Logs + latency metrics
As an assessor/developer, I want latency logs so realtime performance can be verified.

Acceptance criteria:
- Logs timestamps: frame -> estimate -> control -> actuation.
- Summary stats: average + worst-case latency and jitter.

---

## Use Cases

### UC1 Start and track (AUTO)
1. User starts program.
2. System loads config and goes to NEUTRAL.
3. Camera begins streaming frames (event-driven).
4. SunTracker produces SunEstimate events.
5. Controller produces desired tilt/pan.
6. Kinematics produces actuator commands.
7. Actuators update and alignment improves.
8. System reports TRACKING.

Alt:
- Sun not found -> SEARCHING (safe, limited motion).
- Camera failure -> FAULT (stop outputs).
- Limit reached -> SATURATE + warn.

---

### UC2 Reacquire after loss
1. Tracking confidence drops.
2. Enter REACQUIRE.
3. Reduce motion + bounded search.
4. If sun recovered -> TRACKING.
5. If timeout -> FAULT or hold safe.

---

### UC3 Manual mode
1. User switches to MANUAL.
2. System stops automatic control updates.
3. User commands tilt/pan (bounded).
4. System moves platform safely.
5. User switches back to AUTO.

---

### UC4 Safe stop
1. User presses STOP.
2. System stops outputs, signals threads to end.
3. Return to neutral pose (if safe).
4. Release camera resources and exit.

---

## System State Machine (high level)

IDLE -> STARTUP -> NEUTRAL -> (SEARCHING <-> TRACKING) -> STOPPING -> IDLE
                           \-> MANUAL
                           \-> FAULT
