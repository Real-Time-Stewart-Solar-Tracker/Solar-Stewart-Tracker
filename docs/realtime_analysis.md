# Realtime Analysis

## 1. Realtime Goal

The Solar Stewart Tracker must respond to changes in sun direction with low, bounded and measurable latency.

The system is implemented as an **event-driven, three-stage multi-threaded pipeline**:

- T1 Camera (backend-owned)
- T2 Control (vision + control + kinematics)
- T3 Actuator (safety + servo output)

Threads block while waiting for events.
No polling loops are used.
No sleep-based timing is used in the control path.
Wake-ups occur only when new data arrives.

The primary realtime requirement is deterministic end-to-end latency from frame capture to actuator command.

---

## 2. Event-Driven Architecture (Wake-up Mechanisms)

| Event Source              | Thread       | Wake-up Mechanism                                      |
|---------------------------|-------------|--------------------------------------------------------|
| Camera frame ready        | T1 Camera   | Camera backend callback                                |
| FrameEvent available      | T2 Control  | Blocking wait on FrameQueue (condition variable)       |
| ActuatorCommand available | T3 Actuator | Blocking wait on CommandQueue (condition variable)     |

All worker threads block on condition variables and are woken only when new data becomes available.

This ensures:
- No CPU busy-waiting
- Deterministic scheduling behaviour
- Clean separation of event producers and consumers
- No artificial timing delays

---

## 3. Queue Design and Realtime Justification

Two bounded queues separate execution domains:

### FrameQueue (capacity = 1)

- Keeps only the newest frame
- Drops oldest frame if full
- Prevents frame backlog
- Guarantees control operates on freshest data

### CommandQueue (capacity = 1)

- Keeps only the newest command
- Prevents actuator lag
- Ensures latest control output is applied

This bounding guarantees:

- No unbounded memory growth
- Upper bound on pipeline latency
- No cumulative delay across frames

This design decision was made based on realtime control principles.

---

## 4. Latency Measurement Design

Latency instrumentation is implemented in `LatencyMonitor`.

The following timestamps are recorded:

| Stage                     | Timestamp     |
|---------------------------|--------------|
| Frame received            | `t_capture`  |
| Sun estimate computed     | `t_estimate` |
| Control computed          | `t_control`  |
| Actuation issued          | `t_actuate`  |

End-to-end latency is defined as:

End-to-end latency = t_actuate − t_capture

Intermediate latencies:

- Vision latency = `t_estimate − t_capture`
- Control latency = `t_control − t_estimate`
- Actuation latency = `t_actuate − t_control`

This enables identification of computational bottlenecks.

---
## 5. Measured Results (Raspberry Pi – Hardware Execution)

Measurement run: 695 frames  
Platform: Raspberry Pi OS (Bookworm)  
Camera: libcamera YUV420 pipeline  

| Stage    | Avg (ms) | Min (ms) | Max (ms) | Jitter (ms) |
|----------|----------|----------|----------|-------------|
| Total    | 0.800    | 0.636    | 3.806    | 3.171       |
| Vision   | 0.795    | 0.631    | 3.803    | —           |
| Control  | 0.0040   | 0.0012   | 1.252    | —           |
| Actuate  | 0.0010   | 0.0006   | 0.0052   | —           |

---

## 6. Quantitative Evaluation

### 6.1 Dominant Latency Source

Vision processing accounts for ~99% of end-to-end latency.

Control and actuation computation are negligible (<0.01 ms average).

Engineering conclusion:

- Vision is the only performance-critical subsystem.
- Control and kinematics are computationally insignificant.
- Architectural separation is validated by measurement.

---

### 6.2 Jitter Analysis

Worst-case total latency: 3.806 ms  
Minimum latency: 0.636 ms  
Jitter ≈ 3.17 ms  

At 30 Hz:
Frame period ≈ 33 ms  

Observed latency is:

- Well below one frame period
- Highly stable
- Bounded and deterministic

Engineering interpretation:

- Linux scheduling overhead is minimal.
- Bounded queues prevent accumulation.
- Architecture comfortably supports higher frame rates.

---

## 7. Realtime Compliance Verification

The system satisfies:

- No polling loops
- No sleep-based timing in control path
- Condition-variable blocking semantics
- Event-driven camera callbacks
- Bounded queues
- Quantitative latency instrumentation
- Measured worst-case bounds

Measured performance confirms deterministic, bounded behaviour suitable for closed-loop control.

---

## 8. Engineering Decisions Supported by Data

Based on empirical results:

1. No optimisation required in control or kinematics.
2. Vision layer is the only optimisation candidate.
3. Bounded queue capacity = 1 is justified.
4. Freshest-data policy prevents latency accumulation.
5. Architecture can scale without redesign.

All conclusions are derived from measured data.

---

## 9. Conclusion

The Solar Stewart Tracker:

- Demonstrates bounded and measurable realtime performance.
- Achieves < 4 ms worst-case end-to-end latency.
- Maintains low jitter under Linux scheduling.
- Uses a production-level event-driven architecture.
- Separates acquisition, computation and actuation cleanly.

The system satisfies embedded realtime responsiveness requirements.