# Latency Measurement Plan (Realtime Evidence)

This project measures end-to-end latency across the event-driven processing pipeline:

Camera → SunTracker → Controller → Kinematics3RRS → ActuatorManager → ServoDriver

Latency is measured per-frame using monotonic timestamps and aggregated for statistical analysis.

---

## 1. Timestamp Strategy

Each processing stage records a timestamp using `std::chrono::steady_clock`, which guarantees monotonic behavior (no jumps due to system clock adjustments).

Recorded timestamps:

- `t_capture`   — frame captured / received (ICamera)
- `t_estimate`  — sun estimate produced (SunTracker)
- `t_control`   — control setpoint computed (Controller)
- `t_actuate`   — actuator command issued (Kinematics / Actuator path)

All timestamps are associated with a unique `frame_id`.

---

## 2. Latency Metrics

For each `frame_id`, the following metrics are computed:

| Metric | Definition | Meaning |
|--------|------------|---------|
| L_vision | `t_estimate - t_capture` | Vision processing time |
| L_control | `t_control - t_estimate` | Control computation time |
| L_actuation | `t_actuate - t_control` | Kinematics + actuator path time |
| L_total | `t_actuate - t_capture` | End-to-end closed-loop latency |

Note:  
`L_total` is directly measured from first to last stage and is not computed as a simple sum to avoid compounding rounding error.

---

## 3. Reported Statistics

For each run, the system reports:

- Mean latency
- Minimum latency
- Maximum latency
- Jitter (`max - min`)
- Sample count

Statistics are accumulated online in `LatencyMonitor`.

---

## 4. Acceptance Targets (Raspberry Pi Class Hardware)

Targets are selected to be realistic for embedded Linux on Raspberry Pi:

| Metric | Target (Average) | Target (Worst Case) |
|--------|------------------|---------------------|
| L_total | < 50 ms | < 120 ms |
| L_vision | < 30 ms | — |
| L_control | < 5 ms | — |
| L_actuation | < 10 ms | — |
| Jitter | < 80 ms | — |

These thresholds justify the architectural decisions:
- Event-driven callbacks (no polling)
- No sleep-based timing loops
- Bounded computation per stage
- Separation of processing responsibilities (SRP)

---

## 5. Implementation

Latency measurement is implemented in the `LatencyMonitor` module:

- Tracks timestamps per `frame_id`
- Finalizes latency once all stages are recorded
- Maintains streaming statistics (mean/min/max/count)
- Prunes stale entries to prevent unbounded memory growth
- Logs a summary on shutdown

The realtime pipeline contains no sleep-based timing mechanisms.  
All stage transitions are event-driven and non-blocking.

---

## 6. Realtime Compliance Evidence

The measured latency statistics demonstrate:

- Deterministic stage execution
- Bounded per-stage computation
- Stable jitter under expected load
- Responsiveness suitable for solar tracking control

This provides quantitative evidence that the system satisfies realtime responsiveness requirements.