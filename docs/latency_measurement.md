# Latency Measurement Plan (Realtime Evidence)

This project measures end-to-end latency through the event-driven pipeline:

Camera → SunTracker → Controller → Kinematics3RRS → ActuatorManager → ServoDriver

## 1) What we measure (timestamps)

Each event stage records a timestamp using `std::chrono::steady_clock`:

- `t_capture`  : when a frame is captured/received (CameraPublisher)
- `t_estimate` : when sun estimate is produced (SunTracker)
- `t_control`  : when control setpoint is produced (Controller)
- `t_actuate`  : when actuator command is produced (Kinematics/Actuator path)

## 2) Latency metrics computed

For every frame_id we compute:

- `L_total = t_actuate - t_capture`
- `L_vision = t_estimate - t_capture`
- `L_control = t_control - t_estimate`
- `L_actuation = t_actuate - t_control`

We report (per run):
- Average latency (mean)
- Worst-case latency (max)
- Jitter (max - min) and optionally standard deviation

## 3) Acceptance targets (tolerances)

Targets are set to be measurable and realistic for Raspberry Pi class hardware:

- `L_total` average < 50 ms
- `L_total` worst-case < 120 ms
- Jitter < 80 ms

These are used to justify design choices (event-driven callbacks, blocking waits, bounded processing).

## 4) Implementation approach

A small `LatencyMonitor` module:
- receives stage timestamps keyed by `frame_id`
- computes per-frame latencies when all stage timestamps are present
- keeps streaming stats (mean/max/min/count)
- logs a summary on shutdown

No sleep-based timing is used in the realtime pipeline.