# Realtime Analysis

## 1. Realtime Goal

The Solar Stewart Tracker must respond to changes in target position with **low, bounded, and measurable software-side latency**.

The current implementation is structured as an **event-driven multi-threaded pipeline**:

- **Camera path** — backend callback delivers frames
- **Control thread** — vision + control + kinematics
- **Actuator thread** — safety shaping + servo output

The primary real-time requirement is **deterministic software-side latency from frame delivery to actuator command issuance**.

The current architecture is designed so that:

- camera data arrives through callbacks from the backend
- worker threads block while waiting for new work
- bounded queues prevent backlog accumulation
- control decisions are based on the newest available data

This matches the course requirement for **event-driven code using callbacks and waking threads**, rather than delay-driven single-threaded loops.

------------------------------------------------------------------------

## 2. Event-Driven Architecture

The runtime data path is event-driven.

### Thread roles

- **Camera/backend path** acquires frames and emits `FrameEvent` through a registered callback
- **Control thread** blocks on the frame queue and performs:
  - vision
  - control
  - kinematics
- **Actuator thread** blocks on the command queue and performs:
  - safety shaping
  - final actuator output

### Wake-up mechanisms

| Event Source | Execution Context | Wake-up Mechanism |
|---|---|---|
| Camera frame ready | camera/backend path | camera backend callback |
| `FrameEvent` available | control thread | blocking wait on `FrameQueue` |
| `ActuatorCommand` available | actuator thread | blocking wait on `CommandQueue` |

Important accuracy notes:

- the processing pipeline avoids **busy-wait polling**
- worker threads are woken by **callbacks** or **blocking queue waits**
- the Linux application layer may use blocking event-wait mechanisms such as `poll()`, which is consistent with blocking I/O used to wake execution paths

This gives the system the following real-time properties:

- no CPU busy-waiting in the core processing path
- no sleep-based timing driving the control loop
- clean separation between event producers and consumers
- bounded queue-based flow between stages

------------------------------------------------------------------------

## 3. Queue Design and Realtime Justification

Two bounded queues separate the execution domains.

### Frame queue (capacity = 1)

- keeps only the newest frame
- drops the oldest frame if full
- prevents frame backlog
- ensures control uses the freshest available sensor data

### Command queue (capacity = 1)

- keeps only the newest command
- prevents actuator lag caused by queued stale commands
- ensures the actuator path applies the newest available control output

This queue design gives the following real-time benefits:

- no unbounded memory growth
- no cumulative frame backlog
- bounded pipeline latency by design
- freshness is prioritised over historical completeness

This is a deliberate real-time design decision: for a tracker/control pipeline, **the newest frame is more valuable than preserving all older frames**.

------------------------------------------------------------------------

## 4. Latency Measurement Design

Latency instrumentation is implemented in `LatencyMonitor`.

The following timestamps are recorded for each frame:

| Stage | Timestamp |
|---|---|
| Frame received | `t_capture` |
| Sun estimate computed | `t_estimate` |
| Control computed | `t_control` |
| Actuation issued | `t_actuate` |

### End-to-end latency

**End-to-end latency = `t_actuate - t_capture`**

### Intermediate latencies

- **Vision latency** = `t_estimate - t_capture`
- **Control latency** = `t_control - t_estimate`
- **Actuation latency** = `t_actuate - t_control`

This allows the implementation to identify which stage dominates latency and whether the chosen architecture is justified quantitatively.

------------------------------------------------------------------------

## 5. Measured Results

Example measured run:

- **Frames:** 490
- **Platform:** Raspberry Pi class Linux hardware
- **Camera path:** libcamera-backed execution
- **Measurement scope:** software-side pipeline timing

| Stage | Avg (ms) | Min (ms) | Max (ms) | Jitter (ms) |
|---|---:|---:|---:|---:|
| Total | 1.106 | 0.695 | 4.441 | 3.746 |
| Vision | 1.035 | 0.673 | 3.748 | — |
| Control | 0.0036 | 0.0014 | 0.073 | — |
| Actuate | 0.067 | 0.012 | 1.970 | — |

These results show that the measured software-side end-to-end latency remains well below a typical **30 Hz** frame period of approximately **33 ms**.

------------------------------------------------------------------------

## 6. Quantitative Evaluation

### 6.1 Dominant latency source

Vision processing accounts for most of the measured end-to-end latency.

Average values show:

- **Vision** dominates the pipeline
- **Control** is computationally very small
- **Actuation command issuance** is also small relative to total latency

Engineering conclusion:

- vision is the main performance-critical stage
- control is not a meaningful bottleneck in the measured run
- the architectural separation is justified by measurement

### 6.2 Jitter analysis

Worst-case total latency: **4.441 ms**  
Minimum latency: **0.695 ms**  
Jitter: approximately **3.746 ms**

At **30 Hz**:

- frame period ≈ **33 ms**

Observed total latency is therefore:

- well below one frame period
- bounded in practice under the measured conditions
- low enough to avoid normal pipeline accumulation in the measured run

Engineering interpretation:

- queue bounding is effective
- backlog is prevented by the freshest-data policy
- the architecture provides stable response under the measured workload

------------------------------------------------------------------------

## 7. Realtime Compliance Verification

The current implementation satisfies the following real-time design requirements:

- event-driven frame arrival through callbacks
- blocking worker-thread wake-up on queues
- no busy-wait polling in the core processing path
- no sleep-based timing used to drive the control pipeline
- bounded queues between acquisition, control, and actuation
- quantitative latency instrumentation
- measured software-side worst-case end-to-end latency

This is consistent with the course expectation that real-time processing should be achieved using **callbacks, waking threads, timers, signals, and blocking I/O**, rather than using wait statements to establish timing.

------------------------------------------------------------------------

## 8. Engineering Decisions Supported by Data

The measured data supports the following design decisions:

1. **No optimisation is currently required in the control stage**  
   Its contribution to total latency is negligible in the measured run.

2. **Vision is the main optimisation candidate**  
   Most measured end-to-end latency originates there.

3. **Queue capacity = 1 is justified**  
   Measured latency is already low, and freshness is more important than preserving old frames.

4. **Freshest-data policy prevents latency accumulation**  
   Old frames and commands do not build up into backlog.

5. **Separated execution stages are justified**  
   Acquisition, computation, and output remain isolated and measurable.

These are not generic claims; they follow directly from the measured latency breakdown.

------------------------------------------------------------------------

## 9. Scope and Limitations

This document supports claims about:

- software-side timing behaviour
- event-driven architecture
- bounded queue behaviour
- relative computational cost of major pipeline stages

This document does **not** by itself prove:

- hard real-time guarantees
- full physical actuator movement latency
- complete end-to-end electromechanical response of the real platform under all conditions
- behaviour under every possible hardware failure mode

The reported numbers are therefore **software-side empirical measurements**, not formal hard real-time guarantees.

------------------------------------------------------------------------

## 10. Conclusion

The Solar Stewart Tracker demonstrates:

- bounded and measurable software-side real-time behaviour
- event-driven processing based on callbacks and waking threads
- end-to-end software-side latency below **5 ms** in the reported measured run
- low computational cost outside the vision stage
- a clear separation between acquisition, computation, and output

The measured results justify the current queue-based, multi-threaded design and show that the implementation satisfies the project’s real-time responsiveness goals under the tested conditions.