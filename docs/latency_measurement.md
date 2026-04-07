# Realtime Latency Measurement and Evidence

This document presents quantitative evidence for the software-side latency of the system.

The measured processing path is:

**Camera → FrameQueue → SunTracker → (state-dependent Controller) → ManualImuCoordinator → Kinematics3RRS → ActuatorManager → ServoDriver**

Latency data is collected at runtime using monotonic timestamps and written to `artefacts/latency.csv`.

---

## 0. Measurement Platform

| Parameter | Value |
|---|---|
| Hardware | Raspberry Pi 5 (BCM2712, Cortex-A76, 4-core, 4 GB RAM) |
| Operating system | Raspberry Pi OS Bookworm (Debian 12), 64-bit (aarch64) |
| Kernel | Linux 6.6.51+rpt-rpi-2712 (aarch64) |
| Compiler | GCC 12.2.0 (aarch64-linux-gnu) |
| Build type | Release (`-O2`) |
| Camera backend | SimulatedPublisher (timerfd + poll, 30 Hz) |
| Servo backend | ServoDriver::StartupPolicy::LogOnly |
| Frames captured | 419 |

---

## 1. Measured Results

Measurement extracted from runtime output:

- Frames processed: 419
- Data source: LatencyMonitor summary

| Metric | Average (ms) | Minimum (ms) | Maximum (ms) | Jitter (ms) |
|---|---:|---:|---:|---:|
| **L_total** | **8.369570** | 6.829599 | 14.565364 | 7.735765 |
| L_vision (capture to estimate) | 8.242496 | 6.755912 | 14.530086 | 7.774174 |
| L_control (estimate to control) | 0.014822 | 0.006038 | 0.363637 | 0.357599 |
| L_actuation (control to actuate) | 0.112253 | 0.019426 | 3.095969 | 3.076543 |

---

## 2. Pipeline Interpretation

The latency measurements correspond to the following stages:

- **L_vision** — frame arrival to SunTracker output; includes frame handling and vision processing
- **L_control** — Controller and ManualImuCoordinator; includes state-dependent execution (skipped in MANUAL)
- **L_actuation** — Kinematics3RRS to ActuatorManager to ServoDriver
- **L_total** — full path from frame reception to actuator command issuance

---

## 3. Timestamp Strategy

Timestamps are recorded using `std::chrono::steady_clock`. Per frame:

```cpp
t_capture  — frame received
t_estimate — SunTracker output
t_control  — control / coordination complete
t_actuate  — actuator command issued
```

All timestamps are associated with a frame identifier and stored until the full pipeline completes.

---

## 4. Latency Definitions

| Metric | Definition |
|---|---|
| L_vision | t_estimate minus t_capture |
| L_control | t_control minus t_estimate |
| L_actuation | t_actuate minus t_control |
| L_total | t_actuate minus t_capture |

Total latency is measured directly, not inferred.

---

## 5. Measurement Method

Latency is collected using the runtime `LatencyMonitor`:

- timestamps recorded at each pipeline stage
- per-frame data aggregated
- summary statistics computed at shutdown
- raw data exported to CSV at `artefacts/latency.csv`

Execution characteristics during measurement:

- camera produces frames via callback
- control thread blocks on frame queue
- actuator thread blocks on command queue
- no polling or sleep-based scheduling is used in the processing path

---

## 6. Observations

### 6.1 Dominant Latency Source

- vision processing dominates total latency (average 8.24 ms)
- control stage is negligible (average 0.015 ms)
- actuation stage is small but variable (average 0.11 ms)

### 6.2 End-to-End Behaviour

- average latency approximately 8.37 ms
- worst-case latency approximately 14.57 ms
- jitter approximately 7.74 ms

### 6.3 Throughput Context

At 30 Hz, frame period is approximately 33 ms. Measured latency remains well below one frame period, meaning:

- processing completes before the next frame arrives
- no systematic backlog accumulation occurs

---

## 7. Design Decisions Informed by Measured Latency

The measured latency data directly shaped the following architectural choices.

**Frame queue capacity set to 2**

Average end-to-end latency of 8.37 ms is well below the 33 ms frame period at 30 Hz. A capacity of 2 allows the queue to absorb a brief control-thread stall without blocking the camera callback, while keeping the bound tight enough that the control thread never processes a frame more than one period old. A capacity of 1 would cause unnecessary frame drops under any transient load; a larger capacity would allow stale frames to accumulate and degrade tracking responsiveness.

**Command queue capacity set to 8**

The actuation stage (average 0.11 ms, worst case 3.1 ms) is faster than the vision stage by two orders of magnitude. The larger command queue absorbs short bursts from kinematics output without risking drops on the actuator thread, while remaining bounded enough to prevent unbounded command backlog under sustained load.

**30 Hz camera frame rate**

Worst-case measured pipeline latency of 14.57 ms is approximately 44% of a 33 ms frame period. This margin is sufficient for tracking a slowly moving solar target and confirms that 30 Hz is an appropriate operating rate for this application. A higher frame rate would not reduce tracking latency meaningfully given that vision processing dominates the total path.

**Separate control and actuator threads**

Vision and control stages combined (average 8.26 ms) are substantially slower than the actuation stage (average 0.11 ms). Separating these into two threads via a bounded queue ensures that vision processing jitter does not introduce unnecessary delay into the servo output path. The actuator thread processes commands as soon as they arrive rather than waiting for the next vision cycle.

---

## 8. Scope of Measurement

The measurements cover software-side latency only.

Excluded from measurement:

- camera sensor exposure time
- kernel buffering before userspace delivery
- servo mechanical response time
- physical platform motion and settling
- environmental disturbances

The results represent userspace processing latency from frame arrival to actuator command issuance.

---

## 9. Summary

| Metric | Value |
|---|---|
| Average end-to-end latency | approximately 8.37 ms |
| Worst-case end-to-end latency | approximately 14.57 ms |
| Jitter | approximately 7.74 ms |

The system demonstrates consistent end-to-end processing with bounded latency and no accumulation. The dominant cost lies in the vision stage with minimal overhead in control and actuation. The pipeline architecture — frame queue capacity 2, command queue capacity 8, 30 Hz frame rate, dual worker threads — is directly supported by the measured timing evidence presented above.
