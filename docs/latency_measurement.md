# Realtime Latency Measurement and Evidence

This document presents quantitative evidence for the software-side latency of the system, measured on real hardware using the physical camera backend.

The measured processing path is:

**Camera → FrameQueue → SunTracker → (state-dependent Controller) → ManualImuCoordinator → Kinematics3RRS → ActuatorManager → ServoDriver**

Latency data is collected at runtime using monotonic timestamps and written to `artefacts/latency.csv`.

---

## 0. Measurement Platform

| Parameter | Value |
|---|---|
| Hardware | Raspberry Pi 5 (BCM2712, Cortex-A76, 4-core, 4 GB RAM) |
| Operating system | Raspberry Pi OS (Debian Trixie), 64-bit (aarch64) |
| Kernel | Linux 6.6.51+rpt-rpi-2712 (aarch64) |
| Compiler | GCC 12.2.0 (aarch64-linux-gnu) |
| Build type | Release (`-O2`) |
| Camera backend | libcamera2opencv (IMX219 CSI, 640×480, RGB888, 30 fps) |
| Servo backend | ServoDriver::StartupPolicy::RequireHardware |
| Frames captured | 1367 |
| Run duration | 46.6 seconds |

---

## 1. Measured Results

Measurement extracted from runtime output:

- Frames processed: 1367
- Data source: LatencyMonitor summary

| Metric | Average (ms) | Minimum (ms) | Maximum (ms) | Jitter (ms) |
|---|---:|---:|---:|---:|
| **L_total** | **2.397656** | 1.869860 | 4.553350 | 2.683490 |
| L_vision (capture to estimate) | 2.379005 | 1.859520 | 4.531040 | 2.671520 |
| L_control (estimate to control) | 0.001495 | 0.000500 | 0.070347 | 0.069847 |
| L_actuation (control to actuate) | 0.017155 | 0.008648 | 0.156285 | 0.147637 |

---

## 2. Frame-Interval Analysis

The following frame-arrival statistics confirm that the measurement was performed using the real camera backend rather than the synthetic timer-driven simulator.

| Metric | Value |
|---|---|
| Mean frame interval | 34.15 ms |
| Frame interval standard deviation | 4.46 ms |
| Minimum frame interval | 31.20 ms |
| Maximum frame interval | 66.78 ms |
| Effective frame rate | 29.3 fps |

A `timerfd`-driven simulated backend produces frame intervals with sub-millisecond standard deviation and negligible jitter. The measured standard deviation of 4.46 ms and the 35.6 ms jitter range are consistent with real camera sensor readout variation, libcamera DMA buffer completion timing, and kernel scheduling effects. This confirms that the latency data reflects genuine hardware-in-the-loop operation.

---

## 3. Pipeline Interpretation

The latency measurements correspond to the following stages:

- **L_vision** — frame arrival to SunTracker output; includes frame handling and vision processing
- **L_control** — Controller and ManualImuCoordinator; includes state-dependent execution (skipped in MANUAL)
- **L_actuation** — Kinematics3RRS to ActuatorManager to ServoDriver
- **L_total** — full path from frame reception to actuator command issuance

---

## 4. Timestamp Strategy

Timestamps are recorded using `std::chrono::steady_clock`. Per frame:

```cpp
t_capture  — frame received
t_estimate — SunTracker output
t_control  — control / coordination complete
t_actuate  — actuator command issued
```

All timestamps are associated with a frame identifier and stored until the full pipeline completes.

---

## 5. Latency Definitions

| Metric | Definition |
|---|---|
| L_vision | t_estimate minus t_capture |
| L_control | t_control minus t_estimate |
| L_actuation | t_actuate minus t_control |
| L_total | t_actuate minus t_capture |

Total latency is measured directly, not inferred.

---

## 6. Measurement Method

Latency is collected using the runtime `LatencyMonitor`:

- timestamps recorded at each pipeline stage
- per-frame data aggregated
- summary statistics computed at shutdown
- raw data exported to CSV at `artefacts/latency.csv`

Execution characteristics during measurement:

- camera produces frames via libcamera callback through libcamera2opencv
- control thread blocks on frame queue
- actuator thread blocks on command queue
- no polling or sleep-based scheduling is used in the processing path

---

## 7. Observations

### 7.1 Dominant Latency Source

- vision processing dominates total latency (average 2.38 ms)
- control stage is negligible (average 0.0015 ms)
- actuation stage is small (average 0.017 ms)

### 7.2 End-to-End Behaviour

- average latency approximately 2.40 ms
- worst-case latency approximately 4.55 ms
- jitter approximately 2.68 ms

### 7.3 Throughput Context

At 30 Hz, frame period is approximately 33 ms. Measured average latency of 2.40 ms is approximately 7.3% of one frame period, meaning:

- processing completes well before the next frame arrives
- no systematic backlog accumulation occurs
- substantial timing margin exists for transient load spikes

---

## 8. Design Decisions Informed by Measured Latency

The measured latency data directly shaped the following architectural choices.

**Frame queue capacity set to 2**

Average end-to-end latency of 2.40 ms is well below the 33 ms frame period at 30 Hz. A capacity of 2 allows the queue to absorb a brief control-thread stall without blocking the camera callback, while keeping the bound tight enough that the control thread never processes a frame more than one period old. A capacity of 1 would cause unnecessary frame drops under any transient load; a larger capacity would allow stale frames to accumulate and degrade tracking responsiveness.

**Command queue capacity set to 8**

The actuation stage (average 0.017 ms, worst case 0.16 ms) is faster than the vision stage by two orders of magnitude. The larger command queue absorbs short bursts from kinematics output without risking drops on the actuator thread, while remaining bounded enough to prevent unbounded command backlog under sustained load.

**30 Hz camera frame rate**

Worst-case measured pipeline latency of 4.55 ms is approximately 13.8% of a 33 ms frame period. This margin is sufficient for tracking a slowly moving solar target and confirms that 30 Hz is an appropriate operating rate for this application. A higher frame rate would not reduce tracking latency meaningfully given that vision processing dominates the total path.

**Separate control and actuator threads**

Vision and control stages combined (average 2.38 ms) are substantially slower than the actuation stage (average 0.017 ms). Separating these into two threads via a bounded queue ensures that vision processing jitter does not introduce unnecessary delay into the servo output path. The actuator thread processes commands as soon as they arrive rather than waiting for the next vision cycle.

---

## 9. Scope of Measurement

The measurements cover software-side latency only.

Excluded from measurement:

- camera sensor exposure time
- kernel buffering before userspace delivery
- servo mechanical response time
- physical platform motion and settling
- environmental disturbances

The results represent userspace processing latency from frame arrival to actuator command issuance.

---

## 10. Summary

| Metric | Value |
|---|---|
| Average end-to-end latency | approximately 2.40 ms |
| Worst-case end-to-end latency | approximately 4.55 ms |
| Jitter | approximately 2.68 ms |

The system demonstrates consistent end-to-end processing with bounded latency and no accumulation. The dominant cost lies in the vision stage with minimal overhead in control and actuation. The pipeline architecture — frame queue capacity 2, command queue capacity 8, 30 Hz frame rate, dual worker threads — is directly supported by the measured timing evidence presented above. All measurements were taken on real hardware using the libcamera2opencv backend with an IMX219 sensor at 640×480 resolution.