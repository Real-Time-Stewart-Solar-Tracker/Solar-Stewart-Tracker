# Realtime Latency Measurement and Evidence

This document presents quantitative evidence for the **software-side latency** of the current Solar Stewart Tracker repository on Raspberry Pi class Linux hardware.

The measured pipeline is the event-driven userspace path:

**Camera -> SunTracker -> Controller -> Kinematics3RRS -> ActuatorManager -> ServoDriver**

All measurements are collected during execution using **monotonic timestamps** recorded inside the software pipeline.

------------------------------------------------------------------------

## 1) Measured results (empirical data)

Measurement setup:

- **Platform:** Raspberry Pi OS / Linux
- **Camera path:** `LibcameraPublisher`
- **Actuation mode:** `ServoDriver` hardware mode via PCA9685 on `/dev/i2c-1`
- **Camera stream:** `640x480-YUV420`
- **Processed frames:** **829**

| Metric | Average (ms) | Minimum (ms) | Maximum (ms) | Jitter (ms) |
|---|---:|---:|---:|---:|
| **L_total** | **1.108** | 0.699 | 5.336 | 4.637 |
| L_vision | 1.053 | 0.679 | 5.314 | — |
| L_control | 0.0045 | 0.0016 | 0.942 | — |
| L_actuation | 0.050 | 0.0117 | 3.250 | — |

Where:

- **L_vision** = `SunTracker` processing time
- **L_control** = control computation time
- **L_actuation** = command generation / actuation software path after control output
- **L_total** = end-to-end software latency from frame reception in userspace to actuator command issue
- **Jitter** = `max - min` for `L_total`

Observed software-side performance:

- End-to-end average latency ≈ **1.11 ms**
- Worst-case measured latency ≈ **5.34 ms**
- Measured jitter ≈ **4.64 ms**

------------------------------------------------------------------------

## 2) Acceptance criteria

The following acceptance targets are used as practical engineering targets for this repository’s **userspace software pipeline**.

| Metric | Acceptance target | Rationale |
|---|---:|---|
| **L_total** | **< 10 ms** | Keeps software latency well below typical camera frame periods and preserves control responsiveness |
| **Jitter** | **< 10 ms** | Indicates bounded scheduling variation at the software level |
| **L_control** | **< 5 ms** | Ensures control computation remains small relative to frame-to-frame processing time |

Against these targets, the measured software path shows substantial timing margin.

------------------------------------------------------------------------

## 3) Timestamp strategy

Each stage records timestamps using:

```cpp
std::chrono::steady_clock
````

This provides:

* monotonic behaviour
* immunity to wall-clock adjustments
* suitable duration measurement for latency analysis

Recorded timestamps per `frame_id` are:

* `t_capture` — frame delivered from the camera backend into the software pipeline
* `t_estimate` — sun estimate produced
* `t_control` — control/setpoint stage completed
* `t_actuate` — actuator command issued by the software path

Each timestamp is associated with a `frame_id` so stage-to-stage latency can be reconstructed per processed frame.

---

## 4) Latency definitions

For each frame:

| Metric          | Definition               |
| --------------- | ------------------------ |
| **L_vision**    | `t_estimate - t_capture` |
| **L_control**   | `t_control - t_estimate` |
| **L_actuation** | `t_actuate - t_control`  |
| **L_total**     | `t_actuate - t_capture`  |

Important:

* **`L_total` is measured directly** from first to last timestamp
* it is **not treated only as a derived sum**
* this avoids hiding timing irregularities between stages

---

## 5) Measurement method

Latency monitoring is implemented through the repository’s `LatencyMonitor` logic and associated pipeline timestamp recording.

The measurement path:

* records stage timestamps by `frame_id`
* finalises latency once all required stage timestamps are available
* computes running count / mean / min / max
* derives jitter from measured extrema
* prunes stale in-flight entries to avoid unbounded internal growth
* reports summary statistics during shutdown/reporting

Important accuracy note:

* the runtime pipeline is **event-driven**
* worker threads **block while waiting for work**
* Linux application-level event handling may use **blocking `poll()`**
* the processing pipeline does **not** rely on busy-wait loops for normal operation

---

## 6) Architectural interpretation of the results

The measured software latency supports the following conclusions about the current implementation:

1. Per-stage computation is small relative to the end-to-end budget
2. Control computation overhead is negligible compared with vision processing
3. End-to-end software latency remains low across the measured sample
4. The queue-based event-driven design does not introduce large software-side delay
5. The staged architecture keeps the critical path short and understandable

The current architecture choices that support this include:

* callback-based frame delivery from the camera backend
* blocking waits on bounded queues
* separate control and actuation threads
* bounded queue capacity with freshest-data behaviour
* separation of vision, control, kinematics, and output responsibilities

---

## 7) Scope and limitations of the measurement

The measurement begins when a frame is delivered into the userspace pipeline (`t_capture`) and ends when the software issues the actuator command (`t_actuate`).

The following are **not** included in these figures:

* camera sensor exposure time
* kernel / driver buffering before userspace delivery
* physical servo motion time
* mechanical settling time of the platform
* external environmental disturbance effects

Therefore, the measured values represent:

**userspace software-pipeline latency**

This is the appropriate scope for evaluating the responsiveness of the software architecture itself, but it is **not the same thing as full physical closed-loop response time**.

---

## 8) Realtime evidence conclusion

For the measured 829-frame run:

* average software end-to-end latency is approximately **1.11 ms**
* worst-case measured software latency is approximately **5.34 ms**
* measured jitter is approximately **4.64 ms**

Within the scope of the userspace software pipeline, these measurements indicate:

* low end-to-end processing delay
* bounded observed timing variation
* substantial margin against the stated software latency targets

The results therefore support the claim that the current implementation provides **fast and stable software-side responsiveness** for the tracker pipeline on Raspberry Pi class Linux hardware.

---

## 9) Interpretation note

This document should be read as evidence of **software latency performance** for the current repository.

It does **not**, by itself, prove:

* full hard realtime guarantees
* kernel-level scheduling guarantees
* full actuator mechanical response time
* total camera-to-motion physical loop latency

It does provide quantitative evidence that the repository’s **userspace event-driven processing path** is fast, bounded, and suitable for the project’s software control pipeline.



