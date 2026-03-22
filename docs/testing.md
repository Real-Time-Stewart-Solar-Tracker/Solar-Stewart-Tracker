# Testing and Reliability Strategy

This document describes the **current** testing setup in the repository and is intended to match the code under `tests/` and `tests/CMakeLists.txt` exactly.

The testing approach is deliberately lightweight and repository-local:

- a small custom test runner is used instead of an external framework such as GoogleTest or Catch2
- automated tests are registered with **CTest**
- hardware-facing logic is tested with fakes where practical
- hardware-dependent checks are separated from normal software-only regression tests
- one manual hardware smoke test executable is provided outside CTest

This document describes what is **actually present now**, not an ideal or future test plan.

------------------------------------------------------------------------

## 1) Testing philosophy

The repository follows these practical testing rules:

- test core logic with deterministic inputs
- keep image-processing, control, kinematics, queueing, and orchestration testable without physical hardware
- use fake hardware support where that improves automation
- keep hardware-only checks separate from normal regression runs
- register named automated tests with CTest so failures are visible at individual test level
- avoid overstating what is proven by software-only tests

The suite is therefore a mix of:

- **unit-style tests**
- **small integration-style tests**
- **platform/hardware integration tests**
- **one manual hardware smoke test**

------------------------------------------------------------------------

## 2) Test framework used in this repository

The repository does **not** currently use GoogleTest, Catch2, or another external unit-test framework.

Instead, it uses a simple custom runner built from:

- `tests/test_main.cpp`
- `tests/test_common.hpp`

These provide:

- test registration via the `TEST(...)` macro
- assertion helpers such as `REQUIRE(...)` and `REQUIRE_NEAR(...)`
- execution of all tests in an executable
- listing tests with `--list`
- executing one exact test by name with `--run <name>`

### 2.1 Running the custom runner directly

Examples:

```bash
./build/tests/test_core
./build/tests/test_core --list
./build/tests/test_core --run Controller_OutputClamped
````

With no arguments, the runner executes **all tests compiled into that executable**.

This is important because some source-level tests exist in the executable even when they are **not individually exposed as separate CTest entries**.

---

## 3) Test support files

### 3.1 Shared test support

The shared support files are:

* `tests/test_main.cpp`
* `tests/test_common.hpp`

### 3.2 Fake hardware support

The repository includes:

* `tests/FakeI2CDevice.hpp`

This fake is used by automated tests for selected hardware-facing classes so those tests can run without real Linux I2C hardware.

---

## 4) Test executables built by `tests/CMakeLists.txt`

The current `tests/CMakeLists.txt` builds the following executables.

### 4.1 Automated software test executables

These are always built by the test CMake file:

* `test_core`
* `test_pca9685`
* `test_servodriver`

### 4.2 Automated hardware integration executables

These are conditionally built:

* `test_linux_i2c_hw`

  * built on `UNIX AND NOT APPLE`
* `test_libcamera_hw`

  * built on `UNIX AND NOT APPLE AND SOLAR_HAVE_LIBCAMERA_FLAG`

These are registered with CTest as hardware/integration tests and may **skip** when the required device or environment is not available.

### 4.3 Manual hardware executable

This is built but **not** registered with CTest:

* `servo_manual_smoketest`

This is a manual utility, not part of the normal automated regression suite.

---

## 5) CTest integration

The repository uses **CTest** and registers many tests individually with `add_test(...)`.

This means CTest can report failures by specific named case rather than only by executable.

### 5.1 Running CTest on Linux / single-config generators

```bash
ctest --test-dir build --output-on-failure
```

### 5.2 Running CTest on Windows / multi-config generators

```powershell
ctest --test-dir build -C Release --output-on-failure
```

### 5.3 Important note about counts

The total number of CTest entries depends on platform and optional hardware support:

* **42** base automated CTest entries are registered unconditionally
* **43** on `UNIX AND NOT APPLE` when `test_linux_i2c_hw` is included
* **44** on `UNIX AND NOT APPLE` when both:

  * `test_linux_i2c_hw` is included, and
  * `SOLAR_HAVE_LIBCAMERA_FLAG` enables `test_libcamera_hw`

So the exact visible CTest count is platform/configuration dependent.

---

## 6) Automated tests currently registered with CTest

This section lists the tests that are **individually registered** in `tests/CMakeLists.txt`.

## 6.1 `SunTracker` tests

Source file:

* `tests/test_suntracker.cpp`

Executable:

* `test_core`

CTest-registered cases:

* `SunTracker_NoBrightPixels_ConfidenceZero`
* `SunTracker_BrightSpot_Gray8Packed_CentroidApproxCorrect`
* `SunTracker_BrightSpot_Gray8PaddedStride_CentroidApproxCorrect`
* `SunTracker_BrightSpot_RGB888_CentroidApproxCorrect`
* `SunTracker_BrightSpot_BGR888_CentroidApproxCorrect`
* `SunTracker_InvalidBuffer_DoesNotEmitEstimate`

What these tests cover:

* no bright target present
* centroid estimation on packed grayscale data
* centroid estimation on padded-stride grayscale data
* centroid estimation on RGB input
* centroid estimation on BGR input
* rejection of invalid buffer cases

These are deterministic synthetic-frame tests and do not require a real camera.

---

## 6.2 `Controller` tests

Source file:

* `tests/test_controller.cpp`

Executable:

* `test_core`

CTest-registered cases:

* `Controller_LowConfidence_NoMotion`
* `Controller_WithinDeadband_NoMotion`
* `Controller_OutsideDeadband_ProducesMotion`
* `Controller_OutputClamped`

What these tests cover:

* confidence gating
* deadband behaviour
* non-zero response outside the deadband
* output saturation/clamping

---

## 6.3 `ActuatorManager` tests

Source file:

* `tests/test_actuatormanager.cpp`

Executable:

* `test_core`

CTest-registered cases:

* `ActuatorManager_first_command_is_saturated_without_history_limit`
* `ActuatorManager_subsequent_commands_are_rate_limited_per_channel`
* `ActuatorManager_saturation_happens_before_rate_limit`
* `ActuatorManager_runtime_like_large_max_step_effectively_disables_slew_limit`
* `ActuatorManager_callback_can_reenter_onCommand_without_deadlock`

What these tests cover:

* first-command saturation behaviour
* per-channel slew/rate limiting on later commands
* saturation ordering relative to rate limiting
* runtime-like large-step configuration behaviour
* callback re-entry without deadlock

---

## 6.4 `ThreadSafeQueue` tests

Source file:

* `tests/test_threadsafequeue.cpp`

Executable:

* `test_core`

CTest-registered cases:

* `ThreadSafeQueue_FIFO_basic`
* `ThreadSafeQueue_bounded_push_strict_rejects_when_full`
* `ThreadSafeQueue_bounded_push_latest_drops_oldest`
* `ThreadSafeQueue_wait_pop_blocks_then_wakes`
* `ThreadSafeQueue_stop_unblocks_waiters_and_returns_nullopt_when_empty`

What these tests cover:

* FIFO ordering
* strict bounded push rejection
* latest-value overwrite semantics
* blocking wake-up behaviour
* stop/unblock behaviour

---

## 6.5 `Kinematics3RRS` tests

Source file:

* `tests/test_kinematics.cpp`

Executable:

* `test_core`

CTest-registered cases:

* `Kinematics3RRS_outputs_in_range_and_integer_like`
* `Kinematics3RRS_continuity_small_setpoint_changes_small_output_changes`
* `Kinematics3RRS_invalid_geometry_is_surfaced_explicitly`

What these tests cover:

* output finiteness and expected servo range
* continuity under small setpoint changes
* explicit surfacing of invalid geometry conditions

---

## 6.6 `LatencyMonitor` tests registered with CTest

Source file:

* `tests/test_latency_monitor.cpp`

Executable:

* `test_core`

CTest-registered cases:

* `LatencyMonitor_accepts_ordered_timestamps_and_prints`
* `LatencyMonitor_handles_out_of_order_calls_without_crashing`
* `LatencyMonitor_prunes_inflight_frames_under_pressure_without_crashing`

What these tests cover:

* normal ordered timestamp flow
* resilience to out-of-order calls
* pruning behaviour under inflight pressure

### Important accuracy note

`tests/test_latency_monitor.cpp` contains **additional source-level tests** beyond the three individually registered CTest entries. Those additional tests are compiled into `test_core` and run when `test_core` is executed directly without `--run`, but they are **not** exposed as separate named CTest entries.

Those additional source-level tests are:

* `LatencyMonitor_writes_raw_csv_for_finalized_frames`
* `LatencyMonitor_invokes_observer_when_frame_is_finalized`
* `LatencyMonitor_appends_rows_when_truncate_disabled`

So:

* **CTest registration for LatencyMonitor:** 3 named entries
* **Source-level LatencyMonitor tests compiled into `test_core`:** 6 total

---

## 6.7 Trajectory / kinematics integration-style test

Source file:

* `tests/test_trajectory_circular.cpp`

Executable:

* `test_core`

CTest-registered case:

* `Trajectory_CircularSetpoints_ProduceValidServoOutputs`

What this test covers:

* repeated circular setpoints producing finite, in-range servo outputs

This is integration-like because it exercises repeated setpoint-to-command generation rather than a single isolated calculation.

---

## 6.8 `SystemManager` state-machine tests registered with CTest

Source file:

* `tests/test_systemmanager_statemachine.cpp`

Executable:

* `test_core`

CTest-registered cases:

* `SystemManager_start_to_searching_then_tracking_on_bright_frame`
* `SystemManager_manual_mode_emits_commands`
* `SystemManager_start_with_null_camera_enters_fault_and_fails`
* `SystemManager_start_when_servo_driver_requires_missing_hardware_enters_fault`
* `SystemManager_start_when_camera_start_fails_enters_fault`

What these tests cover:

* startup progression into `SEARCHING` and then `TRACKING`
* manual mode command emission
* startup failure with null camera
* startup failure when hardware is required but unavailable
* startup failure when camera start fails

### Important accuracy note

`tests/test_systemmanager_statemachine.cpp` contains one additional source-level test that is compiled into `test_core` but is **not individually registered as a named CTest case**:

* `SystemManager_manual_mode_uses_nonzero_synthetic_frame_ids`

So:

* **CTest registration for SystemManager:** 5 named entries
* **Source-level SystemManager tests compiled into `test_core`:** 6 total

---

## 6.9 `PCA9685` tests

Source file:

* `tests/test_pca9685.cpp`

Executable:

* `test_pca9685`

CTest-registered cases:

* `PCA9685_init_writes_mode_and_prescale`
* `PCA9685_set_pwm_writes_led_register_block`
* `PCA9685_set_pulse_us_uses_frequency_conversion`

What these tests cover:

* initialization register writes
* PWM register block writes
* pulse-width conversion logic

These tests use the fake I2C device rather than real hardware.

---

## 6.10 `ServoDriver` tests

Source file:

* `tests/test_servodriver.cpp`

Executable:

* `test_servodriver`

CTest-registered cases:

* `ServoDriver_log_only_mode_starts_without_hardware`
* `ServoDriver_require_hardware_fails_fast_when_unavailable`
* `ServoDriver_prefer_hardware_falls_back_to_log_only_when_unavailable`
* `ServoDriver_require_hardware_with_injected_pca_enters_hardware_mode`
* `ServoDriver_apply_while_stopped_writes_nothing`
* `ServoDriver_start_without_parking_does_not_write_servo_channels`
* `ServoDriver_start_parks_to_neutral_when_enabled`
* `ServoDriver_stop_parks_to_neutral_when_enabled`
* `ServoDriver_apply_clamps_and_writes_channels`
* `ServoDriver_neutral_degree_maps_midway_between_low_and_high`

What these tests cover:

* startup policy behaviour across log-only / prefer-hardware / require-hardware modes
* fast failure when hardware is required but unavailable
* injected PCA path entering hardware mode
* no writes while stopped
* parking behaviour on start and stop
* clamping and output writes
* neutral-angle mapping

---

## 7) Hardware integration tests

These are automated tests but are platform- and hardware-dependent.

## 7.1 Linux I2C / PCA9685 hardware test

Source file:

* `tests/test_linux_i2c_hw.cpp`

Executable:

* `test_linux_i2c_hw`

CTest-registered case:

* `LinuxI2C_PCA9685_hw_init_and_write`

Build condition:

* `UNIX AND NOT APPLE`

CTest properties:

* labels: `hw;i2c;integration`
* `SKIP_RETURN_CODE 77`

Meaning:

* this is intended for real Linux I2C + PCA9685 hardware
* it may skip cleanly when the required hardware or environment is not available

---

## 7.2 libcamera hardware test

Source file:

* `tests/test_libcamera_hw.cpp`

Executable:

* `test_libcamera_hw`

CTest-registered case:

* `LibcameraPublisher_hw_delivers_frames_with_public_contract`

Build condition:

* `UNIX AND NOT APPLE AND SOLAR_HAVE_LIBCAMERA_FLAG`

CTest properties:

* labels: `hw;camera;integration`
* `SKIP_RETURN_CODE 77`

Meaning:

* this is intended for real libcamera-backed camera hardware
* it may skip cleanly when the camera environment is not available
* it is only built when libcamera support is available in the current configuration

---

## 8) Manual hardware smoke test

Source file:

* `tests/servo_manual_smoketest.cpp`

Executable:

* `servo_manual_smoketest`

This executable is **not** registered with CTest.

Its role is:

* manual checking of servo-path behaviour on suitable hardware
* quick smoke testing outside the normal automated regression suite

It should be described as a **manual hardware utility**, not as part of the normal automated test count.

---

## 9) Summary of what is and is not proven

The current automated software-oriented suite provides good evidence for:

* sun-tracking frame processing logic
* controller gating, deadband, and clamping
* bounded queue semantics and blocking wake-up behaviour
* actuator saturation and slew limiting behaviour
* kinematics output sanity and explicit invalid-geometry surfacing
* latency-monitor robustness for several important software-side behaviours
* system-manager startup/manual/fault-path behaviour at software level
* PCA9685 register-write logic via fake I2C
* ServoDriver startup-policy and mapping behaviour

The current suite does **not**, by itself, prove:

* full physical motion correctness of the real mechanism
* end-to-end hardware reliability under long real-world runs
* hard real-time guarantees
* complete runtime handling of every external hardware failure mode

So the tests provide strong **software-side evidence**, plus limited hardware-integration evidence where supported, but they should not be overstated as proof of complete physical-system validation.

---

## 10) Practical commands

### 10.1 Run the main automated suite directly

```bash
./build/tests/test_core
./build/tests/test_pca9685
./build/tests/test_servodriver
```

### 10.2 List test names in an executable

```bash
./build/tests/test_core --list
./build/tests/test_pca9685 --list
./build/tests/test_servodriver --list
```

### 10.3 Run one exact named test

```bash
./build/tests/test_core --run ThreadSafeQueue_wait_pop_blocks_then_wakes
./build/tests/test_pca9685 --run PCA9685_init_writes_mode_and_prescale
./build/tests/test_servodriver --run ServoDriver_apply_clamps_and_writes_channels
```

### 10.4 Run all CTest-registered checks

Linux / single-config:

```bash
ctest --test-dir build --output-on-failure
```

Windows / multi-config:

```powershell
ctest --test-dir build -C Release --output-on-failure
```

---

## 11) Final accuracy notes

This document is intended to match:

* `tests/CMakeLists.txt`
* the current test source files under `tests/`

If the test inventory changes, this document must be updated at the same time.

In particular, keep these distinctions clear:

* **source-level tests compiled into an executable**
* **named CTest entries individually registered with `add_test(...)`**
* **conditional hardware tests**
* **manual hardware utilities not included in CTest**
