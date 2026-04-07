# Testing and Reliability Strategy

This document describes the testing setup implemented in the repository source. Tests are co-located with their modules under `src/*/tests/` and wired into CTest via each module's own `CMakeLists.txt`.

The test approach is intentionally lightweight:

- a small custom runner is used instead of an external framework
- named checks are registered with CTest via each module's `CMakeLists.txt`
- core logic is exercised with deterministic inputs
- hardware-adjacent tests are separated behind an explicit build option
- fake I2C support is used where that improves automation

---

## 1. Testing Approach

The test strategy is based on the following principles:

- keep core logic testable without physical hardware
- isolate vision, control, queueing, kinematics, and system state as deterministic software tests
- make hardware-adjacent tests opt-in via `-DSOLAR_ENABLE_HW_TESTS=ON`
- expose individual named cases through CTest
---

## 2. Test Framework

The repository does not use GoogleTest, Catch2, or another external framework. Instead it uses a small custom framework at `src/tests/support/`:

| File | Purpose |
|---|---|
| `src/tests/support/test_main.cpp` | Runner entry point |
| `src/tests/support/test_common.hpp` | `TEST_CASE(...)` registration and `REQUIRE(...)` assertion helper |
| `src/tests/support/FakeI2CDevice.hpp` | Fake I2C device for hardware-adjacent software tests |

Each test executable is built by linking `test_main.cpp` together with one or more test source files.

### Running an executable directly

```bash
# Run all tests compiled into an executable
./build/src/control/tests/test_control
```

With no arguments an executable runs every test compiled into it. CTest remains the primary interface for selecting and running tests. Individual test binaries can also be run directly from the build tree, but their command-line interfaces are not assumed to be uniform across all executables.

---

## 3. Test Executables and Module Structure

Tests are co-located with source. Each module's `CMakeLists.txt` conditionally includes its `tests/` subdirectory.

### Always-built executables (`SOLAR_ENABLE_TESTS=ON`)

| Executable | Module | Source path |
|---|---|---|
| `test_common_core` | `src/common` | `src/common/tests/` |
| `test_vision` | `src/vision` | `src/vision/tests/` |
| `test_control` | `src/control` | `src/control/tests/` |
| `test_actuators` | `src/actuators` | `src/actuators/tests/` |
| `test_system` | `src/system` | `src/system/tests/` |

### Hardware-adjacent executables (`SOLAR_ENABLE_TESTS=ON` and `SOLAR_ENABLE_HW_TESTS=ON`)

| Executable | Module | Source path |
|---|---|---|
| `test_pca9685` | `src/actuators` | `src/actuators/tests/` |
| `test_servodriver` | `src/actuators` | `src/actuators/tests/` |
| `test_mpu6050_publisher` | `src/sensors` | `src/sensors/tests/` |
| `test_linux_i2c_hw` | `src/hal` | `src/hal/tests/` |

---

## 4. CTest Integration

CTest is used and named cases are registered with `add_test(...)` in each module's `CMakeLists.txt`, allowing failures to be reported at individual test-case level.

```bash
# Linux / single-config generators
ctest --test-dir build --output-on-failure

# Windows / multi-config generators
ctest --test-dir build -C Release --output-on-failure
```

### Test count

| Condition | CTest entries |
|---|---:|
| `SOLAR_ENABLE_TESTS=ON` only | 55 |
| `SOLAR_ENABLE_TESTS=ON` and `SOLAR_ENABLE_HW_TESTS=ON` | 75 |

---

## 5. CTest Cases by Module

### 5.1 SunTracker

Executable: `test_vision` — Source: `src/vision/tests/test_suntracker.cpp`

| CTest Case | What It Covers |
|---|---|
| `SunTracker_DetectsSingleBrightPixel` | Bright target detection |
| `SunTracker_NoBrightPixels_ProducesZeroConfidence` | Zero-confidence when no bright region exists |
| `SunTracker_WeightedCentroid_IsCorrect` | Weighted centroid on deterministic synthetic input |

### 5.2 Controller

Executable: `test_control` — Source: `src/control/tests/test_controller.cpp`

| CTest Case | What It Covers |
|---|---|
| `Controller_LowConfidence_NoMotion` | Confidence gating |
| `Controller_WithinDeadband_NoMotion` | Deadband behaviour |
| `Controller_OutsideDeadband_ProducesMotion` | Non-zero control response |
| `Controller_OutputClamped` | Output saturation/clamping |

### 5.3 Kinematics3RRS

Executable: `test_control` — Source: `src/control/tests/test_kinematics.cpp`

| CTest Case | What It Covers |
|---|---|
| `Kinematics3RRS_outputs_in_range_for_moderate_setpoint` | Output sanity and servo range |
| `Kinematics3RRS_neutral_setpoint_produces_near_neutral_angles` | Neutral symmetry |
| `Kinematics3RRS_frame_id_forwarded_through_callback` | Frame ID propagation |
| `Kinematics3RRS_small_setpoint_changes_produce_small_output_changes` | Continuity for small input changes |
| `Kinematics3RRS_zero_setpoint_after_valid_does_not_trigger_fallback` | Zero setpoint does not degrade |
| `Kinematics3RRS_large_reachable_setpoint_stays_in_range` | Workspace boundary check |
| `Kinematics3RRS_invalid_geometry_surfaces_error_status` | Invalid-geometry error surfacing |
| `Kinematics3RRS_first_invalid_then_second_invalid_stays_in_range` | Consecutive invalid configs safe |
| `Kinematics3RRS_good_then_bad_geometry_falls_back_to_last_valid` | Fallback after previously valid state |

### 5.4 ManualInputMapper

Executable: `test_control` — Source: `src/control/tests/test_manual_input_mapper.cpp`

| CTest Case | What It Covers |
|---|---|
| `ManualInputMapper_CentreVoltage_MapsToZeroCommand` | Centre mapping |
| `ManualInputMapper_LowAndHighVoltages_MapToMinusAndPlusOne` | Full-range mapping |
| `ManualInputMapper_InversionFlags_FlipDirections` | Inversion behaviour |
| `ManualInputMapper_Deadband_ZerosSmallInputsAndRescalesBeyondDeadband` | Deadband behaviour |
| `ManualInputMapper_OutOfRangeVoltages_AreClampedSafely` | Safe clamping of out-of-range inputs |

### 5.5 ImuFeedbackMapper

Executable: `test_control` — Source: `src/control/tests/test_imu_feedback_mapper.cpp`

| CTest Case | What It Covers |
|---|---|
| `ImuFeedbackMapper_ZeroTilt_ProducesZeroCorrection` | Zero-tilt response |
| `ImuFeedbackMapper_PositiveTilt_ProducesNegativeCorrection` | Correction sign |
| `ImuFeedbackMapper_NegativeTilt_ProducesPositiveCorrection` | Symmetry across tilt direction |
| `ImuFeedbackMapper_RuntimeGainChange_TakesEffectImmediately` | Immediate effect of runtime gain changes |

### 5.6 ImuTiltEstimator

Executable: `test_control` — Source: `src/control/tests/test_imu_tilt_estimator.cpp`

| CTest Case | What It Covers |
|---|---|
| `ImuTiltEstimator_InvalidSample_ReturnsZero` | Invalid sample handling |
| `ImuTiltEstimator_LevelGravity_ReturnsZeroTilt` | Level orientation |
| `ImuTiltEstimator_PositiveAx_WithGravity_ReturnsPositiveTilt` | Positive tilt reconstruction |
| `ImuTiltEstimator_NegativeAx_WithGravity_ReturnsNegativeTilt` | Negative tilt reconstruction |
| `ImuTiltEstimator_DegenerateDenominator_ReturnsZero` | Degenerate denominator protection |

### 5.7 ActuatorManager

Executable: `test_actuators` — Source: `src/actuators/tests/test_actuatormanager.cpp`

| CTest Case | What It Covers |
|---|---|
| `ActuatorManager_first_command_is_saturated_without_history_limit` | First-command behaviour |
| `ActuatorManager_subsequent_commands_are_rate_limited_per_channel` | Per-channel rate limiting |
| `ActuatorManager_saturation_happens_before_history_update` | Saturation ordering |
| `ActuatorManager_resetHistory_disables_slew_limit_for_next_command` | History reset behaviour |
| `ActuatorManager_zero_or_negative_step_holds_previous_value` | Behaviour when allowed step is zero or negative |

### 5.8 ThreadSafeQueue

Executable: `test_common_core` — Source: `src/common/tests/test_threadsafequeue.cpp`

| CTest Case | What It Covers |
|---|---|
| `ThreadSafeQueue_FIFO_basic` | FIFO ordering |
| `ThreadSafeQueue_bounded_push_strict_rejects_when_full` | Strict bounded insertion |
| `ThreadSafeQueue_bounded_push_latest_drops_oldest` | Freshest-value overwrite semantics |
| `ThreadSafeQueue_wait_pop_blocks_then_wakes` | Blocking wake-up behaviour |
| `ThreadSafeQueue_stop_unblocks_waiters_and_returns_nullopt_when_empty` | Stop/unblock behaviour |

### 5.9 LatencyMonitor

Executable: `test_common_core` — Source: `src/common/tests/test_latency_monitor.cpp`

| CTest Case | What It Covers |
|---|---|
| `LatencyMonitor_accepts_ordered_timestamps_and_prints` | Ordered timestamp handling |
| `LatencyMonitor_handles_out_of_order_calls_without_crashing` | Out-of-order robustness |
| `LatencyMonitor_prunes_inflight_frames_under_pressure_without_crashing` | In-flight frame pruning under pressure |

> **Note:** A fourth test (`LatencyMonitor_reports_jitter_over_multiple_records`) is compiled into `test_common_core` but is not individually registered as a CTest entry.

### 5.10 SystemManager

Executable: `test_system`  
Sources: `src/system/tests/test_systemmanager_statemachine.cpp`, `src/system/tests/test_systemmanager_imu_shadow.cpp`

| CTest Case | What It Covers |
|---|---|
| `SystemManager_start_to_searching_then_tracking_on_bright_frame` | Startup progression and tracking-state transition |
| `SystemManager_manual_mode_emits_commands` | Manual mode command emission |
| `SystemManager_start_with_null_camera_enters_fault_and_fails` | Fault entry on null camera |
| `SystemManager_start_when_camera_start_fails_enters_fault` | Fault entry on camera start failure |
| `SystemManager_ImuShadow_StartsWithoutImuBackend` | IMU shadow-mode startup without backend |
| `SystemManager_ImuDisabled_StartsNormally` | IMU disabled mode starts normally |
| `SystemManager_ImuShadow_ManualModeTransitions` | Shadow mode does not block manual transitions |
| `SystemManager_ImuShadow_NullCameraEntersFault` | Null camera enters FAULT |
| `SystemManager_ImuShadow_CameraStartFailureEntersFault` | Camera start failure enters FAULT |
| `SystemManager_ImuShadow_StopFromIdleIsIdempotent` | Stop from IDLE is a safe no-op |
| `SystemManager_ImuLive_StartsNormallyWhenNoImuBackendConfigured` | Live mode degrades gracefully without IMU |
| `SystemManager_ImuShadow_StateObserverReceivesTransitions` | State observer receives startup transitions |

---

## 6. Hardware-Adjacent Tests (`SOLAR_ENABLE_HW_TESTS=ON`)

### 6.1 PCA9685

Executable: `test_pca9685` — Source: `src/actuators/tests/test_pca9685.cpp`

| CTest Case | What It Covers |
|---|---|
| `PCA9685_default_constructor_is_not_started` | Safe default construction |
| `PCA9685_set_pulse_us_fails_before_start` | Pre-start failure handling |
| `PCA9685_invalid_channel_rejected_even_if_not_started` | Channel validation |
| `PCA9685_stop_is_safe_when_not_started` | Safe stop before startup |

### 6.2 ServoDriver

Executable: `test_servodriver` — Source: `src/actuators/tests/test_servodriver.cpp`

| CTest Case | What It Covers |
|---|---|
| `ServoDriver_log_only_mode_starts_without_hardware` | Startup policy across hardware modes |
| `ServoDriver_require_hardware_fails_fast_when_unavailable` | Fast failure on missing required hardware |
| `ServoDriver_prefer_hardware_falls_back_to_log_only_when_unavailable` | Fallback to log-only mode |
| `ServoDriver_require_hardware_with_injected_pca_enters_hardware_mode` | Injected PCA path |
| `ServoDriver_apply_while_stopped_writes_nothing` | No writes while stopped |
| `ServoDriver_start_parks_to_neutral_when_enabled` | Parking on start |
| `ServoDriver_stop_parks_to_neutral_when_enabled` | Parking on stop |
| `ServoDriver_apply_clamps_and_writes_channels` | Command clamping and channel writes |
| `ServoDriver_inverted_channel_maps_correctly` | Inverted channel mapping |

### 6.3 Mpu6050Publisher

Executable: `test_mpu6050_publisher` — Source: `src/sensors/tests/test_mpu6050_publisher.cpp`

| CTest Case | What It Covers |
|---|---|
| `Mpu6050Publisher_StartFailsWithoutCallback` | Callback pre-condition (software) |
| `Mpu6050Publisher_StartFailsIfWhoAmIDoesNotMatch` | WHO_AM_I identity check (software) |
| `Mpu6050Publisher_InitialisesExpectedRegisters` | Six init registers written before GPIO (software) |
| `Mpu6050Publisher_StartSucceedsAndStopClosesDevice` | Full start/stop lifecycle (hardware, GPIO 27) |
| `Mpu6050Publisher_DoubleStartReturnsFalse` | Double-start guard (hardware, GPIO 27) |
| `Mpu6050Publisher_StartupDiscardPreventsEarlyCallbacks` | Startup discard window (hardware, GPIO 27) |

### 6.4 Linux I2C Hardware Check

Executable: `test_linux_i2c_hw` — Source: `src/hal/tests/test_linux_i2c_hw.cpp`

| CTest Case | Labels | Notes |
|---|---|---|
| `LinuxI2C_PCA9685_hw_init_and_write` | `hw;i2c;integration` | Skips cleanly (`SKIP_RETURN_CODE 77`) when hardware is unavailable |

---

## 7. What the Tests Demonstrate

- bright-target detection and centroid logic
- controller gating, deadband, and output clamping
- bounded queue semantics and blocking wake-up behaviour
- actuator command limiting and history behaviour
- kinematics validity, continuity, and fallback behaviour
- latency-monitor robustness
- system-manager startup, manual-mode, and selected fault paths
- manual input mapping
- IMU tilt estimation and IMU feedback mapping
- selected startup and mapping behaviour of PCA9685, ServoDriver, and MPU6050 publisher (when hardware-adjacent tests are enabled)


---

## 8. Practical Commands

```bash
# Run all CTest-registered checks
ctest --test-dir build --output-on-failure -LE hw

# Run hardware tests (requires hardware connected)
SOLAR_RUN_I2C_HW_TESTS=1 ctest --test-dir build --output-on-failure

# Run one CTest entry by name
ctest --test-dir build --output-on-failure -R Controller_OutputClamped

# Run individual module executables directly
./build/src/vision/tests/test_vision
./build/src/control/tests/test_control
./build/src/common/tests/test_common_core
./build/src/actuators/tests/test_actuators
./build/src/system/tests/test_system

# Enable and build hardware-adjacent tests
cmake -S . -B build -G Ninja -DSOLAR_ENABLE_TESTS=ON -DSOLAR_ENABLE_HW_TESTS=ON -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j

# Run hardware-adjacent executables directly
./build/src/actuators/tests/test_pca9685
./build/src/actuators/tests/test_servodriver
./build/src/sensors/tests/test_mpu6050_publisher
```


