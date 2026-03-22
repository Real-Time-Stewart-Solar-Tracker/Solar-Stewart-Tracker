set(TEST_FRAMEWORK
    test_main.cpp
    test_common.hpp
)

add_executable(test_core
    ${TEST_FRAMEWORK}
    test_suntracker.cpp
    test_controller.cpp
    test_actuatormanager.cpp
    test_threadsafequeue.cpp
    test_kinematics.cpp
    test_latency_monitor.cpp
    test_trajectory_circular.cpp
    test_systemmanager_statemachine.cpp
)
target_include_directories(test_core PRIVATE include tests)
target_link_libraries(test_core PRIVATE solar_tracker_core)
solar_enable_coverage(test_core)

add_executable(test_pca9685
    ${TEST_FRAMEWORK}
    test_pca9685.cpp
    FakeI2CDevice.hpp
)
target_include_directories(test_pca9685 PRIVATE include tests)
target_link_libraries(test_pca9685 PRIVATE solar_tracker_core)
solar_enable_coverage(test_pca9685)

add_executable(test_servodriver
    ${TEST_FRAMEWORK}
    test_servodriver.cpp
    FakeI2CDevice.hpp
)
target_include_directories(test_servodriver PRIVATE include tests)
target_link_libraries(test_servodriver PRIVATE solar_tracker_core)
solar_enable_coverage(test_servodriver)

# Manual hardware executable (not part of CTest)
add_executable(servo_manual_smoketest
    servo_manual_smoketest.cpp
)
target_include_directories(servo_manual_smoketest PRIVATE include)
target_link_libraries(servo_manual_smoketest PRIVATE solar_tracker_core)
solar_enable_coverage(servo_manual_smoketest)

# Real camera hardware integration test (automated when explicitly enabled)
if(UNIX AND NOT APPLE AND SOLAR_HAVE_LIBCAMERA_FLAG)
    add_executable(test_libcamera_hw
        test_libcamera_hw.cpp
    )
    target_include_directories(test_libcamera_hw PRIVATE include tests)
    target_link_libraries(test_libcamera_hw PRIVATE solar_tracker_core)
    solar_enable_coverage(test_libcamera_hw)

    add_test(NAME LibcameraPublisher_hw_delivers_frames_with_public_contract
             COMMAND test_libcamera_hw)

    set_tests_properties(LibcameraPublisher_hw_delivers_frames_with_public_contract
        PROPERTIES
        LABELS "hw;camera;integration"
        SKIP_RETURN_CODE 77
    )
endif()

# Real Linux I2C + PCA9685 hardware integration test (automated when explicitly enabled)
if(UNIX AND NOT APPLE)
    add_executable(test_linux_i2c_hw
        test_linux_i2c_hw.cpp
    )
    target_include_directories(test_linux_i2c_hw PRIVATE include tests)
    target_link_libraries(test_linux_i2c_hw PRIVATE solar_tracker_core)
    solar_enable_coverage(test_linux_i2c_hw)

    add_test(NAME LinuxI2C_PCA9685_hw_init_and_write
             COMMAND test_linux_i2c_hw)

    set_tests_properties(LinuxI2C_PCA9685_hw_init_and_write
        PROPERTIES
        LABELS "hw;i2c;integration"
        SKIP_RETURN_CODE 77
    )
endif()

# --- Per-test registration (so CTest shows 1/N) ---

# --- SunTracker ---
add_test(NAME SunTracker_NoBrightPixels_ConfidenceZero
         COMMAND test_core --run SunTracker_NoBrightPixels_ConfidenceZero)
add_test(NAME SunTracker_BrightSpot_Gray8Packed_CentroidApproxCorrect
         COMMAND test_core --run SunTracker_BrightSpot_Gray8Packed_CentroidApproxCorrect)
add_test(NAME SunTracker_BrightSpot_Gray8PaddedStride_CentroidApproxCorrect
         COMMAND test_core --run SunTracker_BrightSpot_Gray8PaddedStride_CentroidApproxCorrect)
add_test(NAME SunTracker_BrightSpot_RGB888_CentroidApproxCorrect
         COMMAND test_core --run SunTracker_BrightSpot_RGB888_CentroidApproxCorrect)
add_test(NAME SunTracker_BrightSpot_BGR888_CentroidApproxCorrect
         COMMAND test_core --run SunTracker_BrightSpot_BGR888_CentroidApproxCorrect)
add_test(NAME SunTracker_InvalidBuffer_DoesNotEmitEstimate
         COMMAND test_core --run SunTracker_InvalidBuffer_DoesNotEmitEstimate)

# --- Controller ---
add_test(NAME Controller_LowConfidence_NoMotion
         COMMAND test_core --run Controller_LowConfidence_NoMotion)
add_test(NAME Controller_WithinDeadband_NoMotion
         COMMAND test_core --run Controller_WithinDeadband_NoMotion)
add_test(NAME Controller_OutsideDeadband_ProducesMotion
         COMMAND test_core --run Controller_OutsideDeadband_ProducesMotion)
add_test(NAME Controller_OutputClamped
         COMMAND test_core --run Controller_OutputClamped)

# --- ActuatorManager ---
add_test(NAME ActuatorManager_first_command_is_saturated_without_history_limit
         COMMAND test_core --run ActuatorManager_first_command_is_saturated_without_history_limit)
add_test(NAME ActuatorManager_subsequent_commands_are_rate_limited_per_channel
         COMMAND test_core --run ActuatorManager_subsequent_commands_are_rate_limited_per_channel)
add_test(NAME ActuatorManager_saturation_happens_before_rate_limit
         COMMAND test_core --run ActuatorManager_saturation_happens_before_rate_limit)
add_test(NAME ActuatorManager_runtime_like_large_max_step_effectively_disables_slew_limit
         COMMAND test_core --run ActuatorManager_runtime_like_large_max_step_effectively_disables_slew_limit)
add_test(NAME ActuatorManager_callback_can_reenter_onCommand_without_deadlock
         COMMAND test_core --run ActuatorManager_callback_can_reenter_onCommand_without_deadlock)

# --- ThreadSafeQueue ---
add_test(NAME ThreadSafeQueue_FIFO_basic
         COMMAND test_core --run ThreadSafeQueue_FIFO_basic)
add_test(NAME ThreadSafeQueue_bounded_push_strict_rejects_when_full
         COMMAND test_core --run ThreadSafeQueue_bounded_push_strict_rejects_when_full)
add_test(NAME ThreadSafeQueue_bounded_push_latest_drops_oldest
         COMMAND test_core --run ThreadSafeQueue_bounded_push_latest_drops_oldest)
add_test(NAME ThreadSafeQueue_wait_pop_blocks_then_wakes
         COMMAND test_core --run ThreadSafeQueue_wait_pop_blocks_then_wakes)
add_test(NAME ThreadSafeQueue_stop_unblocks_waiters_and_returns_nullopt_when_empty
         COMMAND test_core --run ThreadSafeQueue_stop_unblocks_waiters_and_returns_nullopt_when_empty)

# --- Kinematics3RRS ---
add_test(NAME Kinematics3RRS_outputs_in_range_and_integer_like
         COMMAND test_core --run Kinematics3RRS_outputs_in_range_and_integer_like)
add_test(NAME Kinematics3RRS_continuity_small_setpoint_changes_small_output_changes
         COMMAND test_core --run Kinematics3RRS_continuity_small_setpoint_changes_small_output_changes)
add_test(NAME Kinematics3RRS_invalid_geometry_is_surfaced_explicitly
         COMMAND test_core --run Kinematics3RRS_invalid_geometry_is_surfaced_explicitly)

# --- LatencyMonitor ---
add_test(NAME LatencyMonitor_accepts_ordered_timestamps_and_prints
         COMMAND test_core --run LatencyMonitor_accepts_ordered_timestamps_and_prints)
add_test(NAME LatencyMonitor_handles_out_of_order_calls_without_crashing
         COMMAND test_core --run LatencyMonitor_handles_out_of_order_calls_without_crashing)
add_test(NAME LatencyMonitor_prunes_inflight_frames_under_pressure_without_crashing
         COMMAND test_core --run LatencyMonitor_prunes_inflight_frames_under_pressure_without_crashing)

# --- Trajectory (integration-ish, still in test_core) ---
add_test(NAME Trajectory_CircularSetpoints_ProduceValidServoOutputs
         COMMAND test_core --run Trajectory_CircularSetpoints_ProduceValidServoOutputs)

# --- SystemManager state-machine ---
add_test(NAME SystemManager_start_to_searching_then_tracking_on_bright_frame
         COMMAND test_core --run SystemManager_start_to_searching_then_tracking_on_bright_frame)
add_test(NAME SystemManager_manual_mode_emits_commands
         COMMAND test_core --run SystemManager_manual_mode_emits_commands)
add_test(NAME SystemManager_start_with_null_camera_enters_fault_and_fails
         COMMAND test_core --run SystemManager_start_with_null_camera_enters_fault_and_fails)
add_test(NAME SystemManager_start_when_servo_driver_requires_missing_hardware_enters_fault
         COMMAND test_core --run SystemManager_start_when_servo_driver_requires_missing_hardware_enters_fault)
add_test(NAME SystemManager_start_when_camera_start_fails_enters_fault
         COMMAND test_core --run SystemManager_start_when_camera_start_fails_enters_fault)

# --- PCA9685 ---
add_test(NAME PCA9685_init_writes_mode_and_prescale
         COMMAND test_pca9685 --run PCA9685_init_writes_mode_and_prescale)
add_test(NAME PCA9685_set_pwm_writes_led_register_block
         COMMAND test_pca9685 --run PCA9685_set_pwm_writes_led_register_block)
add_test(NAME PCA9685_set_pulse_us_uses_frequency_conversion
         COMMAND test_pca9685 --run PCA9685_set_pulse_us_uses_frequency_conversion)

# --- ServoDriver ---
add_test(NAME ServoDriver_log_only_mode_starts_without_hardware
         COMMAND test_servodriver --run ServoDriver_log_only_mode_starts_without_hardware)
add_test(NAME ServoDriver_require_hardware_fails_fast_when_unavailable
         COMMAND test_servodriver --run ServoDriver_require_hardware_fails_fast_when_unavailable)
add_test(NAME ServoDriver_prefer_hardware_falls_back_to_log_only_when_unavailable
         COMMAND test_servodriver --run ServoDriver_prefer_hardware_falls_back_to_log_only_when_unavailable)
add_test(NAME ServoDriver_require_hardware_with_injected_pca_enters_hardware_mode
         COMMAND test_servodriver --run ServoDriver_require_hardware_with_injected_pca_enters_hardware_mode)
add_test(NAME ServoDriver_apply_while_stopped_writes_nothing
         COMMAND test_servodriver --run ServoDriver_apply_while_stopped_writes_nothing)
add_test(NAME ServoDriver_start_without_parking_does_not_write_servo_channels
         COMMAND test_servodriver --run ServoDriver_start_without_parking_does_not_write_servo_channels)
add_test(NAME ServoDriver_start_parks_to_neutral_when_enabled
         COMMAND test_servodriver --run ServoDriver_start_parks_to_neutral_when_enabled)
add_test(NAME ServoDriver_stop_parks_to_neutral_when_enabled
         COMMAND test_servodriver --run ServoDriver_stop_parks_to_neutral_when_enabled)
add_test(NAME ServoDriver_apply_clamps_and_writes_channels
         COMMAND test_servodriver --run ServoDriver_apply_clamps_and_writes_channels)
add_test(NAME ServoDriver_neutral_degree_maps_midway_between_low_and_high
         COMMAND test_servodriver --run ServoDriver_neutral_degree_maps_midway_between_low_and_high)