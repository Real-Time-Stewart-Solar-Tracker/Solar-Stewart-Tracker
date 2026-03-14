
set(TEST_FRAMEWORK
    test_main.cpp
    test_common.hpp
)

add_executable(test_core
    ${TEST_FRAMEWORK}
    test_suntracker.cpp
    test_controller.cpp
    test_threadsafequeue.cpp
    test_kinematics.cpp
    test_latency_monitor.cpp
    test_trajectory_circular.cpp
    test_systemmanager_statemachine.cpp
)
target_include_directories(test_core PRIVATE include tests)
target_link_libraries(test_core PRIVATE solar_tracker_core)

add_executable(test_pca9685
    ${TEST_FRAMEWORK}
    test_pca9685.cpp
    FakeI2CDevice.hpp
)
target_include_directories(test_pca9685 PRIVATE include tests)
target_link_libraries(test_pca9685 PRIVATE solar_tracker_core)

add_executable(test_servodriver
    ${TEST_FRAMEWORK}
    test_servodriver.cpp
    FakeI2CDevice.hpp
)
target_include_directories(test_servodriver PRIVATE include tests)
target_link_libraries(test_servodriver PRIVATE solar_tracker_core)

# Hardware/manual executable (do NOT add to ctest by default)
add_executable(servo_test
    servo_driver_test.cpp
)
target_include_directories(servo_test PRIVATE include)
target_link_libraries(servo_test PRIVATE solar_tracker_core)

# --- Per-test registration (so CTest shows 1/N) ---
add_test(NAME SunTracker_NoBrightPixels_ConfidenceZero
         COMMAND test_core --run SunTracker_NoBrightPixels_ConfidenceZero)
add_test(NAME SunTracker_BrightSpot_CentroidApproxCorrect
         COMMAND test_core --run SunTracker_BrightSpot_CentroidApproxCorrect)
add_test(NAME SunTracker_BrightSpot_HigherAreaHigherConfidence
         COMMAND test_core --run SunTracker_BrightSpot_HigherAreaHigherConfidence)

add_test(NAME Controller_LowConfidence_NoMotion
         COMMAND test_core --run Controller_LowConfidence_NoMotion)
add_test(NAME Controller_WithinDeadband_NoMotion
         COMMAND test_core --run Controller_WithinDeadband_NoMotion)
add_test(NAME Controller_OutsideDeadband_ProducesMotion
         COMMAND test_core --run Controller_OutsideDeadband_ProducesMotion)
add_test(NAME Controller_OutputClamped
         COMMAND test_core --run Controller_OutputClamped)

# ---  ThreadSafeQueue ---
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
add_test(NAME Kinematics3RRS_invalid_geometry_falls_back_to_last_valid
         COMMAND test_core --run Kinematics3RRS_invalid_geometry_falls_back_to_last_valid)

# --- LatencyMonitor ---
add_test(NAME LatencyMonitor_accepts_ordered_timestamps_and_prints
         COMMAND test_core --run LatencyMonitor_accepts_ordered_timestamps_and_prints)
add_test(NAME LatencyMonitor_handles_out_of_order_calls_without_crashing
         COMMAND test_core --run LatencyMonitor_handles_out_of_order_calls_without_crashing)
add_test(NAME LatencyMonitor_prunes_inflight_frames_under_pressure_without_crashing
         COMMAND test_core --run LatencyMonitor_prunes_inflight_frames_under_pressure_without_crashing)

# ---  Trajectory (integration-ish, still unit-test executable) ---
add_test(NAME Trajectory_CircularSetpoints_ProduceValidServoOutputs
         COMMAND test_core --run Trajectory_CircularSetpoints_ProduceValidServoOutputs)

# --- SystemManager state-machine ---
add_test(NAME SystemManager_start_to_searching_then_tracking_on_bright_frame
         COMMAND test_core --run SystemManager_start_to_searching_then_tracking_on_bright_frame)
add_test(NAME SystemManager_manual_mode_emits_commands
         COMMAND test_core --run SystemManager_manual_mode_emits_commands)

add_test(NAME PCA9685_init_writes_mode_and_prescale
         COMMAND test_pca9685 --run PCA9685_init_writes_mode_and_prescale)
add_test(NAME PCA9685_set_pwm_writes_led_register_block
         COMMAND test_pca9685 --run PCA9685_set_pwm_writes_led_register_block)
add_test(NAME PCA9685_set_pulse_us_uses_frequency_conversion
         COMMAND test_pca9685 --run PCA9685_set_pulse_us_uses_frequency_conversion)

add_test(NAME ServoDriver_start_parks_to_neutral_when_enabled
         COMMAND test_servodriver --run ServoDriver_start_parks_to_neutral_when_enabled)
add_test(NAME ServoDriver_apply_clamps_and_writes_channels
         COMMAND test_servodriver --run ServoDriver_apply_clamps_and_writes_channels)