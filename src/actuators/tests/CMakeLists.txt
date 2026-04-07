set(SOLAR_TEST_SUPPORT_DIR ${CMAKE_SOURCE_DIR}/src/tests/support)

add_executable(test_actuators
    ${SOLAR_TEST_SUPPORT_DIR}/test_main.cpp
    test_actuatormanager.cpp
)

target_include_directories(test_actuators PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(test_actuators PRIVATE solar_actuators)
solar_enable_coverage(test_actuators)

add_test(NAME ActuatorManager_first_command_is_saturated_without_history_limit COMMAND test_actuators --run ActuatorManager_first_command_is_saturated_without_history_limit)
add_test(NAME ActuatorManager_subsequent_commands_are_rate_limited_per_channel COMMAND test_actuators --run ActuatorManager_subsequent_commands_are_rate_limited_per_channel)
add_test(NAME ActuatorManager_saturation_happens_before_history_update COMMAND test_actuators --run ActuatorManager_saturation_happens_before_history_update)
add_test(NAME ActuatorManager_resetHistory_disables_slew_limit_for_next_command COMMAND test_actuators --run ActuatorManager_resetHistory_disables_slew_limit_for_next_command)
add_test(NAME ActuatorManager_zero_or_negative_step_holds_previous_value COMMAND test_actuators --run ActuatorManager_zero_or_negative_step_holds_previous_value)

if(SOLAR_ENABLE_HW_TESTS)
    add_executable(test_pca9685
        ${SOLAR_TEST_SUPPORT_DIR}/test_main.cpp
        test_pca9685.cpp
    )
    target_include_directories(test_pca9685 PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR})
    target_link_libraries(test_pca9685 PRIVATE solar_actuators)
    solar_enable_coverage(test_pca9685)

    add_executable(test_servodriver
        ${SOLAR_TEST_SUPPORT_DIR}/test_main.cpp
        test_servodriver.cpp
    )
    target_include_directories(test_servodriver PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR})
    target_link_libraries(test_servodriver PRIVATE solar_actuators solar_common)
    solar_enable_coverage(test_servodriver)

    add_test(NAME PCA9685_default_constructor_is_not_started COMMAND test_pca9685 --run PCA9685_default_constructor_is_not_started)
    add_test(NAME PCA9685_set_pulse_us_fails_before_start COMMAND test_pca9685 --run PCA9685_set_pulse_us_fails_before_start)
    add_test(NAME PCA9685_invalid_channel_rejected_even_if_not_started COMMAND test_pca9685 --run PCA9685_invalid_channel_rejected_even_if_not_started)
    add_test(NAME PCA9685_stop_is_safe_when_not_started COMMAND test_pca9685 --run PCA9685_stop_is_safe_when_not_started)
    set_tests_properties(
        PCA9685_default_constructor_is_not_started
        PCA9685_set_pulse_us_fails_before_start
        PCA9685_invalid_channel_rejected_even_if_not_started
        PCA9685_stop_is_safe_when_not_started
        PROPERTIES LABELS "hw;contract"
    )

    add_test(NAME ServoDriver_log_only_mode_starts_without_hardware COMMAND test_servodriver --run ServoDriver_log_only_mode_starts_without_hardware)
    add_test(NAME ServoDriver_require_hardware_fails_fast_when_unavailable COMMAND test_servodriver --run ServoDriver_require_hardware_fails_fast_when_unavailable)
    add_test(NAME ServoDriver_prefer_hardware_falls_back_to_log_only_when_unavailable COMMAND test_servodriver --run ServoDriver_prefer_hardware_falls_back_to_log_only_when_unavailable)
    add_test(NAME ServoDriver_require_hardware_with_injected_pca_enters_hardware_mode COMMAND test_servodriver --run ServoDriver_require_hardware_with_injected_pca_enters_hardware_mode)
    add_test(NAME ServoDriver_apply_while_stopped_writes_nothing COMMAND test_servodriver --run ServoDriver_apply_while_stopped_writes_nothing)
    add_test(NAME ServoDriver_start_parks_to_neutral_when_enabled COMMAND test_servodriver --run ServoDriver_start_parks_to_neutral_when_enabled)
    add_test(NAME ServoDriver_stop_parks_to_neutral_when_enabled COMMAND test_servodriver --run ServoDriver_stop_parks_to_neutral_when_enabled)
    add_test(NAME ServoDriver_apply_clamps_and_writes_channels COMMAND test_servodriver --run ServoDriver_apply_clamps_and_writes_channels)
    add_test(NAME ServoDriver_inverted_channel_maps_correctly COMMAND test_servodriver --run ServoDriver_inverted_channel_maps_correctly)
    set_tests_properties(
        ServoDriver_log_only_mode_starts_without_hardware
        ServoDriver_require_hardware_fails_fast_when_unavailable
        ServoDriver_prefer_hardware_falls_back_to_log_only_when_unavailable
        ServoDriver_require_hardware_with_injected_pca_enters_hardware_mode
        ServoDriver_apply_while_stopped_writes_nothing
        ServoDriver_start_parks_to_neutral_when_enabled
        ServoDriver_stop_parks_to_neutral_when_enabled
        ServoDriver_apply_clamps_and_writes_channels
        ServoDriver_inverted_channel_maps_correctly
        PROPERTIES LABELS "hw;contract"
    )
endif()
