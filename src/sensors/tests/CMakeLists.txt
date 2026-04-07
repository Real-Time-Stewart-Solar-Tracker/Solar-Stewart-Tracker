set(SOLAR_TEST_SUPPORT_DIR ${CMAKE_SOURCE_DIR}/src/tests/support)

add_executable(test_mpu6050_publisher
    ${SOLAR_TEST_SUPPORT_DIR}/test_main.cpp
    test_mpu6050_publisher.cpp
)

target_include_directories(test_mpu6050_publisher PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(test_mpu6050_publisher PRIVATE solar_sensors)
solar_enable_coverage(test_mpu6050_publisher)

add_test(NAME Mpu6050Publisher_StartFailsWithoutCallback
    COMMAND test_mpu6050_publisher --run Mpu6050Publisher_StartFailsWithoutCallback)
add_test(NAME Mpu6050Publisher_StartFailsIfWhoAmIDoesNotMatch
    COMMAND test_mpu6050_publisher --run Mpu6050Publisher_StartFailsIfWhoAmIDoesNotMatch)
add_test(NAME Mpu6050Publisher_StartSucceedsAndStopClosesDevice
    COMMAND test_mpu6050_publisher --run Mpu6050Publisher_StartSucceedsAndStopClosesDevice)
add_test(NAME Mpu6050Publisher_DoubleStartReturnsFalse
    COMMAND test_mpu6050_publisher --run Mpu6050Publisher_DoubleStartReturnsFalse)
add_test(NAME Mpu6050Publisher_InitialisesExpectedRegisters
    COMMAND test_mpu6050_publisher --run Mpu6050Publisher_InitialisesExpectedRegisters)
add_test(NAME Mpu6050Publisher_StartupDiscardPreventsEarlyCallbacks
    COMMAND test_mpu6050_publisher --run Mpu6050Publisher_StartupDiscardPreventsEarlyCallbacks)

set_tests_properties(
    Mpu6050Publisher_StartFailsWithoutCallback
    Mpu6050Publisher_StartFailsIfWhoAmIDoesNotMatch
    Mpu6050Publisher_StartSucceedsAndStopClosesDevice
    Mpu6050Publisher_DoubleStartReturnsFalse
    Mpu6050Publisher_InitialisesExpectedRegisters
    Mpu6050Publisher_StartupDiscardPreventsEarlyCallbacks
    PROPERTIES LABELS "hw;component"
)
