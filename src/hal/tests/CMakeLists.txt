add_executable(test_linux_i2c_hw
    test_linux_i2c_hw.cpp
)

target_include_directories(test_linux_i2c_hw PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(test_linux_i2c_hw PRIVATE solar_hal solar_actuators)
solar_enable_coverage(test_linux_i2c_hw)

add_test(NAME LinuxI2C_PCA9685_hw_init_and_write COMMAND test_linux_i2c_hw)
set_tests_properties(LinuxI2C_PCA9685_hw_init_and_write PROPERTIES LABELS "hw;i2c;integration" SKIP_RETURN_CODE 77)
