add_library(solar_actuators STATIC
    ActuatorManager.cpp
    ServoDriver.cpp
    PCA9685.cpp
)

target_include_directories(solar_actuators
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(solar_actuators
    PUBLIC
        solar_common
        solar_hal
)

solar_enable_coverage(solar_actuators)

if(SOLAR_ENABLE_TESTS)
    add_subdirectory(tests)
endif()
