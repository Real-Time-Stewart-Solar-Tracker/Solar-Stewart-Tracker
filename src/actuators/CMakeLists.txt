add_library(solar_actuators
    ActuatorManager.cpp
    PCA9685.cpp
    ServoDriver.cpp
)

target_include_directories(solar_actuators PUBLIC ${CMAKE_SOURCE_DIR}/src)

if(MSVC)
    target_compile_definitions(solar_actuators PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

target_link_libraries(solar_actuators PUBLIC solar_common solar_hal)
solar_enable_coverage(solar_actuators)
add_subdirectory(test)
