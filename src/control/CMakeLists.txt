add_library(solar_control STATIC
    Controller.cpp
    ManualInputMapper.cpp
    ImuFeedbackMapper.cpp
    ImuTiltEstimator.cpp
    Kinematics3RRS.cpp
    ManualImuCoordinator.cpp
)

target_include_directories(solar_control
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(solar_control
    PUBLIC
        solar_common
        solar_hal
)

solar_enable_coverage(solar_control)

if(SOLAR_ENABLE_TESTS)
    add_subdirectory(tests)
endif()
