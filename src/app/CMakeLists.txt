add_library(solar_app
    AppConfig.cpp
    Application.cpp
    CliController.cpp
    LinuxEventLoop.cpp
    SystemFactory.cpp
)

target_include_directories(solar_app
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(solar_app
    PUBLIC
        solar_common
        solar_hal
        solar_control
        solar_vision
        solar_sensors
        solar_ui
        solar_actuators
        solar_system
)

solar_enable_coverage(solar_app)

add_executable(solar_tracker
    main.cpp
)

target_link_libraries(solar_tracker
    PRIVATE
        solar_app
)