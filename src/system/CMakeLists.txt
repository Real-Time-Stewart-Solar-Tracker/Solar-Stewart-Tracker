add_library(solar_system STATIC
    BackendCoordinator.cpp
    GuiManualDispatcher.cpp
    SystemManager.cpp
)

target_include_directories(solar_system
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(solar_system
    PUBLIC
        solar_common
        solar_control
        solar_actuators
        solar_sensors
        solar_vision
)

if(TARGET gpioevent_static)
    target_link_libraries(solar_system PUBLIC gpioevent_static)
elseif(TARGET gpioevent)
    target_link_libraries(solar_system PUBLIC gpioevent)
else()
    message(STATUS "gpioevent target not found: building system without gpioevent")
endif()

solar_enable_coverage(solar_system)
solar_project_warnings(solar_system)

if(SOLAR_ENABLE_TESTS)
    add_subdirectory(tests)
endif()
