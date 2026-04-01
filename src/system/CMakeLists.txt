add_library(solar_system
    SystemManager.cpp
)

target_include_directories(solar_system PUBLIC ${CMAKE_SOURCE_DIR}/src)

if(MSVC)
    target_compile_definitions(solar_system PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

target_link_libraries(solar_system PUBLIC solar_common solar_control solar_sensors solar_vision solar_actuators)
solar_enable_coverage(solar_system)
add_subdirectory(test)
