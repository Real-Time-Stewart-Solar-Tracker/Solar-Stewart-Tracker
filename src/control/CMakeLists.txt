add_library(solar_control
    Controller.cpp
    Kinematics3RRS.cpp
)

target_include_directories(solar_control PUBLIC ${CMAKE_SOURCE_DIR}/src)

if(MSVC)
    target_compile_definitions(solar_control PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

target_link_libraries(solar_control PUBLIC solar_common)
solar_enable_coverage(solar_control)
add_subdirectory(test)
