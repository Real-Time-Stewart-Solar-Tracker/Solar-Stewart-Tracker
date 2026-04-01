add_library(solar_common
    Logger.cpp
    LatencyMonitor.cpp
)

target_include_directories(solar_common PUBLIC ${CMAKE_SOURCE_DIR}/src)

if(MSVC)
    target_compile_definitions(solar_common PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

solar_enable_coverage(solar_common)

add_subdirectory(test)
