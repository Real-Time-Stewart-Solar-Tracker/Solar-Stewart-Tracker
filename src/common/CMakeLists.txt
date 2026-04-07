add_library(solar_common STATIC
    Logger.cpp
    LatencyMonitor.cpp
)

target_include_directories(solar_common
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(solar_common
    PUBLIC
        Threads::Threads
)

solar_enable_coverage(solar_common)

if(SOLAR_ENABLE_TESTS)
    add_subdirectory(tests)
endif()
