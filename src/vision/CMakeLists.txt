add_library(solar_vision STATIC
    SunTracker.cpp
)

target_include_directories(solar_vision
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(solar_vision
    PUBLIC
        solar_common
        solar_sensors
)

solar_enable_coverage(solar_vision)

if(SOLAR_ENABLE_TESTS)
    add_subdirectory(tests)
endif()
