add_library(solar_hal STATIC
    LinuxI2CDevice.cpp
)

target_include_directories(solar_hal
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

target_link_libraries(solar_hal
    PUBLIC
        solar_common
)

solar_enable_coverage(solar_hal)

if(SOLAR_ENABLE_TESTS AND SOLAR_ENABLE_HW_TESTS)
    add_subdirectory(tests)
endif()
