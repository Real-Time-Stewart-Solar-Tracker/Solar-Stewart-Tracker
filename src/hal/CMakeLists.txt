add_library(solar_hal
    LinuxI2CDevice.cpp
)

target_include_directories(solar_hal PUBLIC ${CMAKE_SOURCE_DIR}/src)

if(MSVC)
    target_compile_definitions(solar_hal PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

target_link_libraries(solar_hal PUBLIC solar_common)
solar_enable_coverage(solar_hal)
add_subdirectory(test)
