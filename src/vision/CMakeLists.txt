add_library(solar_vision
    SunTracker.cpp
)

target_include_directories(solar_vision PUBLIC ${CMAKE_SOURCE_DIR}/src)

if(MSVC)
    target_compile_definitions(solar_vision PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

target_link_libraries(solar_vision PUBLIC solar_common)
solar_enable_coverage(solar_vision)
add_subdirectory(test)
