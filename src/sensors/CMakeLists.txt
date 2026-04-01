add_library(solar_sensors
    SimulatedPublisher.cpp
)

target_include_directories(solar_sensors PUBLIC ${CMAKE_SOURCE_DIR}/src)

if(MSVC)
    target_compile_definitions(solar_sensors PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

target_link_libraries(solar_sensors PUBLIC solar_common)

if(UNIX AND NOT APPLE AND SOLAR_HAVE_LIBCAMERA_FLAG)
    target_sources(solar_sensors PRIVATE LibcameraPublisher.cpp)
    target_include_directories(solar_sensors PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
    target_link_libraries(solar_sensors PRIVATE ${LIBCAMERA_LIBRARIES})
    target_compile_options(solar_sensors PRIVATE ${LIBCAMERA_CFLAGS_OTHER})
endif()

solar_enable_coverage(solar_sensors)
add_subdirectory(test)
