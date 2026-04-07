add_library(solar_sensors STATIC
    SimulatedPublisher.cpp
    imu/Mpu6050Publisher.cpp
    manual/ADS1115ManualInput.cpp
)

target_include_directories(solar_sensors
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}
)

target_link_libraries(solar_sensors
    PUBLIC
        solar_common
        solar_hal
        gpioevent_static
)

target_compile_definitions(solar_sensors
    PUBLIC
        SOLAR_HAVE_LIBCAMERA=${SOLAR_HAVE_LIBCAMERA}
        SOLAR_HAVE_OPENCV=${SOLAR_HAVE_OPENCV}
)

if(SOLAR_HAVE_LIBCAMERA)
    target_sources(solar_sensors PRIVATE LibcameraPublisher.cpp)
    target_include_directories(solar_sensors
        PUBLIC
            ${CMAKE_SOURCE_DIR}/external/libcamera2opencv
            ${OpenCV_INCLUDE_DIRS}
            ${LIBCAMERA_INCLUDE_DIRS}
    )
    target_link_libraries(solar_sensors
        PUBLIC
            cam2opencv
            PkgConfig::LIBCAMERA
            ${OpenCV_LIBS}
    )
    target_compile_options(solar_sensors PRIVATE ${LIBCAMERA_CFLAGS_OTHER})
endif()

solar_enable_coverage(solar_sensors)

if(SOLAR_ENABLE_TESTS AND SOLAR_ENABLE_HW_TESTS)
    add_subdirectory(tests)
endif()
