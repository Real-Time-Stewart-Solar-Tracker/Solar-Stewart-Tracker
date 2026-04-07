find_package(Qt5 COMPONENTS Widgets Charts QUIET)

if(Qt5_FOUND)
    add_executable(solar_tracker_qt
        main_qt.cpp
        MainWindow.cpp
    )

    target_include_directories(solar_tracker_qt
        PRIVATE
            ${CMAKE_SOURCE_DIR}/src
    )

    target_link_libraries(solar_tracker_qt
        PRIVATE
            solar_app
            Qt5::Widgets
            Qt5::Charts
    )

    target_compile_definitions(solar_tracker_qt PRIVATE SOLAR_HAVE_QT=1)

    if(SOLAR_HAVE_LIBCAMERA)
        target_include_directories(solar_tracker_qt
            PRIVATE
                ${CMAKE_SOURCE_DIR}/external/libcamera2opencv
        )
    endif()

    solar_enable_coverage(solar_tracker_qt)
    message(STATUS "Qt5 found: solar_tracker_qt target enabled")
else()
    message(STATUS "Qt5 Widgets/Charts not found: solar_tracker_qt will not be built")
endif()
