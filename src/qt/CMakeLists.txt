add_executable(solar_tracker_qt
    main_qt.cpp
    MainWindow.cpp
)
set_target_properties(solar_tracker_qt PROPERTIES
    AUTOMOC ON
    AUTOUIC ON
    AUTORCC ON
)
target_include_directories(solar_tracker_qt PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(solar_tracker_qt PRIVATE solar_app Qt5::Widgets)
if(TARGET Qt5::Charts)
    target_link_libraries(solar_tracker_qt PRIVATE Qt5::Charts)
    target_compile_definitions(solar_tracker_qt PRIVATE SOLAR_HAVE_QT_CHARTS=1)
else()
    target_compile_definitions(solar_tracker_qt PRIVATE SOLAR_HAVE_QT_CHARTS=0)
endif()
solar_enable_coverage(solar_tracker_qt)
