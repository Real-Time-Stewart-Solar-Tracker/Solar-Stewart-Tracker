add_library(solar_ui
    UiViewer.cpp
)

target_include_directories(solar_ui
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
)

if(OpenCV_FOUND)
    target_include_directories(solar_ui
        PUBLIC
            ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(solar_ui
        PUBLIC
            ${OpenCV_LIBS}
    )
endif()