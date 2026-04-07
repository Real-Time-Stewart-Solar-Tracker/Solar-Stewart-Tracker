add_library(solar_app STATIC
    AppConfig.cpp
    SystemFactory.cpp
    CliController.cpp
    LinuxEventLoop.cpp
    Application.cpp
)

target_include_directories(solar_app
    PUBLIC
        ${CMAKE_SOURCE_DIR}/src
        ${CMAKE_SOURCE_DIR}
)

target_link_libraries(solar_app
    PUBLIC
        solar_system
)

solar_enable_coverage(solar_app)
