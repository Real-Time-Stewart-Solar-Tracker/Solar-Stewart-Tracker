
cmake_minimum_required(VERSION 3.16)
project(SolarStewartTracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        add_compile_options(-Wno-free-nonheap-object)
    endif()
endif()

option(SOLAR_TRY_OPENCV "Enable OpenCV viewer if OpenCV is found" ON)
option(SOLAR_ENABLE_COVERAGE "Enable coverage instrumentation" OFF)

function(solar_enable_coverage target_name)
    if(NOT SOLAR_ENABLE_COVERAGE)
        return()
    endif()
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target_name} PRIVATE -O0 -g --coverage)
        target_link_options(${target_name} PRIVATE --coverage)
    endif()
endfunction()

set(SOLAR_HAVE_LIBCAMERA_FLAG 0)
set(SOLAR_HAVE_QT_FLAG 0)
set(SOLAR_HAVE_OPENCV_FLAG 0)

if(UNIX AND NOT APPLE)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(LIBCAMERA QUIET libcamera)
        if(LIBCAMERA_FOUND)
            set(SOLAR_HAVE_LIBCAMERA_FLAG 1)
            message(STATUS "libcamera found -> enabling LibcameraPublisher")
        else()
            message(STATUS "libcamera not found -> using SimulatedPublisher / non-libcamera paths")
        endif()
    else()
        message(STATUS "PkgConfig not found -> libcamera support disabled")
    endif()
endif()

if(SOLAR_TRY_OPENCV)
    find_package(OpenCV QUIET)
    if(OpenCV_FOUND)
        set(SOLAR_HAVE_OPENCV_FLAG 1)
        message(STATUS "OpenCV found -> enabling UiViewer")
    else()
        message(STATUS "OpenCV not found -> building headless")
    endif()
endif()

find_package(Qt5 QUIET COMPONENTS Widgets Charts)
if(Qt5Widgets_FOUND)
    set(SOLAR_HAVE_QT_FLAG 1)
    message(STATUS "Qt5 found -> building solar_tracker_qt")
else()
    message(STATUS "Qt5 not found -> Qt GUI disabled (no solar_tracker_qt target)")
endif()

add_compile_definitions(
    SOLAR_HAVE_LIBCAMERA=${SOLAR_HAVE_LIBCAMERA_FLAG}
    SOLAR_HAVE_QT=${SOLAR_HAVE_QT_FLAG}
    SOLAR_HAVE_OPENCV=${SOLAR_HAVE_OPENCV_FLAG}
)

enable_testing()

add_subdirectory(src/common)
add_subdirectory(src/hal)
add_subdirectory(src/control)
add_subdirectory(src/vision)
add_subdirectory(src/sensors)
if(SOLAR_HAVE_OPENCV_FLAG)
    add_subdirectory(src/ui)
else()
    add_library(solar_ui INTERFACE)
endif()
add_subdirectory(src/actuators)
add_subdirectory(src/system)

add_library(solar_tracker_core INTERFACE)
target_link_libraries(solar_tracker_core
    INTERFACE
        solar_common
        solar_hal
        solar_control
        solar_vision
        solar_sensors
        solar_ui
        solar_actuators
        solar_system
)

target_include_directories(solar_tracker_core INTERFACE ${CMAKE_SOURCE_DIR}/src)

add_subdirectory(src/app)
if(SOLAR_HAVE_QT_FLAG)
    add_subdirectory(src/qt)
endif()

if(SOLAR_ENABLE_COVERAGE)
    find_program(GCOVR_EXECUTABLE gcovr)
    if(GCOVR_EXECUTABLE)
        add_custom_target(coverage
            COMMAND ${CMAKE_CTEST_COMMAND} --test-dir ${CMAKE_BINARY_DIR} --output-on-failure
            COMMAND ${GCOVR_EXECUTABLE}
                    -r ${CMAKE_SOURCE_DIR}
                    --filter ${CMAKE_SOURCE_DIR}/src
                    --exclude ${CMAKE_SOURCE_DIR}/docs
                    --exclude ${CMAKE_SOURCE_DIR}/datasheets
                    --exclude ${CMAKE_SOURCE_DIR}/diagrams
                    --exclude ${CMAKE_SOURCE_DIR}/media
                    --xml-pretty -o ${CMAKE_BINARY_DIR}/coverage.xml
                    --html-details -o ${CMAKE_BINARY_DIR}/coverage.html
            WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
            COMMENT "Running tests and generating coverage report"
        )
    else()
        message(WARNING "SOLAR_ENABLE_COVERAGE=ON but gcovr was not found; no 'coverage' target will be created")
    endif()
endif()
