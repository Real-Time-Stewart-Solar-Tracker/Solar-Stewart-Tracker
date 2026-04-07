cmake_minimum_required(VERSION 3.16)

project(SolarStewartTracker LANGUAGES CXX)

# Prevent in-source builds. cmake must be run with -B <build_dir>.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_BINARY_DIR)
    message(FATAL_ERROR
        "In-source builds are not allowed.\n"
        "Run: cmake -S . -B build-pi\n"
        "Then: cmake --build build-pi")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(SOLAR_ENABLE_TESTS      "Build tests"                                    ON)
option(SOLAR_ENABLE_HW_TESTS   "Register real hardware smoke tests"            OFF)
option(SOLAR_ENABLE_QT         "Build Qt GUI target if Qt5 is available"        ON)
option(SOLAR_TRY_LIBCAMERA     "Try to enable libcamera backend"                ON)
option(SOLAR_TRY_OPENCV        "Try to enable OpenCV support"                   ON)
option(SOLAR_ENABLE_COVERAGE   "Enable coverage flags where supported"          OFF)

if(MSVC)
    add_compile_options(/W4)
else()
    # Base warning flags for all targets including external submodules.
    # -Werror is applied only to project-owned targets via solar_project_warnings()
    # so that warnings in external submodule code do not break the build.
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Call this on every project-owned library and executable to treat warnings
# as errors, matching the lecture requirement for compiler errors enabled.
function(solar_project_warnings target_name)
    if(NOT MSVC)
        target_compile_options(${target_name} PRIVATE -Werror)
    else()
        target_compile_options(${target_name} PRIVATE /WX)
    endif()
endfunction()

function(solar_enable_coverage target_name)
    if(SOLAR_ENABLE_COVERAGE AND NOT MSVC)
        target_compile_options(${target_name} PRIVATE -O0 -g --coverage)
        target_link_options(${target_name} PRIVATE --coverage)
    endif()
endfunction()

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

if(SOLAR_TRY_OPENCV)
    find_package(OpenCV QUIET)
    if(OpenCV_FOUND)
        set(SOLAR_HAVE_OPENCV 1)
        message(STATUS "OpenCV found: optional OpenCV support enabled")
    else()
        set(SOLAR_HAVE_OPENCV 0)
        message(STATUS "OpenCV not found: OpenCV support disabled")
    endif()
else()
    set(SOLAR_HAVE_OPENCV 0)
    message(STATUS "OpenCV probing disabled by SOLAR_TRY_OPENCV=OFF")
endif()

if(SOLAR_TRY_LIBCAMERA)
    pkg_check_modules(LIBCAMERA QUIET IMPORTED_TARGET libcamera)
    if(LIBCAMERA_FOUND AND SOLAR_HAVE_OPENCV)
        set(SOLAR_HAVE_LIBCAMERA 1)
        message(STATUS "libcamera found with OpenCV: optional libcamera backend enabled")
    else()
        set(SOLAR_HAVE_LIBCAMERA 0)
        if(LIBCAMERA_FOUND AND NOT SOLAR_HAVE_OPENCV)
            message(STATUS "libcamera found but OpenCV is required: libcamera backend disabled")
        else()
            message(STATUS "libcamera not found: libcamera backend disabled")
        endif()
    endif()
else()
    set(SOLAR_HAVE_LIBCAMERA 0)
    message(STATUS "libcamera probing disabled by SOLAR_TRY_LIBCAMERA=OFF")
endif()

set(SOLAR_HAVE_GPIOD_DEMO 0)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/external/libgpiod_event_demo/CMakeLists.txt")
    add_subdirectory(external/libgpiod_event_demo)
    set(SOLAR_HAVE_GPIOD_DEMO 1)
    message(STATUS "external/libgpiod_event_demo found")
else()
    message(STATUS "external/libgpiod_event_demo not found: GPIO demo support disabled")
endif()

# Build cam2opencv directly rather than via add_subdirectory on the submodule.
# The submodule's CMakeLists.txt uses CMAKE_SOURCE_DIR for includes which
# resolves to the project root when used as a subdirectory, causing the
# metadataprinter example to fail. Building only the library target directly
# avoids that problem entirely.
set(SOLAR_HAVE_LIBCAM2OPENCV 0)
set(LIBCAM2OPENCV_DIR "${CMAKE_CURRENT_SOURCE_DIR}/external/libcamera2opencv")
if(SOLAR_HAVE_LIBCAMERA AND EXISTS "${LIBCAM2OPENCV_DIR}/libcam2opencv.cpp")
    add_library(cam2opencv STATIC "${LIBCAM2OPENCV_DIR}/libcam2opencv.cpp")
    target_include_directories(cam2opencv
        PUBLIC
            "${LIBCAM2OPENCV_DIR}"
            ${LIBCAMERA_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
    )
    target_link_libraries(cam2opencv
        PUBLIC
            PkgConfig::LIBCAMERA
            ${OpenCV_LIBS}
    )
    set(SOLAR_HAVE_LIBCAM2OPENCV 1)
    message(STATUS "external/libcamera2opencv found")
else()
    if(SOLAR_HAVE_LIBCAMERA)
        message(STATUS "external/libcamera2opencv not found")
    else()
        message(STATUS "external/libcamera2opencv found but libcamera/OpenCV unavailable")
    endif()
endif()

if(SOLAR_ENABLE_TESTS)
    enable_testing()
endif()

set(SOLAR_HAVE_LIBCAMERA  ${SOLAR_HAVE_LIBCAMERA}  CACHE INTERNAL "")
set(SOLAR_HAVE_OPENCV     ${SOLAR_HAVE_OPENCV}     CACHE INTERNAL "")
set(SOLAR_HAVE_GPIOD_DEMO ${SOLAR_HAVE_GPIOD_DEMO} CACHE INTERNAL "")

add_subdirectory(src/common)
add_subdirectory(src/hal)
add_subdirectory(src/sensors)
add_subdirectory(src/control)
add_subdirectory(src/vision)
add_subdirectory(src/actuators)
add_subdirectory(src/system)
add_subdirectory(src/app)

if(SOLAR_ENABLE_QT)
    add_subdirectory(src/qt)
endif()

add_executable(solar_tracker src/main.cpp)
target_include_directories(solar_tracker PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(solar_tracker PRIVATE solar_app)
solar_enable_coverage(solar_tracker)
solar_project_warnings(solar_tracker)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/demo/CMakeLists.txt")
    add_subdirectory(demo)
endif()
