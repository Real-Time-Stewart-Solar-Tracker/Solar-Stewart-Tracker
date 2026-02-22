cmake_minimum_required(VERSION 3.16)
project(SolarStewartTracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Warnings (professional setup)
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Core library (all sources except main + optional libcamera)
# -----------------------------
file(GLOB_RECURSE CORE_SOURCES
    src/*.cpp
)

# Exclude entry point (only belongs to the app)
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*main\\.cpp$")

# Exclude libcamera implementation by default (optional later)
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*LibcameraPublisher\\.cpp$")

add_library(solar_tracker_core STATIC ${CORE_SOURCES})
target_include_directories(solar_tracker_core PUBLIC include)

# -----------------------------
# Optional: libcamera support (Linux only, when available)
# -----------------------------
if(UNIX AND NOT APPLE)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(LIBCAMERA QUIET libcamera)
        if(LIBCAMERA_FOUND)
            target_sources(solar_tracker_core PRIVATE src/sensors/LibcameraPublisher.cpp)
            target_include_directories(solar_tracker_core PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
            target_link_libraries(solar_tracker_core PRIVATE ${LIBCAMERA_LIBRARIES})
            target_compile_options(solar_tracker_core PRIVATE ${LIBCAMERA_CFLAGS_OTHER})
            target_compile_definitions(solar_tracker_core PUBLIC SOLAR_HAVE_LIBCAMERA=1)
        else()
            target_compile_definitions(solar_tracker_core PUBLIC SOLAR_HAVE_LIBCAMERA=0)
        endif()
    else()
        target_compile_definitions(solar_tracker_core PUBLIC SOLAR_HAVE_LIBCAMERA=0)
    endif()
else()
    target_compile_definitions(solar_tracker_core PUBLIC SOLAR_HAVE_LIBCAMERA=0)
endif()

# -----------------------------
# Optional: OpenCV support (UI/plot + mouse interaction)
# -----------------------------
find_package(OpenCV QUIET)

if(OpenCV_FOUND)
    message(STATUS "Found OpenCV: ${OpenCV_VERSION}")

    # UiViewer.cpp is inside solar_tracker_core (CORE_SOURCES), so core needs OpenCV too
    target_include_directories(solar_tracker_core PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(solar_tracker_core PRIVATE ${OpenCV_LIBS})

    target_compile_definitions(solar_tracker_core PUBLIC SOLAR_HAVE_OPENCV=1)
else()
    message(STATUS "OpenCV not found (UI disabled)")
    target_compile_definitions(solar_tracker_core PUBLIC SOLAR_HAVE_OPENCV=0)
endif()

# -----------------------------
# Main executable
# -----------------------------
add_executable(solar_tracker src/main.cpp)
target_link_libraries(solar_tracker PRIVATE solar_tracker_core)

# main.cpp includes UiViewer.hpp which includes <opencv2/...>
# so the executable must also get OpenCV include dirs/libs
if(OpenCV_FOUND)
    target_include_directories(solar_tracker PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(solar_tracker PRIVATE ${OpenCV_LIBS})
endif()

# -----------------------------
# Tests
# -----------------------------
enable_testing()

set(TEST_SOURCES
    tests/test_main.cpp
    tests/test_common.hpp
    tests/test_suntracker.cpp
    tests/test_controller.cpp
)

add_executable(solar_tracker_tests ${TEST_SOURCES})
target_include_directories(solar_tracker_tests PRIVATE include tests)
target_link_libraries(solar_tracker_tests PRIVATE solar_tracker_core)

add_test(NAME solar_tracker_unit_tests COMMAND solar_tracker_tests)
