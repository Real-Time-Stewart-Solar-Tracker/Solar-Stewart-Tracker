cmake_minimum_required(VERSION 3.16)
project(SolarStewartTracker LANGUAGES CXX)

# ============================================================
# Global CMake settings
# ============================================================
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Warnings (professional default)
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
    # GCC 13/14 can emit noisy/false-positive warnings in some STL paths on ARM
    if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
        add_compile_options(-Wno-free-nonheap-object)
    endif()
endif()

# ============================================================
# Options (no Qt option on purpose; Qt is auto-detected)
# ============================================================
option(SOLAR_TRY_OPENCV "Enable OpenCV viewer if OpenCV is found" ON)
option(SOLAR_HAVE_IMU "Enable MPU6050 IMU mode (Linux only)" OFF)

# ============================================================
# Core library sources (everything except entry points + qt + ui)
# ============================================================
file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS
    src/*.cpp
)

# Exclude entry points
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*[/\\\\]main\\.cpp$")
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*[/\\\\]main_qt\\.cpp$")

# Exclude Qt sources from core
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*[/\\\\]qt[/\\\\].*")

# Exclude UI sources from core by default (added back conditionally)
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*[/\\\\]ui[/\\\\].*")

# Exclude libcamera implementation by default (added conditionally)
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*LibcameraPublisher\\.cpp$")

# Exclude MPU6050 implementation by default (added conditionally)
list(FILTER CORE_SOURCES EXCLUDE REGEX ".*MPU6050Publisher\\.cpp$")

add_library(solar_tracker_core STATIC ${CORE_SOURCES})
target_include_directories(solar_tracker_core PUBLIC include)

# MSVC: silence sscanf “unsafe” warning without changing code
if(MSVC)
    target_compile_definitions(solar_tracker_core PRIVATE _CRT_SECURE_NO_WARNINGS)
endif()

# ============================================================
# Feature flags (computed once)
# ============================================================
set(SOLAR_HAVE_LIBCAMERA_FLAG 0)
set(SOLAR_HAVE_QT_FLAG        0)
set(SOLAR_HAVE_OPENCV_FLAG    0)
set(SOLAR_HAVE_IMU_FLAG       0)

# ============================================================
# Optional: libcamera support (Linux only)
# ============================================================
if(UNIX AND NOT APPLE)
    find_package(PkgConfig QUIET)
    if(PkgConfig_FOUND)
        pkg_check_modules(LIBCAMERA QUIET libcamera)
        if(LIBCAMERA_FOUND)
            target_sources(solar_tracker_core PRIVATE src/sensors/LibcameraPublisher.cpp)
            target_include_directories(solar_tracker_core PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
            target_link_libraries(solar_tracker_core PRIVATE ${LIBCAMERA_LIBRARIES})
            target_compile_options(solar_tracker_core PRIVATE ${LIBCAMERA_CFLAGS_OTHER})
            set(SOLAR_HAVE_LIBCAMERA_FLAG 1)
            message(STATUS "libcamera found -> enabling LibcameraPublisher")
        else()
            message(STATUS "libcamera not found -> using SimulatedPublisher / non-libcamera paths")
        endif()
    endif()
endif()

# ============================================================
# Optional: OpenCV UI (auto-enable if OpenCV is found)
# ============================================================
if(SOLAR_TRY_OPENCV)
    find_package(OpenCV QUIET)
    if(OpenCV_FOUND)
        message(STATUS "OpenCV found -> enabling UiViewer")

        # UiViewer.cpp was excluded earlier -> add it only when OpenCV exists
        target_sources(solar_tracker_core PRIVATE src/ui/UiViewer.cpp)

        if(DEFINED OpenCV_INCLUDE_DIRS)
            target_include_directories(solar_tracker_core PRIVATE ${OpenCV_INCLUDE_DIRS})
        endif()

        if(DEFINED OpenCV_LIBS)
            target_link_libraries(solar_tracker_core PRIVATE ${OpenCV_LIBS})
        elseif(DEFINED OpenCV_LIBRARIES)
            target_link_libraries(solar_tracker_core PRIVATE ${OpenCV_LIBRARIES})
        endif()

        set(SOLAR_HAVE_OPENCV_FLAG 1)
    else()
        message(STATUS "OpenCV not found -> building headless")
    endif()
endif()

# ============================================================
# Optional: MPU6050 IMU mode (Linux only)
# ============================================================
if(SOLAR_HAVE_IMU)
    if(UNIX AND NOT APPLE)
        target_sources(solar_tracker_core PRIVATE src/sensors/MPU6050Publisher.cpp)
        set(SOLAR_HAVE_IMU_FLAG 1)
        message(STATUS "IMU mode enabled -> MPU6050Publisher (i2c-1, 0x68)")
    else()
        message(WARNING "SOLAR_HAVE_IMU=ON but not Linux -- IMU disabled")
    endif()
endif()

# ============================================================
# CLI app (always built)
# ============================================================
add_executable(solar_tracker
    src/main.cpp
)
target_link_libraries(solar_tracker PRIVATE solar_tracker_core)
target_include_directories(solar_tracker PRIVATE include)

# ============================================================
# Qt GUI (auto-enable if Qt5 Widgets is found)
#   - No -DSOLAR_HAVE_QT=ON needed
#   - If Qt isn't installed, target is simply not built
# ============================================================
find_package(Qt5 QUIET COMPONENTS Widgets Charts)

if(Qt5Widgets_FOUND)
    message(STATUS "Qt5 found -> building solar_tracker_qt")
    set(SOLAR_HAVE_QT_FLAG 1)

    add_executable(solar_tracker_qt
        src/qt/main_qt.cpp
        src/qt/MainWindow.cpp
    )

    # Enable Qt autogen ONLY for this target
    set_target_properties(solar_tracker_qt PROPERTIES
        AUTOMOC ON
        AUTOUIC ON
        AUTORCC ON
    )

    target_include_directories(solar_tracker_qt PRIVATE include src/qt)
    target_link_libraries(solar_tracker_qt PRIVATE solar_tracker_core Qt5::Widgets)

    if(TARGET Qt5::Charts)
        target_link_libraries(solar_tracker_qt PRIVATE Qt5::Charts)
        target_compile_definitions(solar_tracker_qt PRIVATE SOLAR_HAVE_QT_CHARTS=1)
    else()
        target_compile_definitions(solar_tracker_qt PRIVATE SOLAR_HAVE_QT_CHARTS=0)
    endif()
else()
    message(STATUS "Qt5 not found -> Qt GUI disabled (no solar_tracker_qt target)")
endif()

# ============================================================
# Publish feature flags to code (define ONCE; no redefinition spam)
# ============================================================
target_compile_definitions(solar_tracker_core PUBLIC
    SOLAR_HAVE_LIBCAMERA=${SOLAR_HAVE_LIBCAMERA_FLAG}
    SOLAR_HAVE_QT=${SOLAR_HAVE_QT_FLAG}
    SOLAR_HAVE_OPENCV=${SOLAR_HAVE_OPENCV_FLAG}
    SOLAR_HAVE_IMU=${SOLAR_HAVE_IMU_FLAG}
)

# ============================================================
# Tests
# ============================================================
enable_testing()
add_subdirectory(tests)