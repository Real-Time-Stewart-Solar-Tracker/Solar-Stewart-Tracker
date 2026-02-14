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

# Include directory
include_directories(include)

# Collect sources
file(GLOB_RECURSE SOURCES
    src/*.cpp
)

add_executable(solar_tracker ${SOURCES})

# Linux-specific: link libcamera only if building on Linux
if(UNIX AND NOT APPLE)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBCAMERA libcamera)

    if(LIBCAMERA_FOUND)
        target_include_directories(solar_tracker PRIVATE ${LIBCAMERA_INCLUDE_DIRS})
        target_link_libraries(solar_tracker ${LIBCAMERA_LIBRARIES})
        target_compile_options(solar_tracker PRIVATE ${LIBCAMERA_CFLAGS_OTHER})
    endif()
endif()