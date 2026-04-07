cmake_minimum_required(VERSION 3.16)
project(pipeline_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------
# Usage from the repo root:
#   cmake -B build -DCMAKE_BUILD_TYPE=Release
#   cmake --build build --target pipeline_demo
# -----------------------------------------------------------------------

add_executable(pipeline_demo main.cpp)

target_link_libraries(pipeline_demo
    PRIVATE
        solar_vision      # SunTracker
        solar_control     # Controller, Kinematics3RRS
        solar_sensors     # SimulatedPublisher
        solar_common      # Logger, Types
)

# Coverage and warning flags must be applied to the demo executable so that
# when the static libraries are built with --coverage (SOLAR_ENABLE_COVERAGE=ON),
# the linker can resolve __gcov_init / __gcov_exit symbols.
solar_enable_coverage(pipeline_demo)
solar_project_warnings(pipeline_demo)

target_include_directories(pipeline_demo
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src
)
