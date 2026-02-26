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

# Shared test framework sources (provides Register + main runner)
set(TEST_FRAMEWORK
    tests/test_main.cpp
    tests/test_common.hpp
)

# Core logic tests
add_executable(test_core
    ${TEST_FRAMEWORK}
    tests/test_suntracker.cpp
    tests/test_controller.cpp
)
target_include_directories(test_core PRIVATE include tests)
target_link_libraries(test_core PRIVATE solar_tracker_core)

# PCA9685 driver tests
add_executable(test_pca9685
    ${TEST_FRAMEWORK}
    tests/test_pca9685.cpp
    tests/FakeI2CDevice.hpp
)
target_include_directories(test_pca9685 PRIVATE include tests)
target_link_libraries(test_pca9685 PRIVATE solar_tracker_core)

# ServoDriver hardware-abstraction tests
add_executable(test_servodriver
    ${TEST_FRAMEWORK}
    tests/test_servodriver.cpp
    tests/FakeI2CDevice.hpp
)
target_include_directories(test_servodriver PRIVATE include tests)
target_link_libraries(test_servodriver PRIVATE solar_tracker_core)
# --- Per-test registration (so CTest shows 1/N like your screenshot) ---

# SunTracker tests (in test_core)
add_test(NAME SunTracker_NoBrightPixels_ConfidenceZero
         COMMAND test_core --run SunTracker_NoBrightPixels_ConfidenceZero)

add_test(NAME SunTracker_BrightSpot_CentroidApproxCorrect
         COMMAND test_core --run SunTracker_BrightSpot_CentroidApproxCorrect)

add_test(NAME SunTracker_BrightSpot_HigherAreaHigherConfidence
         COMMAND test_core --run SunTracker_BrightSpot_HigherAreaHigherConfidence)

# Controller tests (in test_core)
add_test(NAME Controller_LowConfidence_NoMotion
         COMMAND test_core --run Controller_LowConfidence_NoMotion)

add_test(NAME Controller_WithinDeadband_NoMotion
         COMMAND test_core --run Controller_WithinDeadband_NoMotion)

add_test(NAME Controller_OutsideDeadband_ProducesMotion
         COMMAND test_core --run Controller_OutsideDeadband_ProducesMotion)

add_test(NAME Controller_OutputClamped
         COMMAND test_core --run Controller_OutputClamped)

# PCA9685 tests
add_test(NAME PCA9685_init_writes_mode_and_prescale
         COMMAND test_pca9685 --run PCA9685_init_writes_mode_and_prescale)

add_test(NAME PCA9685_set_pwm_writes_led_register_block
         COMMAND test_pca9685 --run PCA9685_set_pwm_writes_led_register_block)

add_test(NAME PCA9685_set_pulse_us_uses_frequency_conversion
         COMMAND test_pca9685 --run PCA9685_set_pulse_us_uses_frequency_conversion)

# ServoDriver tests
add_test(NAME ServoDriver_start_parks_to_neutral_when_enabled
         COMMAND test_servodriver --run ServoDriver_start_parks_to_neutral_when_enabled)

add_test(NAME ServoDriver_apply_clamps_and_writes_channels
         COMMAND test_servodriver --run ServoDriver_apply_clamps_and_writes_channels)