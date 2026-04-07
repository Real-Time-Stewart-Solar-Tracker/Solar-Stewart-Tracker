set(SOLAR_TEST_SUPPORT_DIR ${CMAKE_SOURCE_DIR}/src/tests/support)

add_executable(test_vision
    ${SOLAR_TEST_SUPPORT_DIR}/test_main.cpp
    test_suntracker.cpp
)

target_include_directories(test_vision PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}
)

target_link_libraries(test_vision PRIVATE solar_vision)
solar_enable_coverage(test_vision)

add_test(NAME SunTracker_DetectsSingleBrightPixel COMMAND test_vision --run SunTracker_DetectsSingleBrightPixel)
add_test(NAME SunTracker_NoBrightPixels_ProducesZeroConfidence COMMAND test_vision --run SunTracker_NoBrightPixels_ProducesZeroConfidence)
add_test(NAME SunTracker_WeightedCentroid_IsCorrect COMMAND test_vision --run SunTracker_WeightedCentroid_IsCorrect)
