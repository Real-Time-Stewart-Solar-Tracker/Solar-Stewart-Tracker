
if(UNIX AND NOT APPLE AND SOLAR_HAVE_LIBCAMERA_FLAG)
    add_executable(test_libcamera_hw
        test_libcamera_hw.cpp
    )
    target_include_directories(test_libcamera_hw PRIVATE ${CMAKE_SOURCE_DIR}/src ${CMAKE_SOURCE_DIR}/src/common/test)
    target_link_libraries(test_libcamera_hw PRIVATE solar_tracker_core)
    solar_enable_coverage(test_libcamera_hw)

    add_test(NAME LibcameraPublisher_hw_delivers_frames_with_public_contract COMMAND test_libcamera_hw)
    set_tests_properties(LibcameraPublisher_hw_delivers_frames_with_public_contract PROPERTIES LABELS "hw;camera;integration" SKIP_RETURN_CODE 77)
endif()
