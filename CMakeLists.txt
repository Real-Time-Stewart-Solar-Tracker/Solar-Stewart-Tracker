cmake_minimum_required(VERSION 3.10)
project(RRS_Tracker)

# C++17 standard (uses structured bindings, std::make_unique, etc.)
# C++17标准（使用了structured bindings、std::make_unique等特性）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies / 查找依赖库
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories / 包含目录
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# Core source files / 核心源文件
set(CORE_SOURCES
    src/pca9685.cpp
    src/kinematics.cpp
    src/pid_controller.cpp
    src/vision_system.cpp
    src/tracker_controller.cpp
)

# ========== Main program / 主程序 ==========
add_executable(main_tracker
    ${CORE_SOURCES}
    src/main_tracker.cpp
)

target_link_libraries(main_tracker
    ${OpenCV_LIBS}
    i2c
    pthread
)

# ========== Hardware test - simple servo test / 硬件测试 — 简单舵机测试 ==========
add_executable(test_servos
    src/pca9685.cpp
    tests/test_servos.cpp
)

target_link_libraries(test_servos
    i2c
)

# ========== Hardware test - circular trajectory / 硬件测试 — 圆形轨迹测试 ==========
add_executable(test_circular
    src/pca9685.cpp
    src/kinematics.cpp
    tests/test_circular.cpp
)

target_link_libraries(test_circular
    i2c
)

# ========== Google Test unit tests / 单元测试 ==========
# GTest is optional: skip unit tests if not installed
# GTest 可选：如果未安装则跳过单元测试
find_package(GTest QUIET)

if(GTest_FOUND)
    enable_testing()

    # PID controller unit test / PID控制器单元测试
    add_executable(test_pid
        src/pid_controller.cpp
        tests/test_pid.cpp
    )
    target_include_directories(test_pid PRIVATE ${CMAKE_SOURCE_DIR}/include)
    target_link_libraries(test_pid GTest::GTest GTest::Main pthread)
    add_test(NAME PIDController COMMAND test_pid)

    # Inverse kinematics unit test / 逆运动学单元测试
    add_executable(test_kinematics_unit
        src/kinematics.cpp
        tests/test_kinematics.cpp
    )
    target_include_directories(test_kinematics_unit PRIVATE ${CMAKE_SOURCE_DIR}/include)
    target_link_libraries(test_kinematics_unit GTest::GTest GTest::Main pthread)
    add_test(NAME Kinematics COMMAND test_kinematics_unit)

    message(STATUS "Google Test found, unit tests enabled / 已找到，单元测试已启用")
    message(STATUS "  Run / 运行: cmake --build build && cd build && ctest --verbose")
else()
    message(STATUS "Google Test not found, skipping unit tests / 未找到，跳过单元测试")
    message(STATUS "  Install / 安装: sudo apt-get install libgtest-dev && cd /usr/src/gtest && sudo cmake . && sudo make && sudo cp lib/*.a /usr/lib/")
endif()
