cmake_minimum_required(VERSION 3.10)
project(RRS_Tracker)

# 设置C++标准
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖库
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# 包含目录
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# 源文件
set(CORE_SOURCES
    src/pca9685.cpp
    src/kinematics.cpp
    src/pid_controller.cpp
    src/vision_system.cpp
)

# 主程序
add_executable(main_tracker
    ${CORE_SOURCES}
    src/main_tracker.cpp
)

target_link_libraries(main_tracker
    ${OpenCV_LIBS}
    i2c
    pthread
)

# 测试程序 - 简单舵机测试
add_executable(test_servos
    src/pca9685.cpp
    tests/test_servos.cpp
)

target_link_libraries(test_servos
    i2c
)

# 测试程序 - 圆形轨迹测试
add_executable(test_circular
    src/pca9685.cpp
    src/kinematics.cpp
    tests/test_circular.cpp
)

target_link_libraries(test_circular
    i2c
)
