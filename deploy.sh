#!/bin/bash

# 3RRS C++追踪系统 - 自动部署脚本
# 在树莓派5上运行此脚本以一键部署系统

echo "=================================================="
echo "  3RRS C++追踪系统 - 自动部署"
echo "=================================================="
echo ""

# 检查是否在树莓派上运行
if [ ! -f /proc/device-tree/model ] || ! grep -q "Raspberry Pi" /proc/device-tree/model; then
    echo "警告: 此脚本应该在树莓派上运行"
    read -p "是否继续？(y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 1. 更新系统
echo "[步骤 1/5] 更新系统..."
sudo apt-get update

# 2. 安装依赖库
echo ""
echo "[步骤 2/5] 安装依赖库..."
sudo apt-get install -y \
    build-essential \
    cmake \
    libi2c-dev \
    libopencv-dev \
    libeigen3-dev \
    libgtest-dev

# 检查安装是否成功
if [ $? -ne 0 ]; then
    echo "错误: 依赖库安装失败"
    exit 1
fi

# 编译安装GTest库（如果尚未安装）
if [ ! -f /usr/lib/libgtest.a ]; then
    echo "编译安装 Google Test..."
    cd /usr/src/gtest && sudo cmake . && sudo make && sudo cp lib/*.a /usr/lib/
    cd - > /dev/null
fi

# 3. 检查I2C是否启用
echo ""
echo "[步骤 3/5] 检查I2C..."
if [ ! -e /dev/i2c-1 ]; then
    echo "警告: I2C未启用"
    echo "请运行 'sudo raspi-config' 并在 Interface Options 中启用 I2C"
    echo "然后重启树莓派"
    exit 1
else
    echo "I2C 已启用 ✓"
fi

# 4. 编译项目
echo ""
echo "[步骤 4/5] 编译项目..."

# 创建build目录
if [ ! -d "build" ]; then
    mkdir build
fi

cd build

# CMake配置
echo "正在配置CMake..."
cmake ..

if [ $? -ne 0 ]; then
    echo "错误: CMake配置失败"
    exit 1
fi

# 编译
echo "正在编译..."
make -j4  # 使用4个核心并行编译

if [ $? -ne 0 ]; then
    echo "错误: 编译失败"
    exit 1
fi

echo "编译成功 ✓"
echo "生成的可执行文件:"
ls -lh main_tracker test_servos test_circular test_pid test_kinematics_unit 2>/dev/null

# 运行单元测试
echo ""
echo "运行单元测试..."
ctest --verbose
if [ $? -eq 0 ]; then
    echo "单元测试全部通过 ✓"
else
    echo "警告: 部分单元测试失败"
fi

# 5. 完成
echo ""
echo "[步骤 5/5] 部署完成！"
echo ""
echo "=================================================="
echo "  可用的程序："
echo "=================================================="
echo ""
echo "1. 主程序（视觉追踪）："
echo "   ./main_tracker"
echo ""
echo "2. 舵机测试："
echo "   ./test_servos"
echo ""
echo "3. 圆形轨迹测试："
echo "   ./test_circular"
echo ""
echo "=================================================="
echo ""
echo "提示："
echo "- 运行 'ctest --verbose' 执行单元测试"
echo "- 运行前请确认硬件连接正确"
echo "- 按 Ctrl+C 退出程序"
echo "- 如需帮助，查看 README.md"
echo ""
