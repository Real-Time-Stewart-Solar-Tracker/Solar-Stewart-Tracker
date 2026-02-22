#!/bin/bash
# setup_pi.sh — 树莓派5 太阳追踪系统环境配置
# 在树莓派上运行：chmod +x setup_pi.sh && ./setup_pi.sh

set -e

echo "=========================================="
echo "  3RRS 太阳追踪 — 树莓派5 环境配置"
echo "=========================================="

# 1. 系统依赖
echo "[1/4] 安装系统依赖..."
sudo apt update
sudo apt install -y \
    python3-smbus \
    i2c-tools \
    python3-opencv \
    python3-numpy \
    python3-picamera2

# 2. Python 依赖
echo "[2/4] 安装 Python 依赖..."
pip3 install --break-system-packages smbus2 2>/dev/null || pip3 install smbus2

# 3. 检查 I2C 是否启用
echo "[3/4] 检查 I2C 接口..."
if [ -e /dev/i2c-1 ]; then
    echo "  ✅ I2C-1 已启用"
else
    echo "  ⚠  I2C 未启用！请运行："
    echo "     sudo raspi-config → Interface Options → I2C → Yes"
    echo "     然后重启 sudo reboot"
fi

# 4. 扫描 I2C 设备
echo "[4/4] 扫描 I2C 设备..."
if command -v i2cdetect &> /dev/null; then
    echo "  I2C 总线1上的设备："
    i2cdetect -y 1
    echo ""
    echo "  如果看到 0x40，说明 PCA9685 已连接 ✅"
else
    echo "  ⚠  i2cdetect 不可用"
fi

echo ""
echo "=========================================="
echo "  配置完成！运行程序："
echo "  python3 main_tracker.py"
echo "  python3 main_tracker.py --demo       # 无摄像头演示"
echo "  python3 main_tracker.py --no-display # 无头模式"
echo "=========================================="
