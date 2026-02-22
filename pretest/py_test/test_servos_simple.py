#!/usr/bin/env python3
"""
test_servos_simple.py - 简单舵机测试脚本
让三个舵机在0-60度之间慢慢循环运动

使用方法：
  python3 test_servos_simple.py

硬件连接：
  树莓派 GPIO2(SDA) → PCA9685 SDA
  树莓派 GPIO3(SCL) → PCA9685 SCL
  PCA9685 V+ → 5~6V 舵机电源
  PCA9685 通道 0/1/2 → 舵机 1/2/3
"""

import time
import signal
import sys

# ============================================================
#  PCA9685 I2C 舵机驱动
# ============================================================
class PCA9685:
    """直接通过 smbus 操作 PCA9685 寄存器，驱动舵机。"""

    # 寄存器地址
    MODE1       = 0x00
    PRESCALE    = 0xFE
    LED0_ON_L   = 0x06

    # 舵机脉宽参数（微秒）
    SERVO_MIN_US = 500   # 0°
    SERVO_MAX_US = 2500  # 180°

    def __init__(self, address=0x40, bus_num=1, freq=50):
        """
        初始化 PCA9685。
        :param address: I2C 地址（默认 0x40）
        :param bus_num: I2C 总线号（树莓派5 通常为 1）
        :param freq:    PWM 频率（舵机用 50Hz）
        """
        try:
            import smbus2
            self.bus = smbus2.SMBus(bus_num)
        except ImportError:
            import smbus
            self.bus = smbus.SMBus(bus_num)

        self.address = address
        self._init_device(freq)
        print(f"[PCA9685] 初始化完成 addr=0x{address:02X} freq={freq}Hz")

    def _init_device(self, freq):
        """复位并设置 PWM 频率。"""
        # 进入 SLEEP 模式
        self.bus.write_byte_data(self.address, self.MODE1, 0x10)
        time.sleep(0.005)

        # 计算 prescale: round(25MHz / (4096 * freq)) - 1
        prescale = round(25_000_000 / (4096 * freq)) - 1
        self.bus.write_byte_data(self.address, self.PRESCALE, prescale)

        # 退出 SLEEP，启用自增
        self.bus.write_byte_data(self.address, self.MODE1, 0x20)
        time.sleep(0.005)

        self._period_us = 1_000_000 / freq  # 一个周期的微秒数

    def set_pwm(self, channel, on, off):
        """设置指定通道的 PWM on/off 值（0~4095）。"""
        reg = self.LED0_ON_L + 4 * channel
        self.bus.write_byte_data(self.address, reg,     on & 0xFF)
        self.bus.write_byte_data(self.address, reg + 1, on >> 8)
        self.bus.write_byte_data(self.address, reg + 2, off & 0xFF)
        self.bus.write_byte_data(self.address, reg + 3, off >> 8)

    def set_servo_angle(self, channel, degree):
        """
        设置舵机角度（0~180°）。
        内部将角度转换为脉宽，再转换为 PWM 计数值。
        """
        degree = max(0, min(180, degree))
        pulse_us = self.SERVO_MIN_US + (self.SERVO_MAX_US - self.SERVO_MIN_US) * degree / 180.0
        counts = int(pulse_us / self._period_us * 4096)
        self.set_pwm(channel, 0, counts)

    def close(self):
        """关闭 I2C 总线。"""
        try:
            # 所有通道输出关闭
            for ch in range(3):
                self.set_pwm(ch, 0, 0)
            self.bus.close()
        except Exception:
            pass


# ============================================================
#  主程序
# ============================================================
def main():
    print("=" * 50)
    print("  简单舵机测试程序 - 0~60度循环运动")
    print("=" * 50)

    # 初始化 PCA9685
    try:
        pca = PCA9685(address=0x40, bus_num=1, freq=50)
    except Exception as e:
        print(f"[错误] PCA9685 初始化失败: {e}")
        print("请检查：")
        print("  1. I2C 是否已启用（sudo raspi-config）")
        print("  2. PCA9685 连接是否正确")
        print("  3. 是否安装了 smbus2: pip3 install smbus2")
        return

    # 舵机归中（90°初始位置）
    print("\n[初始化] 舵机归中到 90°...")
    for ch in range(3):
        pca.set_servo_angle(ch, 90)
    time.sleep(1.0)

    # 优雅退出处理
    running = True
    def signal_handler(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("\n[开始] 三个舵机将在0~60度之间慢慢循环运动")
    print("       按 Ctrl+C 退出\n")

    # 参数设置
    min_angle = 0      # 最小角度
    max_angle = 60     # 最大角度
    step = 1           # 每次变化的角度（度）
    delay = 0.05       # 每步延迟时间（秒）- 调整这个值可以改变速度

    current_angle = min_angle
    direction = 1  # 1表示增加，-1表示减少

    try:
        while running:
            # 设置所有三个舵机到当前角度
            for ch in range(3):
                pca.set_servo_angle(ch, current_angle)
            
            print(f"\r当前角度: {current_angle:3d}°  ", end='', flush=True)
            
            # 更新角度
            current_angle += step * direction
            
            # 检查边界并反转方向
            if current_angle >= max_angle:
                current_angle = max_angle
                direction = -1
            elif current_angle <= min_angle:
                current_angle = min_angle
                direction = 1
            
            time.sleep(delay)

    except KeyboardInterrupt:
        pass

    # 清理退出
    print("\n\n[退出] 舵机回中...")
    for ch in range(3):
        pca.set_servo_angle(ch, 90)
    time.sleep(0.5)
    pca.close()
    print("[完成] ✅ 已安全退出")


if __name__ == "__main__":
    main()
