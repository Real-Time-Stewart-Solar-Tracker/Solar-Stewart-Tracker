#!/usr/bin/env python3
"""
test_circular_ik.py - 圆形轨迹逆运动学测试
使用3RRS逆运动学，让平台按圆形轨迹运动，验证舵机协同工作

使用方法：
  python3 test_circular_ik.py

硬件连接：
  树莓派 GPIO2(SDA) → PCA9685 SDA
  树莓派 GPIO3(SCL) → PCA9685 SCL
  PCA9685 V+ → 5~6V 舵机电源
  PCA9685 通道 0/1/2 → 舵机 1/2/3
"""

import time
import math
import signal
import sys
import numpy as np

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
#  3RRS 逆运动学（精确移植自 sun211.m）
# ============================================================
class RRSKinematics:
    """
    3RRS 并联平台逆运动学。
    所有几何参数和算法逻辑与 sun211.m 完全一致。
    """

    def __init__(self):
        # ===== 几何参数（对齐 sun211.m 第53行）=====
        self.Rb = 0.20   # 静平台半径
        self.Rp = 0.12   # 动平台半径
        self.h  = 0.18   # 平台高度
        self.L1 = 0.10   # 主动臂长
        self.L2 = 0.18   # 从动臂长

        # 基座与平台铰链角度 [0°, 120°, 240°]
        base_deg = [0, 120, 240]
        self.base_angles = [math.radians(a) for a in base_deg]

        # 基座铰链点 Bi (3×3)
        self.B = np.array([
            [self.Rb * math.cos(a), self.Rb * math.sin(a), 0.0]
            for a in self.base_angles
        ])

        # 动平台局部坐标 P_local (3×3)
        self.P_local = np.array([
            [self.Rp * math.cos(a), self.Rp * math.sin(a), 0.0]
            for a in self.base_angles
        ])

        # 平台中心初始位置
        self.p0 = np.array([0.0, 0.0, self.h])

        # ===== 舵机映射参数（对齐 sun211.m 第14~16行）=====
        self.servo_offset = [90, 90, 90]       # 舵机中位
        self.servo_dir    = [-1, -1, -1]        # 方向映射
        self.servo_min    = 0
        self.servo_max    = 180

        # IK 状态：前一步关节角（用于分支连续性选择）
        self._q2_prev = [0.0, 0.0, 0.0]

        # 兜底角度（防止 IK 失败时输出 0）
        self._last_valid = list(self.servo_offset)

    @staticmethod
    def eul_zyx(yaw_deg, pitch_deg, roll_deg):
        """
        ZYX 欧拉角旋转矩阵（对齐 sun211.m eulZYX 函数）。
        输入单位：度
        """
        cy, sy = math.cos(math.radians(yaw_deg)),   math.sin(math.radians(yaw_deg))
        cp, sp = math.cos(math.radians(pitch_deg)),  math.sin(math.radians(pitch_deg))
        cr, sr = math.cos(math.radians(roll_deg)),    math.sin(math.radians(roll_deg))

        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])

        return Rz @ Ry @ Rx

    @staticmethod
    def _choose_closest(q1, q2, q_prev):
        """对齐 sun211.m chooseClosest：选择离 q_prev 更近的分支。"""
        def wrap(x):
            return (x + math.pi) % (2 * math.pi) - math.pi
        q1 = wrap(q1)
        q2 = wrap(q2)
        q_prev = wrap(q_prev)
        return q1 if abs(q1 - q_prev) <= abs(q2 - q_prev) else q2

    @staticmethod
    def _other_branch(q1, q2, q_cur):
        """对齐 sun211.m otherBranch：返回另一个分支。"""
        def wrap(x):
            return (x + math.pi) % (2 * math.pi) - math.pi
        q1 = wrap(q1)
        q2 = wrap(q2)
        q_cur = wrap(q_cur)
        return q2 if abs(q_cur - q1) < abs(q_cur - q2) else q1

    def solve(self, pitch_deg, roll_deg):
        """
        逆运动学求解。
        输入：平台姿态 (pitch, roll) 单位度
        输出：[servo1_deg, servo2_deg, servo3_deg] 整数角度
        
        算法逻辑逐行对齐 sun211.m 第157~205行。
        """
        # 旋转矩阵（yaw = 0）
        R = self.eul_zyx(0, pitch_deg, roll_deg)

        # 动平台各铰链点在世界坐标系中的位置
        P = (R @ self.P_local.T).T + self.p0  # (3, 3)

        # 用上一次有效角度兜底
        result = list(self._last_valid)

        for i in range(3):
            Bi = self.B[i]
            Pi = P[i]
            r_vec = Pi - Bi

            # 局部坐标系 (ex, ey, ez)
            theta = math.atan2(Bi[1], Bi[0])
            ex = np.array([math.cos(theta), math.sin(theta), 0.0])
            ez = np.array([0.0, 0.0, 1.0])
            ey = np.cross(ez, ex)

            x     = float(np.dot(r_vec, ex))
            z     = float(r_vec[2])
            yperp = float(np.dot(r_vec, ey))

            # 解算关节角 q2
            C   = (x*x + yperp*yperp + z*z + self.L1**2 - self.L2**2) / (2 * self.L1)
            Rxz = math.hypot(x, z)

            if Rxz < 1e-6:
                continue  # 继续用兜底角度

            ratio = max(-1.0, min(1.0, C / Rxz))
            a     = math.acos(ratio)
            gamma = math.atan2(z, x)

            # 选择最近分支
            q2c = self._choose_closest(gamma + a, gamma - a, self._q2_prev[i])

            # dot < 0 构型检查（对齐 sun211.m 第190~193行）
            Ki_joint = Bi + self.L1 * (math.cos(q2c) * ex + math.sin(q2c) * ez)
            if np.dot(Ki_joint - Bi, ex) < 0:
                q2c = self._other_branch(gamma + a, gamma - a, q2c)

            self._q2_prev[i] = q2c

            # 机构角 → 舵机角（对齐 sun211.m 第197~198行）
            servo_deg = self.servo_offset[i] + self.servo_dir[i] * math.degrees(q2c)
            servo_deg = max(self.servo_min, min(self.servo_max, round(servo_deg)))
            result[i] = servo_deg

        self._last_valid = list(result)
        return result


# ============================================================
#  主程序
# ============================================================
def main():
    print("=" * 60)
    print("  3RRS 圆形轨迹逆运动学测试")
    print("=" * 60)

    # ---------- 1. 初始化 PCA9685 ----------
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

    # ---------- 2. 初始化逆运动学 ----------
    ik = RRSKinematics()
    print("[初始化] 3RRS逆运动学引擎就绪")

    # ---------- 3. 圆形轨迹参数 ----------
    # 调整这些参数可以改变圆形轨迹的特性
    radius = 20.0       # 圆的半径（度）- 控制倾斜幅度
    period = 3.0        # 圆的周期（秒）- 控制转一圈的时间
    dt = 0.02           # 时间步长（秒）- 50Hz 控制频率

    print(f"\n[轨迹参数]")
    print(f"  圆形半径: {radius}°")
    print(f"  旋转周期: {period}秒")
    print(f"  控制频率: {1/dt:.0f}Hz")

    # ---------- 4. 优雅退出处理 ----------
    running = True
    def signal_handler(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("\n[开始] 平台将按圆形轨迹运动")
    print("       按 Ctrl+C 退出\n")

    # ---------- 5. 主控制循环 ----------
    t = 0.0
    loop_count = 0

    try:
        while running:
            t_start = time.time()

            # 计算圆形轨迹上的 pitch 和 roll
            # pitch = radius * sin(ωt)
            # roll  = radius * cos(ωt)
            omega = 2 * math.pi / period  # 角速度
            pitch = radius * math.sin(omega * t)
            roll  = radius * math.cos(omega * t)

            # 逆运动学求解
            servo_angles = ik.solve(pitch, roll)

            # 发送到舵机
            for ch in range(3):
                pca.set_servo_angle(ch, servo_angles[ch])

            # 终端输出（约10Hz）
            loop_count += 1
            if loop_count % 5 == 0:
                # 计算当前在圆上的角度位置（0-360度）
                circle_angle = (omega * t * 180 / math.pi) % 360
                print(f"[圆周:{circle_angle:6.1f}°] "
                      f"pitch={pitch:+6.1f}° roll={roll:+6.1f}° → "
                      f"S1={servo_angles[0]:3d}° S2={servo_angles[1]:3d}° S3={servo_angles[2]:3d}°")

            # 更新时间
            t += dt

            # 控制循环频率
            elapsed = time.time() - t_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass

    # ---------- 6. 清理退出 ----------
    print("\n\n[退出] 舵机回中...")
    for ch in range(3):
        pca.set_servo_angle(ch, 90)
    time.sleep(0.5)
    pca.close()
    print("[完成] ✅ 已安全退出")


if __name__ == "__main__":
    main()
