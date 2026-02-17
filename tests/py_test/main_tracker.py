#!/usr/bin/env python3
"""
main_tracker.py  —  树莓派5 全栈太阳追踪程序
==============================================
功能：CSI摄像头 → OpenCV颜色追踪 → PID → 3RRS逆运动学 → PCA9685舵机
所有 IK/PID/舵机参数严格对齐 sun211.m

运行：
  python3 main_tracker.py              # 正常模式（需要摄像头+PCA9685）
  python3 main_tracker.py --demo       # 演示模式（自动画圆，无需摄像头）
  python3 main_tracker.py --no-display # 无显示模式（无头树莓派）

硬件接线：
  树莓派 GPIO2(SDA) → PCA9685 SDA
  树莓派 GPIO3(SCL) → PCA9685 SCL
  PCA9685 V+ → 5~6V 舵机电源
  PCA9685 通道 0/1/2 → 舵机 1/2/3
"""

import time
import math
import argparse
import signal
import sys
import threading
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
        self.servo_offset = [90, 90, 90]       # 舵机中位（逆运动学计算用）
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
#  PID 控制器
# ============================================================
class PIDController:
    """
    单轴 PID 控制器。
    参数对齐 sun211.m：Kp=0.02, Ki=0, Kd=0
    """

    def __init__(self, kp=0.02, ki=0.0, kd=0.0, max_tilt=35.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_tilt = max_tilt

        self._integral = 0.0
        self._last_err = 0.0
        self._output = 0.0    # 当前累积输出（对齐 MATLAB 的 curr_roll/pitch）

    def update(self, error, dt):
        """
        PID 更新（对齐 sun211.m 第146~155行）。
        error: 目标偏差
        dt:    时间步长
        返回:  限幅后的输出角度
        """
        self._integral += error * dt
        derivative = (error - self._last_err) / dt if dt > 0 else 0.0
        self._last_err = error

        out = self.kp * error + self.ki * self._integral + self.kd * derivative
        self._output += out
        self._output = max(-self.max_tilt, min(self.max_tilt, self._output))

        return self._output

    def reset(self):
        self._integral = 0.0
        self._last_err = 0.0
        self._output = 0.0


# ============================================================
#  OpenCV HSV 颜色追踪视觉系统
# ============================================================
class VisionSystem:
    """
    使用 picamera2 + OpenCV 进行 HSV 红色目标追踪。
    在独立线程中运行，主循环通过 get_target() 获取最新结果。
    """

    # HSV 红色阈值（红色在 H 两端）
    RED_LOWER_1 = np.array([0,   100, 100])
    RED_UPPER_1 = np.array([10,  255, 255])
    RED_LOWER_2 = np.array([160, 100, 100])
    RED_UPPER_2 = np.array([180, 255, 255])

    MIN_CONTOUR_AREA = 300   # 最小轮廓面积（过滤噪声）

    def __init__(self, width=640, height=480, smooth_window=5, show_display=True):
        self.width  = width
        self.height = height
        self.show_display = show_display

        # 平滑滤波
        self._smooth_window = smooth_window
        self._history_x = []
        self._history_y = []

        # 线程安全的目标数据
        self._lock = threading.Lock()
        self._target_dx = 0.0  # 归一化偏差 [-1, 1]
        self._target_dy = 0.0
        self._target_found = False
        self._frame_for_display = None

        self._running = False
        self._thread = None

    def start(self):
        """启动视觉采集线程。"""
        self._running = True
        self._thread = threading.Thread(target=self._capture_loop, daemon=True)
        self._thread.start()

    def stop(self):
        """停止视觉线程。"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def get_target(self):
        """
        获取最新归一化目标偏差。
        返回: (found, dx, dy)
          found: 是否检测到目标
          dx:    水平偏差 [-1, 1]，正值=目标在右侧
          dy:    垂直偏差 [-1, 1]，正值=目标在下方
        """
        with self._lock:
            return self._target_found, self._target_dx, self._target_dy

    def get_display_frame(self):
        """获取最新的显示帧（带标注）。"""
        with self._lock:
            return self._frame_for_display

    def _capture_loop(self):
        """视觉采集主循环（在后台线程运行）。"""
        try:
            from picamera2 import Picamera2

            picam2 = Picamera2()
            config = picam2.create_preview_configuration(
                main={"size": (self.width, self.height), "format": "RGB888"}
            )
            picam2.configure(config)
            picam2.start()
            print(f"[Vision] Picamera2 已启动 {self.width}x{self.height}")
            time.sleep(1.0)  # 等待摄像头稳定

            while self._running:
                frame = picam2.capture_array()
                self._process_frame(frame)

            picam2.stop()
            picam2.close()

        except ImportError:
            print("[Vision] ⚠ picamera2 不可用，尝试使用 OpenCV VideoCapture...")
            self._capture_loop_cv2()
        except Exception as e:
            print(f"[Vision] 摄像头错误: {e}")

    def _capture_loop_cv2(self):
        """备用方案：使用 OpenCV VideoCapture。"""
        import cv2
        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        print(f"[Vision] OpenCV VideoCapture 已启动")

        while self._running:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.01)
                continue
            self._process_frame(frame, is_bgr=True)

        cap.release()

    def _process_frame(self, frame, is_bgr=False):
        """处理单帧图像：HSV颜色检测 + 轮廓分析。"""
        import cv2

        if is_bgr:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            display = frame.copy()
        else:
            hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
            display = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

        # 红色掩膜（双区间合并）
        mask1 = cv2.inRange(hsv, self.RED_LOWER_1, self.RED_UPPER_1)
        mask2 = cv2.inRange(hsv, self.RED_LOWER_2, self.RED_UPPER_2)
        mask = mask1 | mask2

        # 形态学去噪
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # 轮廓查找
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        found = False
        cx, cy = self.width / 2, self.height / 2

        if contours:
            # 选择最大轮廓
            biggest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(biggest)

            if area > self.MIN_CONTOUR_AREA:
                M = cv2.moments(biggest)
                if M["m00"] > 0:
                    cx = M["m10"] / M["m00"]
                    cy = M["m01"] / M["m00"]
                    found = True

                    # 绘制检测结果
                    x, y, w, h = cv2.boundingRect(biggest)
                    cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.circle(display, (int(cx), int(cy)), 6, (0, 0, 255), -1)
                    cv2.putText(display, f"({cx:.0f},{cy:.0f})",
                                (int(cx) + 10, int(cy) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # 画中心十字
        cv2.drawMarker(display, (self.width // 2, self.height // 2),
                       (255, 255, 255), cv2.MARKER_CROSS, 30, 1)

        # 归一化偏差：以画面中心为原点，[-1, 1]
        raw_dx = (cx - self.width / 2) / (self.width / 2)
        raw_dy = (cy - self.height / 2) / (self.height / 2)

        # Moving Average 平滑滤波
        if found:
            self._history_x.append(raw_dx)
            self._history_y.append(raw_dy)
            if len(self._history_x) > self._smooth_window:
                self._history_x.pop(0)
                self._history_y.pop(0)

        smooth_dx = sum(self._history_x) / len(self._history_x) if self._history_x else 0.0
        smooth_dy = sum(self._history_y) / len(self._history_y) if self._history_y else 0.0

        # 状态文字
        status = "TRACKING" if found else "SEARCHING"
        color = (0, 255, 0) if found else (0, 0, 255)
        cv2.putText(display, status, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

        # 更新共享状态
        with self._lock:
            self._target_found = found
            self._target_dx = smooth_dx
            self._target_dy = smooth_dy
            if self.show_display:
                self._frame_for_display = display

    def cleanup(self):
        """清理资源。"""
        self.stop()
        try:
            import cv2
            cv2.destroyAllWindows()
        except Exception:
            pass


# ============================================================
#  主程序
# ============================================================
def main():
    parser = argparse.ArgumentParser(description="树莓派5 3RRS太阳追踪系统")
    parser.add_argument("--demo",       action="store_true", help="演示模式（自动画圆，无需摄像头）")
    parser.add_argument("--no-display", action="store_true", help="无显示模式（无头树莓派）")
    parser.add_argument("--no-servo",   action="store_true", help="无舵机模式（仅视觉调试）")
    parser.add_argument("--i2c-bus",    type=int, default=1, help="I2C总线号（默认1）")
    parser.add_argument("--i2c-addr",   type=int, default=0x40, help="PCA9685地址（默认0x40）")
    args = parser.parse_args()

    show_display = not args.no_display
    print("=" * 50)
    print("  3RRS 太阳追踪系统 — 树莓派5全栈版")
    print("=" * 50)

    # ---------- 1. 初始化 PCA9685 ----------
    pca = None
    if not args.no_servo:
        try:
            pca = PCA9685(address=args.i2c_addr, bus_num=args.i2c_bus, freq=50)
            # 舵机归中（60°初始位置）
            for ch in range(3):
                pca.set_servo_angle(ch, 60)
            print("[Main] 舵机已归中 (60°)")
            time.sleep(0.5)
        except Exception as e:
            print(f"[Main] ⚠ PCA9685 初始化失败: {e}")
            print("[Main]   继续运行（仅终端输出角度）")
            pca = None

    # ---------- 2. 初始化 IK ----------
    ik = RRSKinematics()
    print("[Main] IK 引擎就绪")

    # ---------- 3. 初始化 PID ----------
    # 增大Kp以提高响应速度（原sun211.m: Kp=0.02）
    pid_pitch = PIDController(kp=0.3, ki=0.0, kd=0.0, max_tilt=35.0)
    pid_roll  = PIDController(kp=0.3, ki=0.0, kd=0.0, max_tilt=35.0)
    print("[Main] PID 控制器就绪 (Kp=0.3, maxTilt=35°)")

    # ---------- 4. 初始化视觉系统 ----------
    vision = None
    if not args.demo:
        vision = VisionSystem(width=640, height=480, smooth_window=5,
                              show_display=show_display)
        vision.start()
        print("[Main] 视觉系统已启动")
    else:
        print("[Main] 🎪 演示模式：自动画圆轨迹")

    # ---------- 5. 主控制循环 ----------
    dt = 0.02  # 50Hz（对齐 sun211.m）
    last_sent_angles = [None, None, None]  # 仅变化发送
    current_servo_angles = [60.0, 60.0, 60.0]  # 当前舵机实际角度（平滑插值用）
    demo_t = 0.0

    # 舵机平滑参数
    servo_smooth_factor = 0.15  # 平滑系数（0-1），越小越平滑但响应越慢

    # 优雅退出
    running = True
    def signal_handler(sig, frame):
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    print("[Main] 🚀 开始追踪！(Ctrl+C 退出)\n")

    loop_count = 0

    try:
        while running:
            t_start = time.time()

            # --- 获取目标偏差 ---
            if args.demo:
                # 演示模式：自动绕圆
                demo_t += dt
                target_pitch = 20.0 * math.sin(demo_t * 0.5)
                target_roll  = 20.0 * math.cos(demo_t * 0.5)
                # 直接设置为目标角度（跳过 PID）
                pitch = target_pitch
                roll  = target_roll
            else:
                found, dx, dy = vision.get_target()

                if found:
                    # 视觉偏差 → PID 目标
                    # dx > 0 表示目标在画面右侧 → 需要平台向右倾 → pitch 正向
                    # dy > 0 表示目标在画面下方 → 需要平台向下倾 → roll 正向
                    # 增大映射系数以提高响应幅度（原35.0）
                    target_pitch = dx * 50.0
                    target_roll  = dy * 50.0
                else:
                    target_pitch = 0.0
                    target_roll  = 0.0

                # PID 更新
                pitch = pid_pitch.update(target_pitch - pid_pitch._output, dt)
                roll  = pid_roll.update(target_roll  - pid_roll._output,  dt)

            # --- IK 解算 ---
            target_servo_angles = ik.solve(pitch, roll)

            # --- 舵机角度平滑插值 ---
            # 使用低通滤波实现平滑过渡：current = current + alpha * (target - current)
            for i in range(3):
                # 如果是第一次，直接使用目标角度
                if last_sent_angles[i] is None:
                    current_servo_angles[i] = float(target_servo_angles[i])
                else:
                    # 线性插值：逐步接近目标角度
                    angle_diff = target_servo_angles[i] - current_servo_angles[i]
                    current_servo_angles[i] += angle_diff * servo_smooth_factor

            # --- 发送到舵机（四舍五入到整数）---
            servo_angles = [round(a) for a in current_servo_angles]
            
            changed = False
            for i in range(3):
                if servo_angles[i] != last_sent_angles[i]:
                    changed = True
                    if pca:
                        pca.set_servo_angle(i, servo_angles[i])
                    last_sent_angles[i] = servo_angles[i]

            # --- 终端状态输出（约2Hz）---
            loop_count += 1
            if loop_count % 25 == 0:
                if args.demo:
                    src = "DEMO"
                elif vision:
                    found, dx, dy = vision.get_target()
                    src = f"dx={dx:+.2f} dy={dy:+.2f}" if found else "无目标"
                else:
                    src = "N/A"

                print(f"[{src}] pitch={pitch:+6.1f}° roll={roll:+6.1f}° "
                      f"→ S1={servo_angles[0]:3d}° S2={servo_angles[1]:3d}° S3={servo_angles[2]:3d}°")

            # --- 显示视觉画面 ---
            if show_display and vision:
                import cv2
                frame = vision.get_display_frame()
                if frame is not None:
                    # 在画面上叠加舵机角度信息
                    info = f"P:{pitch:+.1f} R:{roll:+.1f} S:[{servo_angles[0]},{servo_angles[1]},{servo_angles[2]}]"
                    cv2.putText(frame, info, (10, frame.shape[0] - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    cv2.imshow("Solar Tracker", frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27:  # ESC
                        running = False

            # --- 控制循环频率 50Hz ---
            elapsed = time.time() - t_start
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass

    # ---------- 6. 清理退出 ----------
    print("\n[Main] 正在退出...")
    if vision:
        vision.cleanup()
    if pca:
        # 舵机回中后关闭
        for ch in range(3):
            pca.set_servo_angle(ch, 60)
        time.sleep(0.3)
        pca.close()
    print("[Main] ✅ 已安全退出")


if __name__ == "__main__":
    main()
