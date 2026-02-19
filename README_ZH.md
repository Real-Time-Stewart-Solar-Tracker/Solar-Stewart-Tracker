# 3RRS 视觉追踪系统 - C++版本

[English Version (README_EN.md)](README_EN.md)

基于树莓派5和OpenCV的3RRS并联机构视觉追踪系统。

---

## 📋 项目概述

本项目使用C++17实现完整的视觉追踪控制系统：
- **PCA9685舵机驱动**：通过I2C控制三个舵机（RAII资源管理）
- **3RRS逆运动学**：完全对齐MATLAB sun211.m算法
- **PID控制器**：实现平台姿态控制
- **OpenCV视觉系统**：HSV颜色空间红色目标检测（回调推送模式）
- **TrackerController**：事件驱动的控制核心（依赖注入 + 回调链）

---

## 🏗️ SOLID 设计原则

本项目严格遵循 SOLID 面向对象设计原则：

### Single Responsibility (单一职责)
每个类仅负责一个独立功能：

| 类 | 职责 |
|----|------|
| `PCA9685` | I2C通信和PWM舵机控制 |
| `RRSKinematics` | 3RRS并联机构逆运动学求解 |
| `PIDController` | PID控制算法计算 |
| `VisionSystem` | 摄像头采集和颜色检测 |
| `TrackerController` | 组件协调和控制流程 |

### Open/Closed (开闭原则)
- 通过抽象接口扩展功能，无需修改现有代码
- 例如：可以实现新的 `IServoDriver` 来支持不同的舵机驱动板

### Liskov Substitution (里氏替换)
- 所有接口实现都可以无缝替换
- 例如：可以用 MockServoDriver 替换 PCA9685 进行单元测试

### Interface Segregation (接口隔离)
- `IServoDriver`: 仅暴露 `setServoAngle()` 和 `close()`
- `IKinematicsSolver`: 仅暴露 `solve()`
- `IVisionSource`: 仅暴露 `start()`, `stop()`, `setCallback()`, `cleanup()`

### Dependency Inversion (依赖倒置)
- `TrackerController` 依赖抽象接口而非具体实现
- 通过构造函数依赖注入 `unique_ptr<IServoDriver>` 等

---

## ⚡ 实时架构设计

### 事件驱动架构（非轮询）

```
┌─────────────────┐    回调通知      ┌───────────────────────┐
│   VisionSystem  │ ──────────────→ │   TrackerController   │
│ (后台采集线程)    │  onVisionUpdate │ (condition_variable   │
│ libcamera管道I/O │                 │  定时器线程 50Hz)      │
└─────────────────┘                 │                       │
                                    │  PID → IK → Servo    │
                                    └───────────────────────┘

主线程: sigwait(SIGINT/SIGTERM) — 零轮询等待退出
```

### 关键实时技术
- **回调链**: Vision → `onVisionUpdate()` → PID → IK → Servo
- **条件变量定时器**: `condition_variable::wait_for` 实现精确50Hz控制频率（可被stop信号立即唤醒）
- **信号等待**: 主线程使用 `sigwait()` 阻塞等待退出信号，零CPU消耗
- **线程安全**: `std::mutex` + `std::atomic` 保护所有共享状态

### 延迟分析

| 环节 | 耗时 | 说明 |
|------|------|------|
| 摄像头帧间隔 | ~33ms | 30fps libcamera管道 |
| 视觉处理 | ~5ms | HSV转换 + 形态学 + 轮廓检测 |
| 回调传递 | <0.1ms | mutex lock/unlock |
| PID计算 | <0.1ms | 浮点运算 |
| IK求解 | ~0.5ms | 3组三角函数 + Eigen矩阵运算 |
| I2C舵机通信 | ~1ms | 3通道 × 5字节写入 |
| **端到端延迟** | **~40ms** | **满足实时追踪需求** |

---

## 💾 内存管理

- 所有组件使用 `std::unique_ptr` 管理（RAII）
- 零 `new`/`delete` 调用
- PCA9685 支持 move 语义（move-only类型）
- 完全无内存泄漏

---

## 🚀 快速开始

### 1. 安装依赖

```bash
# 更新系统
sudo apt-get update

# 安装编译工具和库
sudo apt-get install -y \
    build-essential \
    cmake \
    libi2c-dev \
    libopencv-dev \
    libeigen3-dev

# 安装 Google Test（可选，用于单元测试）
sudo apt-get install -y libgtest-dev
cd /usr/src/gtest && sudo cmake . && sudo make && sudo cp lib/*.a /usr/lib/

# 启用I2C（如果未启用）
sudo raspi-config
# 选择: Interfacing Options > I2C > Enable
```

### 2. 编译项目

```bash
cd cpp_tracker
mkdir -p build && cd build
cmake ..
make
```

编译产物：
- `main_tracker` — 主程序
- `test_servos` — 舵机硬件测试
- `test_circular` — 圆形轨迹测试
- `test_pid` — PID单元测试（需要GTest）
- `test_kinematics_unit` — IK单元测试（需要GTest）

### 3. 运行单元测试

```bash
cd build
ctest --verbose
```

### 4. 运行主程序

```bash
./main_tracker
# 按 Ctrl+C 优雅退出
```

---

## 📁 项目结构

```
cpp_tracker/
├── CMakeLists.txt              # CMake配置 (C++17 + GTest)
├── README.md                   # 本文档
├── include/                    # 头文件
│   ├── interfaces.hpp          # SOLID抽象接口定义
│   ├── pca9685.hpp             # PCA9685舵机驱动 (IServoDriver)
│   ├── kinematics.hpp          # 3RRS逆运动学 (IKinematicsSolver)
│   ├── pid_controller.hpp      # PID控制器
│   ├── vision_system.hpp       # OpenCV视觉系统 (IVisionSource)
│   └── tracker_controller.hpp  # 事件驱动控制器
├── src/                        # 源文件
│   ├── pca9685.cpp
│   ├── kinematics.cpp
│   ├── pid_controller.cpp
│   ├── vision_system.cpp
│   ├── tracker_controller.cpp
│   └── main_tracker.cpp        # 主程序入口
└── tests/                      # 测试
    ├── test_pid.cpp             # PID单元测试 (GTest)
    ├── test_kinematics.cpp      # IK单元测试 (GTest)
    ├── test_servos.cpp          # 舵机硬件测试
    └── test_circular.cpp        # 圆形轨迹硬件测试
```

---

## 🔧 硬件连接

### I2C连接（PCA9685）
| 树莓派引脚 | PCA9685 |
|-----------|---------|
| GPIO2 (SDA) | SDA |
| GPIO3 (SCL) | SCL |
| GND | GND |

### 舵机电源
- PCA9685的V+接5-6V电源正极
- GND接电源负极和树莓派GND

### 舵机连接
- 通道0 → 舵机1
- 通道1 → 舵机2
- 通道2 → 舵机3

### 摄像头
- CSI摄像头连接到树莓派CSI接口

---

## ⚙️ 系统参数

### 几何参数
- 静平台半径 Rb: 0.20m
- 动平台半径 Rp: 0.12m
- 平台高度 h: 0.18m
- 主动臂长 L1: 0.10m
- 从动臂长 L2: 0.18m

### 控制参数
- **PID增益**: Kp=0.3, Ki=0.0, Kd=0.0
- **最大倾斜**: 35°
- **控制频率**: 50Hz
- **舵机平滑系数**: 0.15

### 视觉参数
- **分辨率**: 640x480
- **HSV红色检测**:
  - 区间1: H[0,10], S[50,255], V[50,255]
  - 区间2: H[160,180], S[50,255], V[50,255]
- **最小轮廓面积**: 100像素
- **平滑窗口**: 5帧

---

## 🧪 测试策略

### 单元测试 (Google Test)

**PIDController 测试** (`tests/test_pid.cpp`):
- 初始状态验证
- 比例/积分/微分控制响应
- 输出限幅（正负方向）
- Reset功能
- 参数存储验证
- 边界条件（dt=0）

**RRSKinematics 测试** (`tests/test_kinematics.cpp`):
- 零角度中位验证
- 输出范围安全性
- 连续性验证（无突变）
- Pitch/Roll对称性
- 极限角稳定性
- 收敛性验证

### 硬件集成测试

- `test_servos` — 验证舵机I2C通信和运动
- `test_circular` — 验证IK+舵机协同的圆形轨迹

---

## 🐛 常见问题

### 1. 编译错误：找不到OpenCV
```bash
sudo apt-get install libopencv-dev
```

### 2. 编译错误：找不到Eigen3
```bash
sudo apt-get install libeigen3-dev
```

### 3. GTest未找到
```bash
sudo apt-get install libgtest-dev
cd /usr/src/gtest && sudo cmake . && sudo make && sudo cp lib/*.a /usr/lib/
```

### 4. 运行时错误：Failed to open I2C bus
```bash
# 确认I2C已启用
ls /dev/i2c-*
sudo raspi-config  # 启用I2C
```

### 5. 运行时错误：无法打开摄像头
```bash
libcamera-hello  # 检测摄像头
```

---

## 📄 许可

MIT License

---

## 👨‍💻 开发信息

- **C++版本**: C++17
- **测试平台**: 树莓派5 (ARM64)
- **OpenCV版本**: 4.x
- **CMake版本**: >= 3.10
- **测试框架**: Google Test
