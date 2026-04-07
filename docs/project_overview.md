# Project Overview

## Project Identity

This repository implements a real-time Stewart-platform solar tracker on Raspberry Pi/Linux userspace.

The system processes a camera stream, detects the dominant light source, converts image-space error into platform tilt and pan setpoints, maps those setpoints to actuator commands, applies output conditioning, and drives the platform through a servo-based actuation system.

---

## End-to-End Behaviour

The automatic runtime pipeline is:

```text
Camera → SunTracker → Controller → ManualImuCoordinator → Kinematics3RRS → ActuatorManager → ServoDriver
```

This staged flow is orchestrated in `src/system/SystemManager.cpp`.

Manual mode uses the same downstream actuation path.

```text
GUI valueChanged / ADS1115 callback
→ store latest manual state
→ control thread tick
→ ManualImuCoordinator builds manual setpoint
→ Kinematics3RRS → ActuatorManager → ServoDriver
```

This means manual mode is continuous without turning the GUI into the timing source.

---

## What the repo  Supports

The  repository  supports:

- automatic tracking
- manual mode from GUI or potentiometers
- IMU observation or live correction path
- headless Linux event-loop execution
- Qt GUI execution
- simulated camera fallback
- software unit tests
- Raspberry Pi hardware smoke-test workflow
- latency CSV generation
- Doxygen generation
- release packaging workflow

---

## Main Repository Subsystems

### Runtime and composition
- `src/main.cpp`
- `src/qt/main_qt.cpp`
- `src/app/Application.cpp`
- `src/app/LinuxEventLoop.cpp`
- `src/app/CliController.cpp`
- `src/app/SystemFactory.cpp`
- `src/app/AppConfig.cpp`

### Vision and control
- `src/vision/SunTracker.cpp`
- `src/control/Controller.cpp`
- `src/control/Kinematics3RRS.cpp`
- `src/control/ManualInputMapper.cpp`
- `src/control/ImuFeedbackMapper.cpp`
- `src/control/ImuTiltEstimator.cpp`
- `src/control/ManualImuCoordinator.cpp`

### Actuation
- `src/actuators/ActuatorManager.cpp`
- `src/actuators/ServoDriver.cpp`
- `src/actuators/PCA9685.cpp`

### Hardware/backend support
- `src/sensors/SimulatedPublisher.cpp`
- `src/sensors/LibcameraPublisher.cpp`
- `src/sensors/manual/ADS1115ManualInput.cpp`
- `src/sensors/imu/Mpu6050Publisher.cpp`
- `src/hal/LinuxI2CDevice.cpp`

### System orchestration
- `src/system/SystemManager.cpp`
- `src/system/SystemManager.hpp`
- `src/system/TrackerState.hpp`

### GUI
- `src/qt/MainWindow.cpp`
- `src/qt/MainWindow.hpp`

---

## Runtime Modes and Backends

Configuration supports:

- `CameraBackend::Simulated`
- `CameraBackend::Libcamera`
- `StartupMode::Auto`
- `StartupMode::Manual`
- `ManualInputBackend::ADS1115`
- `ImuBackend::Mpu6050Gpio`
- `ImuFeedbackMode::Disabled`
- `ImuFeedbackMode::Shadow`
- `ImuFeedbackMode::Live`

Manual mode also distinguishes command ownership between:

- `ManualCommandSource::Gui`
- `ManualCommandSource::Pot`


