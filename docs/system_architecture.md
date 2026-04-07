# System Architecture

## Overview

The system is structured as a staged, event-driven pipeline in Linux userspace. Frame acquisition, tracking, control, kinematic mapping, and actuator output are separated into distinct classes so that each stage has a clear responsibility and a bounded interface. This is consistent with the taught architecture of sensor events propagating through callbacks and typed data transformations to control outputs.

The implemented automatic processing path is:

**ICamera → SunTracker → Controller → ManualImuCoordinator → Kinematics3RRS → ActuatorManager → ServoDriver**

`SystemManager` orchestrates this pipeline and manages runtime state. `BackendCoordinator` owns hardware backend lifecycle. `GuiManualDispatcher` provides the event-driven GUI manual path. `SystemFactory` assembles the runtime graph.

This class structure is stronger than a monolithic alternative because it separates:

- sensor acquisition from estimation
- estimation from control
- control from mechanism-specific mapping
- mechanism mapping from actuator safety conditioning
- actuator conditioning from final hardware output
- runtime orchestration from backend implementation details

That separation improves maintainability, reduces coupling, and makes the realtime event flow explicit.

---

## Architectural Rationale

The core design decision is to preserve a forward-only event path through specialised classes instead of building one large tracker class. In this repository, each stage transforms one well-defined form of data into the next.

For automatic tracking:

- the camera backend emits frames
- the tracker estimates target position and confidence
- the controller converts that estimate into a platform command
- the manual/IMU coordination layer applies optional correction
- the kinematics layer maps platform motion into actuator-space commands
- the actuator manager applies command conditioning and output safety policy
- the servo driver performs final calibrated output to the physical device

For manual mode, both input paths are independently event-driven:

**Pot-driven manual:** ADS1115 ALERT/RDY GPIO edge → `onManualPotSample_()` → setpoint built and dispatched directly to `Kinematics3RRS`. Timing is driven by the ADS1115 conversion rate, independent of camera frames.

**GUI-driven manual:** Qt slider → `setManualSetpoint()` → `GuiManualDispatcher::setSetpoint()` → push to bounded queue → `GuiManualDispatcher` worker thread wakes immediately → setpoint built via `ManualImuCoordinator` and dispatched directly to `Kinematics3RRS`. Timing is driven by operator input rate, independent of camera frames.

The control thread handles only the automatic path. It is not involved in either manual path.

This staged structure is appropriate because the taught approach is event-driven userspace code built around callbacks, blocking waits, and clean class interfaces rather than polling loops or single-threaded delay-based control.

---

## Diagram 1 — UML Class Diagram

```mermaid
classDiagram
direction LR

class SystemManager {
  +start() bool
  +stop() void
  +state() TrackerState
  +enterManual() void
  +exitManual() void
  +setManualSetpoint(tilt_rad, pan_rad) void
  +setTrackerThreshold(thr) void
  +setManualCommandSource(src) void
  +registerFrameObserver(cb) void
  +registerEstimateObserver(cb) void
  +registerSetpointObserver(cb) void
  +registerCommandObserver(cb) void
  +registerLatencyObserver(cb) void
  -onFrame_(fe) void
  -controlLoop_() void
  -actuatorLoop_() void
  -setState_(s) void
  -applyNeutralOnce_() void
  -applyParkOnce_(servo_deg) void
}

class BackendCoordinator {
  +start(log) StartResult
  +stop() void
}

class GuiManualDispatcher {
  +start() void
  +stop() void
  +setSetpoint(tilt_rad, pan_rad) void
  +registerSetpointObserver(cb) void
}

class ICamera {
  <<interface>>
  +registerFrameCallback(cb) void
  +start() bool
  +stop() void
  +isRunning() bool
}

class LibcameraPublisher
class SimulatedPublisher
ICamera <|.. LibcameraPublisher
ICamera <|.. SimulatedPublisher

class ThreadSafeQueue~T~ {
  +ThreadSafeQueue(capacity)
  +wait_pop() optional~T~
  +push_latest(item) bool
  +stop() void
  +reset() void
  +clear() void
}

class SunTracker {
  +registerEstimateCallback(cb) void
  +setThreshold(thr) void
  +onFrame(fe) void
}

class Controller {
  +registerSetpointCallback(cb) void
  +onEstimate(est) void
}

class Kinematics3RRS {
  +registerCommandCallback(cb) void
  +onSetpoint(sp) void
}

class ActuatorManager {
  +registerSafeCommandCallback(cb) void
  +onCommand(cmd) void
}

class ServoDriver {
  +start() bool
  +stop() void
  +apply(cmd) void
}

class LatencyMonitor {
  +onCapture(frame_id, t) void
  +onEstimate(frame_id, t) void
  +onControl(frame_id, t) void
  +onActuate(frame_id, t) void
}

class ManualImuCoordinator {
  +setManualCommandSource(src) void
  +manualCommandSource() ManualCommandSource
  +updateImuSample(sample) void
  +applyImuCorrection(sp) PlatformSetpoint
  +buildManualSetpointFromPot(sample, state, frame_id, t, sp) bool
  +buildManualSetpointFromGui(tilt, pan, state, frame_id, t, sp) bool
}

class ADS1115ManualInput {
  +registerCallback(cb) void
  +start() bool
  +stop() void
}

class Mpu6050Publisher {
  +registerEventCallback(cb) void
  +start() bool
  +stop() void
}

SystemManager o-- ICamera
SystemManager o-- SunTracker
SystemManager o-- Controller
SystemManager o-- Kinematics3RRS
SystemManager o-- ActuatorManager
SystemManager o-- ServoDriver
SystemManager o-- LatencyMonitor
SystemManager o-- ManualImuCoordinator
SystemManager o-- BackendCoordinator
SystemManager o-- GuiManualDispatcher
BackendCoordinator o-- ADS1115ManualInput
SystemManager o-- Mpu6050Publisher

SystemManager o-- ThreadSafeQueue~FrameEvent~ : frame_q_ cap=2
SystemManager o-- ThreadSafeQueue~ActuatorCommand~ : cmd_q_ cap=8

ICamera ..> FrameEvent : emits via callback
SunTracker ..> SunEstimate : emits via callback
Controller ..> PlatformSetpoint : emits via callback
Kinematics3RRS ..> ActuatorCommand : emits via callback
ActuatorManager ..> ActuatorCommand : emits safe callback
ADS1115ManualInput ..> ManualPotSample : emits via callback
Mpu6050Publisher ..> ImuSample : emits via callback
ManualImuCoordinator ..> PlatformSetpoint : adjusts / builds
```

---

## Diagram 2.1 — Sequence Diagram — Automatic Runtime Pipeline

```mermaid
sequenceDiagram
autonumber

participant Cam as Camera backend
participant SM as SystemManager
participant FQ as FrameQueue cap=2
participant CT as Control thread
participant ST as SunTracker
participant C as Controller
participant MIC as ManualImuCoordinator
participant K as Kinematics3RRS
participant CQ as CommandQueue cap=8
participant AT as Actuator thread
participant AM as ActuatorManager
participant SD as ServoDriver
participant LM as LatencyMonitor

SM->>Cam: registerFrameCallback(onFrame_)
SM->>ST: registerEstimateCallback(...)
SM->>C: registerSetpointCallback(...)
SM->>K: registerCommandCallback(...)
SM->>AM: registerSafeCommandCallback(...)
SM->>SD: start()
SM->>Cam: start()
SM->>SM: setState(SEARCHING)

Cam-->>SM: FrameCallback(FrameEvent)
SM->>LM: onCapture(frame_id, t_capture)
SM->>FQ: push_latest(FrameEvent)

CT->>FQ: wait_pop()
FQ-->>CT: FrameEvent
CT->>ST: onFrame(FrameEvent)

ST-->>CT: EstimateCallback(SunEstimate)
CT->>LM: onEstimate(frame_id, t_estimate)

alt confidence >= threshold
  CT->>SM: setState(TRACKING)
else confidence < threshold
  CT->>SM: setState(SEARCHING)
end

CT->>C: onEstimate(SunEstimate)
C-->>CT: SetpointCallback(PlatformSetpoint)
CT->>LM: onControl(frame_id, t_control)

CT->>MIC: applyImuCorrection(PlatformSetpoint)
MIC-->>CT: corrected PlatformSetpoint

CT->>K: onSetpoint(corrected PlatformSetpoint)
K-->>CT: CommandCallback(ActuatorCommand)
CT->>CQ: push_latest(ActuatorCommand)

AT->>CQ: wait_pop()
CQ-->>AT: ActuatorCommand
AT->>AM: onCommand(ActuatorCommand)

AM-->>AT: SafeCommandCallback(ActuatorCommand)
AT->>LM: onActuate(frame_id, t_actuate)
AT->>SD: apply(ActuatorCommand)
```

---

## Diagram 2.2 — Sequence Diagram — Manual Input + IMU Update Paths

```mermaid
sequenceDiagram
autonumber

participant ADS as ADS1115ManualInput
participant IMU as Mpu6050Publisher
participant User as Qt / CLI User
participant SM as SystemManager
participant BC as BackendCoordinator
participant GMD as GuiManualDispatcher
participant MIC as ManualImuCoordinator
participant K as Kinematics3RRS
participant CQ as CommandQueue cap=8
participant AT as Actuator thread
participant AM as ActuatorManager
participant SD as ServoDriver
participant LM as LatencyMonitor

Note over BC,ADS: BackendCoordinator owns ADS1115 and IMU lifecycle
BC->>ADS: start()
BC->>IMU: start()

IMU-->>SM: IMU callback(sample)
SM->>MIC: updateImuSample(sample)

User->>SM: enterManual()
SM->>MIC: setManualCommandSource(Pot or GUI)
SM->>SM: setState(MANUAL)

alt Pot-controlled manual mode — ADS1115 ALERT/RDY edge driven
  ADS-->>SM: ManualPotSample callback
  SM->>MIC: buildManualSetpointFromPot(sample, state, id, t, sp)
  MIC-->>SM: PlatformSetpoint
  SM->>MIC: applyImuCorrection(sp)
  MIC-->>SM: corrected PlatformSetpoint
  SM->>LM: onControl(frame_id, t_control)
  SM->>K: onSetpoint(corrected PlatformSetpoint)
  Note over SM,K: Dispatched directly in ADS1115 callback — no frame dependency
else GUI-controlled manual mode — operator input rate driven
  User->>SM: setManualSetpoint(tilt, pan)
  SM->>GMD: setSetpoint(tilt, pan)
  Note over GMD: push_latest to bounded queue — wakes immediately
  GMD->>MIC: buildManualSetpointFromGui(tilt, pan, state, id, t, sp)
  MIC-->>GMD: PlatformSetpoint
  GMD->>MIC: applyImuCorrection(sp)
  MIC-->>GMD: corrected PlatformSetpoint
  GMD->>LM: onControl(frame_id, t_control)
  GMD->>K: onSetpoint(corrected PlatformSetpoint)
  Note over GMD,K: Dispatched from GuiManualDispatcher thread — no frame dependency
end

K-->>K: CommandCallback(ActuatorCommand)
K->>CQ: push_latest(ActuatorCommand)

AT->>CQ: wait_pop()
CQ-->>AT: ActuatorCommand
AT->>AM: onCommand(ActuatorCommand)
AM-->>AT: SafeCommandCallback(ActuatorCommand)
AT->>LM: onActuate(frame_id, t_actuate)
AT->>SD: apply(ActuatorCommand)

User->>SM: exitManual()
SM->>SM: setState(SEARCHING)
```

---

## Diagram 3 — Component Diagram

```mermaid
flowchart LR
  subgraph EntryPoints["Entry Points"]
    CLI["solar_tracker (CLI)"]
    QT["solar_tracker_qt (Qt UI)"]
  end

  subgraph Core["Core Realtime Pipeline"]
    SM["SystemManager"]
    FQ["FrameQueue cap=2<br/>ThreadSafeQueue&lt;FrameEvent&gt;"]
    CQ["CommandQueue cap=8<br/>ThreadSafeQueue&lt;ActuatorCommand&gt;"]
    ST["SunTracker"]
    C["Controller"]
    MIC["ManualImuCoordinator"]
    K["Kinematics3RRS"]
    AM["ActuatorManager"]
    SD["ServoDriver"]
    LM["LatencyMonitor"]
  end

  subgraph Vision["Camera Backends"]
    ICam["ICamera"]
    Lib["LibcameraPublisher"]
    Sim["SimulatedPublisher"]
  end

  subgraph ManualInput["Manual Input"]
    ADS["ADS1115ManualInput<br/>ALERT/RDY GPIO edge"]
    GMD["GuiManualDispatcher<br/>dedicated thread + queue"]
    BC["BackendCoordinator<br/>owns ADS1115 + IMU"]
  end

  subgraph ImuFeedback["IMU Feedback"]
    MPU["Mpu6050Publisher"]
  end

  CLI --> SM
  QT --> SM

  Lib -- implements --> ICam
  Sim -- implements --> ICam
  ICam -->|Frame callback| SM

  SM --> FQ
  FQ -->|wait_pop in control thread| ST
  ST --> C
  C --> MIC
  MIC --> K
  K --> CQ
  CQ -->|wait_pop in actuator thread| AM
  AM --> SD

  ADS -->|manual sample callback| SM
  SM -->|dispatch direct to K| K
  QT -->|setManualSetpoint| SM
  SM -->|setSetpoint| GMD
  GMD -->|dispatch direct to K| K
  MPU -->|IMU callback| SM
  SM --> MIC

  SM --> LM
```

---

## Diagram 4 — Threaded Event Architecture

```mermaid
flowchart TB
  CAM["Camera backend<br/>libcamera or simulated"]
  ADS["ADS1115 manual input"]
  MPU["MPU6050 IMU"]
  UI["Qt / CLI manual input"]

  subgraph T1["Callback / producer side"]
    CAMCB["Frame callback"]
    ADSCB["Manual sample callback"]
    MPUCB["IMU callback"]
    UICB["GUI / CLI target update"]
  end

  subgraph T2["Control thread — automatic path only"]
    FQ["FrameQueue cap=2"]
    ST["SunTracker"]
    CTRL["Controller"]
    MIC["ManualImuCoordinator"]
    KIN["Kinematics3RRS"]
    CQ["CommandQueue cap=8"]
  end

  subgraph T4["Pot callback thread (ADS1115)"]
    ADSCB2["onManualPotSample_()"]
    POTK["→ Kinematics3RRS"]
  end

  subgraph T5["GuiManualDispatcher thread"]
    GUIQ["GuiManualDispatcher queue"]
    GUIK["→ Kinematics3RRS"]
  end

  subgraph T3["Actuator thread"]
    ACT["ActuatorManager"]
    SERVO["ServoDriver"]
  end

  CAM --> CAMCB --> FQ
  FQ --> ST --> CTRL --> MIC --> KIN --> CQ
  ADS --> ADSCB --> ADSCB2 --> POTK
  UI --> UICB --> GUIQ --> GUIK
  MPU --> MPUCB --> MIC
  CQ --> ACT --> SERVO
```

---

## Component Responsibilities

### SystemManager

Coordinates runtime execution, manages threads, processes incoming events, and controls system state. Manual timing is fully delegated: potentiometer commands dispatch directly from the ADS1115 callback, and GUI commands dispatch from `GuiManualDispatcher`. The control thread handles only the automatic pipeline.

### BackendCoordinator

Owns the lifecycle of optional hardware backends: ADS1115 manual input and MPU-6050/ICM-20600 IMU. Starts, stops, and owns I2C/GPIO resources independently of pipeline orchestration.

### GuiManualDispatcher

Owns a dedicated worker thread and bounded freshest-data queue for GUI manual setpoints. When `setManualSetpoint()` is called, the dispatcher wakes immediately and dispatches the setpoint to `Kinematics3RRS` without depending on camera-frame timing.

### ICamera and Camera Backends

Provide frame acquisition. Multiple implementations can be used without affecting downstream processing.

### SunTracker

Processes image data and produces a target estimate including position and confidence.

### Controller

Transforms the target estimate into a platform-level motion command.

### ManualImuCoordinator

Handles manual input ownership and optional IMU-based adjustments. It builds manual setpoints and applies IMU correction where allowed by mode/source policy.

### Kinematics3RRS

Maps platform motion commands into actuator-specific commands based on mechanism geometry.

### ActuatorManager

Applies conditioning to actuator commands such as clamping and rate limiting to ensure safe output.

### ServoDriver and PCA9685

Convert actuator commands into hardware signals and communicate with the PWM controller.

### Input Devices

- `ADS1115ManualInput` provides potentiometer-based manual control input
- `Mpu6050Publisher` provides IMU data for orientation feedback

---

## Architectural Summary

The system is built as a pipeline of independent processing stages connected through well-defined interfaces. Each stage transforms data and passes it forward without direct knowledge of internal details of other components.

This structure enables:

- predictable data flow
- clear separation between computation and hardware access
- safe multi-threaded execution
- straightforward extension and modification of individual stages

The inter-thread queues are intentionally bounded:

- Frame queue capacity is small (2) to minimise latency and prevent stale frames accumulating.
- Command queue capacity is larger (8) to absorb short bursts without dropping actuator updates prematurely.

Both queues use a latest-wins policy, ensuring that the system prioritises current data while maintaining bounded memory and predictable behaviour.