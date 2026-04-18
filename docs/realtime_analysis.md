# Realtime Analysis

## Purpose

This document describes the realtime execution model and timing behaviour of the system.

---

## 1. Runtime Model

The system operates as a Linux userspace, event-driven runtime built on:

- poll() on file descriptors for application-level events  
- blocking std::condition_variable waits for inter-thread communication  
- GPIO edge-triggered callbacks for hardware sensor events  
- explicit state machine transitions  

No component introduces polling loops, sleep()-based pacing, or busy-waiting in the processing path.

---

## 2. Application Event Loop — LinuxEventLoop

src/app/LinuxEventLoop.cpp implements the headless runtime lifecycle. It multiplexes event sources using a single poll() call with an infinite timeout (-1).

File descriptors:

- signalfd → SIGINT / SIGQUIT / SIGHUP / SIGTERM → clean shutdown  
- timerfd (CLOCK_MONOTONIC) → configurable tick (default 30 Hz) → CLI servicing  
- stdin (dup'd) → terminal input  

The loop remains blocked in the kernel until an event occurs.

---

## 3. Inter-Thread Queues — ThreadSafeQueue

src/common/ThreadSafeQueue.hpp provides blocking data transfer between worker threads using std::mutex and std::condition_variable with wait_pop().

Two queues are defined in SystemManager:

- frame_q_ → ThreadSafeQueue<FrameEvent> → capacity 2 → push_latest()  
- cmd_q_ → ThreadSafeQueue<ActuatorCommand> → capacity 8 → push_latest()  

The push_latest() policy ensures consumers process the most recent data when under load. Queue sizes remain bounded.

---

## 4. Worker Threads

SystemManager owns two worker threads. A third dedicated thread is owned by GuiManualDispatcher.

Control thread (controlLoop_):

- blocks on frame_q_.wait_pop()  
- processes frames through SunTracker and Controller in automatic modes (SEARCHING / TRACKING)  
- drains frames without processing in MANUAL and FAULT states  
- is not involved in manual command generation  

GuiManualDispatcher thread:

- blocks on a bounded freshest-data queue (capacity 1)  
- wakes immediately when setManualSetpoint() is called  
- dispatches directly to Kinematics3RRS, independent of camera-frame timing  

Pot-manual path (no dedicated thread):

- ADS1115 ALERT/RDY GPIO edge fires onManualPotSample_() in the ADS1115 callback thread  
- dispatches directly to Kinematics3RRS from that callback context  

Actuator thread (actuatorLoop_):

- blocks on cmd_q_.wait_pop()  
- applies ActuatorManager → ServoDriver  
- no alternative actuation path exists  

---

## 5. Sensor Wakeups

Camera:

- SimulatedPublisher uses timerfd + poll() + eventfd  
- LibcameraPublisher delivers frames via callback-based backend  

Manual input:

- ADS1115ManualInput uses ALERT/RDY GPIO edge  
- callback dispatches directly to Kinematics3RRS in pot-manual mode  

IMU:

- Mpu6050Publisher uses GPIO data-ready interrupt  
- samples forwarded to coordinator  

---

## 6. Data Flow

Automatic path:

Frame callback  
→ frame_q_.push_latest(...)  
→ control thread wait_pop()  
→ SunTracker  
→ Controller  
→ ManualImuCoordinator::applyImuCorrection(...)  
→ Kinematics3RRS  
→ cmd_q_.push_latest(...)  
→ actuator thread wait_pop()  
→ ActuatorManager  
→ ServoDriver  

Pot-manual path:

ADS1115 ALERT/RDY GPIO edge  
→ onManualPotSample_() in ADS1115 callback thread  
→ ManualImuCoordinator builds manual setpoint  
→ applyImuCorrection  
→ Kinematics3RRS  
→ cmd_q_.push_latest(...)  
→ actuator thread wait_pop()  
→ ActuatorManager  
→ ServoDriver  

GUI-manual path:

setManualSetpoint() called (any thread)  
→ GuiManualDispatcher queue push_latest  
→ GuiManualDispatcher worker thread wakes immediately  
→ ManualImuCoordinator builds manual setpoint  
→ applyImuCorrection  
→ Kinematics3RRS  
→ cmd_q_.push_latest(...)  
→ actuator thread wait_pop()  
→ ActuatorManager  
→ ServoDriver  

All three paths share the downstream Kinematics → ActuatorManager → ServoDriver stages. The control thread handles only the automatic path. Both manual paths are independently event-driven.

---

## 7. Qt GUI Timing

Qt timers are used only for UI refresh and visualisation.

GUI slider interactions call setManualSetpoint(), which pushes to the GuiManualDispatcher queue. The dispatcher's dedicated worker thread wakes immediately and dispatches directly to Kinematics3RRS. Qt timers are never used as a control timing source.

---

## 8. IMU Policy in Manual Mode

Manual GUI input does not apply continuous IMU correction by default. Manual commands are generated directly from operator input.

IMU-assisted behaviour can be enabled through configured feedback modes.

---

## 9. Shutdown Behaviour

stop() performs:

- camera shutdown  
- backend shutdown  
- GuiManualDispatcher shutdown  
- queue termination  
- worker thread join  
- actuator neutral/park handling  
- servo driver shutdown  

Queues notify blocked threads during shutdown, ensuring clean exit.

---

## 10. Summary

The runtime follows a consistent event-driven execution model:

- all worker threads block on blocking primitives (poll, condition_variable)
- all wakeups originate from external events (file descriptors, queue notifications, GPIO interrupts)  
- no component introduces independent timing loops or polling  

The automatic path is processed by a single control thread. Both manual paths are independently event-driven: the pot path dispatches from the ADS1115 callback thread, and the GUI path dispatches from the GuiManualDispatcher worker thread. All three paths converge at Kinematics3RRS before actuation.

This structure ensures:

- separation between automatic, pot-manual, and GUI-manual timing  
- no camera-frame dependency in either manual path  
- separation between realtime processing and UI interaction  
- explicit ownership of timing and data flow