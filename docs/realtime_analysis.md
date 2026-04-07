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
- stdin (dup’d) → terminal input  

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

SystemManager owns two worker threads.

Control thread (controlLoop_):

- blocks on frame_q_.wait_pop()  
- processes frames through SunTracker and Controller in automatic modes  
- builds manual setpoints from stored input state in manual mode  
- performs all command generation  

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
- callback updates stored manual state only  

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

Manual path:

GUI valueChanged / ADS1115 callback  
→ store latest manual state  
→ control thread wait_pop()  
→ ManualImuCoordinator builds manual setpoint  
→ Kinematics3RRS  
→ cmd_q_.push_latest(...)  
→ actuator thread wait_pop()  
→ ActuatorManager  
→ ServoDriver  

Both paths are processed through the same control and actuation stages. The control thread maintains timing ownership in all modes.

---

## 7. Qt GUI Timing

Qt timers are used only for UI refresh and visualisation.

GUI interactions update stored manual state via valueChanged. The GUI does not generate actuator commands. Continuous command generation remains in the control thread.

---

## 8. IMU Policy in Manual Mode

Manual GUI input does not apply continuous IMU correction by default. Manual commands are generated directly from operator input.

IMU-assisted behaviour can be enabled through configured feedback modes.

---

## 9. Shutdown Behaviour

stop() performs:

- camera shutdown  
- backend shutdown  
- queue termination  
- worker thread join  
- actuator neutral/park handling  
- servo driver shutdown  

Queues notify blocked threads during shutdown, ensuring clean exit.

---

## 10. Summary

The runtime follows a consistent event-driven execution model:

- all worker threads block on kernel-backed primitives (poll, condition_variable)  
- all wakeups originate from external events (file descriptors, queue notifications, GPIO interrupts)  
- no component introduces independent timing loops or polling  

Control execution is centralised in a single thread. All inputs are reduced to state and processed within that thread before actuation.

This structure ensures:

- a single, deterministic control path  
- consistent behaviour across automatic and manual modes  
- separation between realtime processing and UI interaction  
- explicit ownership of timing and data flow  