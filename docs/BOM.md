# Bill of Materials (BOM) — Solar Stewart Tracker

## 1. Hardware Components

| Item No. | Component | Specification / Model | Qty | Supplier | Unit Cost (£) | Total (£) | Notes |
|----------|-----------|-----------------------|-----|----------|---------------|-----------|-------|
| 1 | Raspberry Pi 5 | 8GB Model | 1 | Lab / Personal | 80.00 | 80.00 | Main controller |
| 2 | IMX219 Camera | 8MP CSI Camera Module | 1 | Amazon | 25.00 | 25.00 | Vision input |
| 3 | PCA9685 PWM Driver | 16-channel I2C PWM | 1 | Amazon | 12.00 | 12.00 | Servo control |
| 4 | High-Torque Servo | RDS3230 or equivalent | 3 | Amazon | 15.00 | 45.00 | 3RRS actuators |
| 5 | External Power Supply | 5–6V High Current | 1 | Amazon | 20.00 | 20.00 | Servo supply |
| 6 | Breadboard & Wiring | Jumper wires, headers | 1 set | Lab | 10.00 | 10.00 | Prototyping |
| 7 | Structural Frame | Acrylic / 3D Printed | 1 | Self-fabricated | 15.00 | 15.00 | Platform support |
| 8 | Fasteners & Mounts | Screws, brackets | Assorted | Hardware Store | 10.00 | 10.00 | Mechanical assembly |

---

## 2. Software Dependencies

| Software | Purpose | Cost |
|----------|----------|------|
| C++17 | Core implementation | Free |
| CMake | Build system | Free |
| OpenCV | Vision processing | Free |
| libcamera | Camera backend (Pi) | Free |
| GitHub | Version control | Free |

---

## 3. Cost Summary

| Category | Cost (£) |
|----------|----------|
| Electronics | 182.00 |
| Mechanical | 25.00 |
| Miscellaneous | 10.00 |
| **Total Hardware Cost** | **£217.00** |

---

## 4. Budget Assessment

- Total estimated cost: **£217.00**
- Within expected student project budget: **Yes**
- Major cost drivers:
  - Raspberry Pi
  - High-torque servos
  - Camera module

---

## 5. Design Considerations

- External servo power supply used to prevent brownout.
- PWM driver isolates timing from Raspberry Pi CPU.
- Capacity=1 queue architecture prevents actuator backlog.
- Modular design allows hardware substitution without software redesign.

---

## 6. Traceability to Architecture

| Subsystem | Hardware Used |
|-----------|--------------|
| Vision (T1/T2) | IMX219 Camera |
| Control (T2) | Raspberry Pi 5 |
| Actuation (T3) | PCA9685 + Servos |
| Safety Layer | ActuatorManager (software) |
| Logging | Console / File logger |

---

## 7. Reproducibility Notes

All components are commercially available.
Part numbers and specifications allow replication of the system.
Mechanical components can be 3D printed using standard PLA.
