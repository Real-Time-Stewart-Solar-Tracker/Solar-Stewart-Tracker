# EXACT CONNECTIONS 

---

##  Raspberry Pi Pins (PHYSICAL)

| Physical Pin | Signal | Connection |
|-------------|--------|-----------|
| Pin 1 | 3.3V | PCA9685 VCC, ADS1115 VDD, MPU6050 VCC |
| Pin 3 | GPIO2 (SDA1) | PCA9685 SDA, ADS1115 SDA, MPU6050 SDA |
| Pin 5 | GPIO3 (SCL1) | PCA9685 SCL, ADS1115 SCL, MPU6050 SCL |
| Pin 6 | GND | ALL module grounds + external supply ground |
| Pin 11 | GPIO17 | ADS1115 ALERT/RDY |
| Pin 13 | GPIO27 | MPU6050 INT |

---

##  PCA9685 Connections

| Pin | Connects To |
|-----|------------|
| VCC | Raspberry Pi Pin 1 (3.3V) |
| GND | Raspberry Pi Pin 6 (GND) |
| SDA | Raspberry Pi Pin 3 (GPIO2 / SDA1) |
| SCL | Raspberry Pi Pin 5 (GPIO3 / SCL1) |
| V+  | External 5–6V power supply (+) |

### Servo Outputs

| Channel | Connects To |
|--------|-------------|
| CH0 | Servo 0 signal |
| CH1 | Servo 1 signal |
| CH2 | Servo 2 signal |

---

## ADS1115 Connections

| Pin | Connects To |
|-----|------------|
| VDD | Raspberry Pi Pin 1 (3.3V) |
| GND | Raspberry Pi Pin 6 (GND) |
| SDA | Raspberry Pi Pin 3 (GPIO2 / SDA1) |
| SCL | Raspberry Pi Pin 5 (GPIO3 / SCL1) |
| ALERT/RDY | Raspberry Pi Pin 11 (GPIO17) |

###  Analog Inputs

| Channel | Source |
|--------|--------|
| A0 | Tilt potentiometer (wiper) |
| A1 | Pan potentiometer (wiper) |

---

##  MPU6050 Connections

| Pin | Connects To |
|-----|------------|
| VCC | Raspberry Pi Pin 1 (3.3V) |
| GND | Raspberry Pi Pin 6 (GND) |
| SDA | Raspberry Pi Pin 3 (GPIO2 / SDA1) |
| SCL | Raspberry Pi Pin 5 (GPIO3 / SCL1) |
| INT | Raspberry Pi Pin 13 (GPIO27) |

---

##  Servo Wiring

Each servo has 3 wires:

| Servo Wire | Connects To |
|-----------|------------|
| Red (Power) | External 5–6V power supply (+) |
| Brown / Black (GND) | External supply GND (shared with Pi GND) |
| Yellow / Orange (Signal) | PCA9685 channel (CH0 / CH1 / CH2) |

---

##  Potentiometers (2 Units)

Each potentiometer has 3 pins:

| Potentiometer Pin | Connects To |
|------------------|------------|
| One end | Raspberry Pi Pin 1 (3.3V) |
| Other end | Raspberry Pi Pin 6 (GND) |
| Middle (Wiper) | ADS1115 A0 (tilt) or A1 (pan) |

---

##  External Power Supply

| Pin | Connects To |
|-----|------------|
| +V (5–6V) | PCA9685 V+ and all servo power pins |
| GND | Common ground (connected to Raspberry Pi GND) |

---

##  CSI Camera

| Connection | Description |
|-----------|------------|
| CSI Ribbon Cable | Connects directly to Raspberry Pi CSI port |
| GPIO Usage | None |

---

#  CRITICAL ELECTRICAL RULES

- All grounds MUST be connected together (common ground)
- I2C bus is shared:
  - SDA → PCA9685, ADS1115, MPU6050
  - SCL → PCA9685, ADS1115, MPU6050
- DO NOT swap SDA and SCL
- DO NOT power servos from Raspberry Pi
- DO NOT connect 5–6V to any Raspberry Pi GPIO pin
- PCA9685 has TWO power domains:
  - VCC → 3.3V (logic)
  - V+ → 5–6V (servos)

---

#  SYSTEM SUMMARY

- Raspberry Pi controls system logic
- I2C bus connects sensors and driver:
  - PCA9685 (actuation)
  - ADS1115 (manual input)
  - MPU6050 (feedback)
- External supply powers servos only
- Interrupt-driven design:
  - ADS1115 → GPIO17
  - MPU6050 → GPIO27

---