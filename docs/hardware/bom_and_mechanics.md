# Bill of Materials & Mechanical Specification

**Project:** 2-Axis (XZ) Cartesian Manipulator & Vision-Based Sorting Conveyor  
**Undergraduate Thesis:** Integrasi Computer Vision dan Sistem Sortasi Konveyor untuk Pengendalian Kualitas Komponen Kapasitor Berdasarkan Polaritas  

---

## 1. System Overview & Gantry Kinematics

The physical sorting manipulator is a 2-Axis Cartesian Robot operating in the vertical-longitudinal plane ($X$ and $Z$ axes):
- **$X$-Axis (Horizontal / Conveyor Transverse):** Moves the end-effector across the conveyor belt width to align with detected capacitors or sorting bins.
- **$Z$-Axis (Vertical / Pick-and-Place):** Lowers and raises the pneumatic suction gripper toward the conveyor bed.
- **End-Effector:** Integrated vacuum suction cup with a miniature servo motor for continuous angular polarity correction ($[0^\circ, 360^\circ)$).

```text
                  +===============================+
                  |       X-Axis Linear Rail      |
                  +===============================+
                                  |
                                  v  [X-Carriage]
                         +-----------------+
                         |  Z-Axis Column  |
                         |   (Lead Screw)  |
                         +-----------------+
                                  |
                                  v  [Z-Stage]
                         +-----------------+
                         | Reorientation   |
                         | Servo Motor     |
                         +-----------------+
                                  |
                                  v
                        ( Vacuum Suction )
                               (Cup)
                                 |
                                 v
               === [ Moving Conveyor Belt ] ===
```

---

## 2. Mechanical Hardware Specifications

| Component | Specification | Description |
| :--- | :--- | :--- |
| **X-Axis Drive** | GT2 Timing Belt (2 mm pitch, 6 mm width) / Linear Guide Rail | High-speed horizontal positioning |
| **X-Axis Resolution** | 20-tooth GT2 Pulley (40 mm/rev) | $\approx 80\ \text{steps/mm}$ @ 1/16 microstepping |
| **Z-Axis Drive** | T8 Lead Screw (8 mm diameter, 2 mm pitch, 4-start, 8 mm lead) | High holding force against gravitational sag |
| **Z-Axis Resolution** | 8 mm advance per revolution | $\approx 400\ \text{steps/mm}$ @ 1/16 microstepping |
| **Stroke Length** | $X$: 300 mm, $Z$: 150 mm | Tailored to industrial conveyor width |
| **End-Effector Gripper**| Pneumatic Suction Cup (SMC / Festo industrial silicone cup) | Non-destructive capacitor pick-up |
| **End-Effector Rotation**| MG90S / SG90 Metal-Gear Micro Servo | Continuous capacitor polarity angle adjustment |

---

## 3. Electrical & Actuator Bill of Materials (BOM)

| Subsystem | Part Name | Manufacturer / Model | Quantity | Operating Specs |
| :--- | :--- | :--- | :---: | :--- |
| **MCU Board** | STM32 Nucleo-64 | STMicroelectronics `NUCLEO-F446RE` | 1 | Cortex-M4F @ 180 MHz, 512 KB Flash, 128 KB SRAM |
| **Stepper Drivers** | TMC2240 Stepper Driver | MKS TMC2240 v1.0 (SPI Mode) | 2 | Trinamic TMC2240, SPI interface, StealthChop2, StallGuard4 |
| **Position Feedback** | Magnetic Rotary Encoders | ams OSRAM `AS5600` | 2 | 12-bit contactless rotary encoder (4096 counts/rev), I2C |
| **Motors** | Bipolar Stepper Motors | NEMA 17 (1.5 A - 1.8 A/phase) | 2 | $1.8^\circ$ step angle (200 full steps/rev) |
| **Pneumatics** | Vacuum Generator / Ejector | Venturi Vacuum Ejector + Solenoid Valve (12V/24V) | 1 | Fast-response vacuum pull and blow-off release |
| **Power Supply** | Industrial SMPS | 24V DC, 10A (240W) | 1 | Powers stepper drivers, PLC, and solenoid valves |
| **Logic Supply** | Step-down Buck Converter | 24V ➔ 5V (3A) | 1 | Powers servos, camera lighting, and auxiliary sensors |
| **Vision Camera** | Overhead Inspection Camera | USB 2.0 / USB 3.0 Industrial Webcam | 1 | 1080p @ 30 FPS, fixed focal length, overhead perpendicular mount |
| **Conveyor PLC** | Programmable Logic Controller| OMRON CP2E / CP1E | 1 | Relay/Transistor outputs for AC/DC conveyor motor & interlocks |

---

## 4. Pneumatic & Solenoid Gripper Control

The vacuum suction subsystem is driven by the STM32 via an optocoupled MOSFET / relay module:

```text
    STM32 GPIO (3.3V) ➔ Optocoupler (PC817) ➔ Power MOSFET (IRFZ44N / Relay) ➔ 24V Solenoid Valve ➔ Vacuum Generator
```

- **Pick Action:** Solenoid valve energized $\to$ Venturi generator creates vacuum $\to$ Suction cup secures capacitor.
- **Drop/Sort Action:** Solenoid valve de-energized $\to$ Atmospheric bleed valve opens $\to$ Capacitor drops into sorting bin.
