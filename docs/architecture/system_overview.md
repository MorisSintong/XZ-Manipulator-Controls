# System Architecture & Technical Specification

**Project:** Integrasi Computer Vision dan Sistem Sortasi Konveyor untuk Pengendalian Kualitas (Quality Control) Komponen Kapasitor Berdasarkan Polaritas  
**Repository:** `XZ-Manipulator-Controls`  

---

## 1. High-Level System Architecture

The automated quality control and sorting cell integrates three synchronized layers:
1. **Perception & Decision Layer (Host PC - Python / YOLO):**
   - High-resolution overhead computer vision inspection.
   - Real-time detection of electrolytic capacitors moving along the conveyor belt.
   - Contour analysis / Principal Component Analysis (PCA) for continuous orientation angle calculation ($[0^\circ, 360^\circ)$).
   - Polarity determination (anode/cathode lead orientation vs can body stripe).
   - Timestamped object tracking with FIFO debounce to prevent duplicate triggers.
   - Packet assembly with CRC16 and transmission via high-speed UART to the motion controller.
2. **Motion & Actuation Layer (STM32F446RE - FreeRTOS):**
   - 2-Axis ($XZ$) Cartesian gantry motion control.
   - Closed-loop position verification using dual AS5600 12-bit magnetic encoders over dual hardware I2C buses (`I2C1`, `I2C3`).
   - Trinamic TMC2240 stepper drivers configured and monitored via high-speed SPI (`SPI2`).
   - Sensorless homing and mechanical stall detection using StallGuard4.
   - Pneumatic vacuum gripper actuation and end-effector micro-servo angular correction.
3. **Material Handling & Interlocking Layer (OMRON PLC):**
   - Conveyor belt drive control and variable frequency / speed regulation.
   - Proximity optical sensors for part arrival detection.
   - Safety interlocking, emergency stop (E-STOP), and reject gate pneumatic actuation.

```mermaid
flowchart TD
    subgraph HostPC ["Perception Layer (Host PC / Python)"]
        Cam[Overhead USB Camera] --> Cap[Frame Acquisition 30 FPS]
        Cap --> YOLO[Ultralytics YOLO Model]
        YOLO --> Angle[PCA Continuous Angle & Polarity]
        Angle --> Track[Spatial Conveyor Tracker & FIFO]
        Track --> UART_TX[Framed UART Packet Encoder (CRC16)]
    end

    subgraph MCU ["Motion Control Layer (STM32F446RE / FreeRTOS)"]
        UART_RX[DMA Circular UART RX] --> Parser[CRC16 Validator & Motion Queue]
        Parser --> FreeRTOS[FreeRTOS Task Scheduler]
        FreeRTOS --> MotionTask[Trajectory & Kinematics Task]
        FreeRTOS --> EncTask[AS5600 Closed-Loop Feedback Task]
        FreeRTOS --> Telemetry[USART2 / SEGGER RTT Telemetry]

        MotionTask -->|SPI2 & Step/Dir| TMC[Dual TMC2240 Stepper Drivers]
        MotionTask -->|PWM| Servo[End-Effector Reorientation Servo]
        MotionTask -->|GPIO / Relay| Vac[Vacuum Solenoid Ejector]
        EncTask -->|I2C1 & I2C3| AS5600[Dual AS5600 Magnetic Encoders]
    end

    subgraph PLC ["Material Handling Layer (OMRON PLC)"]
        Sensor[Conveyor Photoelectric Sensors] --> Ladder[Conveyor Ladder Logic]
        Ladder --> Motor[Conveyor Belt Motor Drive]
        Ladder <-->|Modbus TCP / Digital Handshake| MCU
    end

    UART_TX -->|USB-UART / 115200 baud| UART_RX
    TMC --> XZ_Gantry[2-Axis XZ Cartesian Manipulator]
```

---

## 2. Subsystem Functional Breakdown

### A. Vision System (`VisionSystem/`)
- **Inputs:** Live 1080p video feed from perpendicular overhead camera.
- **Processing:**
  - Bounding box inference via trained YOLO weights.
  - Homography transformation matrix converting pixel coordinates $(u, v)$ to conveyor frame coordinates $(X, Y)$ in millimeters.
  - Principal Component Analysis (PCA) on extracted binary contour to determine physical orientation $\theta \in [0.0^\circ, 360.0^\circ)$ with $\le \pm 2.5^\circ$ error.
  - Centroid tracking with boundary debounce to queue each capacitor exactly once as it crosses the designated inspection threshold line.
- **Outputs:** Framed binary telemetry packet dispatched to STM32.

### B. Motion Controller Firmware (`MotorControls/MotionFirmware/`)
- **Target:** STM32F446RE (ARM Cortex-M4F @ 180 MHz).
- **Architecture:** FreeRTOS pre-emptive multitasking:
  - `MotionTask`: Calculates trapezoidal velocity profiles, issues Step/Dir pulses to TMC2240, and coordinates pick-and-place trajectories.
  - `EncoderTask`: Periodically reads absolute 12-bit angle registers from $X$ and $Z$ AS5600 encoders over I2C1 and I2C3.
  - `CommTask`: Receives UART commands via DMA ring buffer, verifies CRC16 checksums, and pushes targets to the motion queue.
  - `SafetyTask`: Monitors StallGuard4 flags, driver thermal warnings, and software endstops.

### C. Industrial Conveyor Controller (`Program_Ladder/`)
- **Target:** OMRON PLC (CP1E / CP2E family) programmed via CX-Programmer.
- **Functions:**
  - Conveyor continuous motion with adjustable indexing.
  - Optical proximity detection for entry, inspection, and bin positions.
  - Hardware E-Stop fail-safe trip circuits.

---

## 3. Physical Sorting Lifecycle

```text
1. Capacitor placed on conveyor ➔ Travels toward inspection zone.
2. Photo-sensor triggers acquisition ➔ Vision model infers bounding box, polarity, and angle.
3. Spatial tracker logs object into queue ➔ UART packet sent to STM32 with (X, Y, angle, class).
4. STM32 Motion Task plans X-axis alignment and moves carriage above target conveyor coordinate.
5. Z-axis descends to pickup height ➔ Vacuum solenoid energized (suction cup grips capacitor).
6. Z-axis retracts ➔ Servo rotates gripper by calculated angular correction to reorient polarity.
7. X-axis traverses to destination bin:
   - "Correct Polarity / Passed QC" ➔ Placed onto sorting tray in standardized orientation.
   - "Incorrect / Defective" ➔ Discharged into reject bin.
8. Vacuum released ➔ Manipulator returns to Standby / Home position.
```
