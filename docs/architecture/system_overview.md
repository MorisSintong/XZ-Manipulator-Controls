# System Architecture & Technical Specification

**Project:** Integrasi Computer Vision dan Sistem Sortasi Konveyor untuk Pengendalian Kualitas (Quality Control) Komponen Kapasitor Berdasarkan Polaritas  
**Repository:** `XZ-Manipulator-Controls`  

**Implementation status:** The host VisionSystem and dual-axis `motion_diagnostics` firmware profile are implemented. The physical controller target is now the BTT SKR Octopus Pro V1.1 (STM32H723), but the checked-in motion firmware and build still target the STM32F446RE Nucleo-64 and have not been ported. The current motion-control profile uses a cooperative foreground state machine with timer-compare interrupts and has no RTOS implementation. A future systems stage may evaluate an RTOS if more complex kinematics and coordinated real-time responsibilities warrant task scheduling; this is an option, not a current commitment. The default safe-Z and pick-depth settings are zero, and the firmware does not actuate the vacuum or orientation servo. Hardware qualification and the complete physical sorting cycle remain outstanding.

---

## 1. High-Level System Architecture

The automated quality control and sorting cell integrates three synchronized layers:
1. **Perception & Decision Layer (Host PC - Python / YOLO):**
   - Overhead camera inspection; supported resolution and frame rate depend on the selected camera and host.
   - Real-time detection of electrolytic capacitors moving along the conveyor belt.
   - Contour analysis / Principal Component Analysis (PCA) for continuous orientation angle calculation ($[0^\circ, 360^\circ)$).
   - Polarity determination (anode/cathode lead orientation vs can body stripe).
   - Timestamped object tracking with FIFO debounce to prevent duplicate triggers.
   - Default pixel-to-millimeter mapping uses configured per-axis scales and origin offsets. The mapper supports an optional homography, but the live launcher does not load a calibrated homography file.
   - The $\pm 2.5^\circ$ orientation result is covered by synthetic tests; physical accuracy remains unqualified.
   - Packet assembly with CRC16 and transmission via high-speed UART to the motion controller.
2. **Motion & Actuation Layer (planned BTT SKR Octopus Pro V1.1 / STM32H723; firmware port pending):**
   - Planned physical mapping: X and Z TMC2240s in SKR driver sockets 0 and 2, with board-specific connections in `docs/hardware/wiring.md`.
   - The existing Nucleo firmware uses dual AS5600 buses (`I2C1`, `I2C3`) and TMC2240 `SPI2`; those are F446 assignments, not the H723 board mapping.
   - The H723 firmware port, including encoder buses, driver SPI, UART, timers, and homing, is pending.
   - The existing diagnostic profile records conveyor Y, class, angle, and correction but does not actuate the vacuum gripper or orientation servo.
3. **Material Handling & Interlocking Layer (OMRON PLC):**
   - Conveyor belt drive control and variable frequency / speed regulation.
   - Proximity optical sensors for part arrival detection.
   - Safety interlocking, emergency stop (E-STOP), and reject gate pneumatic actuation.

```mermaid
flowchart TD
    subgraph HostPC ["Perception Layer (Host PC / Python)"]
        Cam[Overhead USB Camera] --> Cap[Camera Frame Acquisition]
        Cap --> YOLO[Ultralytics YOLO Model]
        YOLO --> Angle[PCA Continuous Angle & Polarity]
        Angle --> Track[Spatial Conveyor Tracker & FIFO]
        Track --> UART_TX[Framed UART Packet Encoder (CRC16)]
    end

    subgraph MCU ["Motion Control Hardware (SKR Octopus Pro V1.1 / STM32H723; firmware port pending)"]
        UART_RX[UART / USB link - H723 mapping pending] --> Parser[CRC16 Validator & Motion Queue]
        Parser --> MotionTask[Foreground Motion Executor]
        MotionTask --> EncTask[Foreground AS5600 Sampling]
        MotionTask --> Telemetry[Board serial link / diagnostic logs - H723 mapping pending]

        MotionTask -->|SPI1 & board driver sockets| TMC[Dual TMC2240 Stepper Drivers]
        MotionTask -. inactive in current profile .-> Servo[End-Effector Reorientation Servo]
        MotionTask -. inactive in current profile .-> Vac[Vacuum Solenoid Ejector]
        EncTask -->|I2C buses per wiring.md| AS5600[Dual AS5600 Magnetic Encoders]
    end

    subgraph PLC ["Material Handling Layer (OMRON PLC)"]
        Sensor[Conveyor Photoelectric Sensors] --> Ladder[Conveyor Ladder Logic]
        Ladder --> Motor[Conveyor Belt Motor Drive]
        Ladder -. intended, not hardware-qualified interlock .-> MCU
    end

    UART_TX -->|115200 baud; H723 physical link pending| UART_RX
    TMC --> XZ_Gantry[2-Axis XZ Cartesian Manipulator]
```

---

## 2. Subsystem Functional Breakdown

### A. Vision System (`VisionSystem/`)
- **Inputs:** Live frames from a perpendicular overhead camera; resolution and frame rate depend on the selected camera and host.
- **Processing:**
  - Bounding box inference via trained YOLO weights.
  - Default pixel-to-millimeter mapping uses configured per-axis scales and origin offsets. The mapper supports an optional homography, but the live launcher does not load a calibrated homography file.
  - Principal Component Analysis (PCA) estimates orientation $\theta \in [0.0^\circ, 360.0^\circ)$. The $\pm 2.5^\circ$ result is covered by synthetic tests; physical accuracy remains unqualified.
  - Centroid tracking with boundary debounce to queue each capacitor exactly once as it crosses the designated inspection threshold line.
- **Outputs:** Framed binary telemetry packet dispatched to STM32.

### B. Motion Controller Firmware (`MotorControls/MotionFirmware/`)
- **Existing firmware target:** STM32F446RE (`NUCLEO-F446RE`). Its core clock and peripheral bindings are defined by `MotionFirmware.ioc`; this firmware is not compatible with the H723 board without a port.
- **Physical board target:** BTT SKR Octopus Pro V1.1 (STM32H723), with TMC2240 modules in the X, Z, and optional theta driver sockets. H723 peripheral, UART, timer, encoder, and driver settings remain to be implemented and hardware-qualified.
- **Selectable firmware profiles** (built side-by-side from a shared CubeMX HAL base):
  - **`motion_diagnostics` (default):** Cooperative foreground state machine with circular `USART2` RX DMA, timer-compare step generation (`TIM2`/`TIM5`), and binary-only metrology replies. It has no RTOS implementation; future task scheduling may be evaluated if the kinematics and coordination workload grows.
  - **`stallguard_cli`:** Interactive ASCII console (`USART2` + SEGGER RTT) for two-end StallGuard4 homing, travel calibration, and `soft`/`bounce` axis cycling.
  - **`z_bounce_emulation`:** Single-axis Z bounce responder used by the Renode/Mock UART emulation harness.
- In `motion_diagnostics`, the firmware validates 18-byte binary frames, homes Z upward then X, and executes an absolute X move followed by configured Z down/dwell/up phases only when readiness and bounds allow.
- AS5600 samples and driver evidence are collected by the foreground executor; timer-compare ISRs generate the step edges.
- Conveyor Y, class, angle, and servo correction are echoed in diagnostic records; the current profile does not perform conveyor timing prediction, suction, bin selection, or servo control.
- The default configuration is a dry run with zero pick depth and `bounds_confirmed=false`. Firmware compilation and host tests do not qualify physical hardware.

### C. Industrial Conveyor Controller (`Program_Ladder/`)
- **Target:** OMRON PLC (CP1E / CP2E family) programmed via CX-Programmer.
- **Functions:**
  - Conveyor continuous motion with adjustable indexing.
  - Optical proximity detection for entry, inspection, and bin positions.
  - Hardware E-Stop fail-safe trip circuits.

---

## 3. Intended Physical Sorting Lifecycle

The sequence below describes the target workcell workflow; it is not fully implemented by the current dry-run diagnostic firmware profile.

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

---

## 4. Related Specifications

- **Host ➔ STM32 Command Wire Format:** [`communication_protocol.md`](communication_protocol.md)
- **STM32 ➔ Host Diagnostic Records & Metrology Design:** [`motion_diagnostics_design.md`](motion_diagnostics_design.md)
- **SKR Octopus Pro V1.1 Pinout & Harness Wiring:** [`../hardware/wiring.md`](../hardware/wiring.md)
- **Mechanical & Electrical Bill of Materials:** [`../hardware/bom_and_mechanics.md`](../hardware/bom_and_mechanics.md)
- **Verification & Test Plan Index:** [`../testing/README.md`](../testing/README.md)
