# System Architecture & Technical Specification

**Project:** Integrasi Computer Vision dan Sistem Sortasi Konveyor untuk Pengendalian Kualitas (Quality Control) Komponen Kapasitor Berdasarkan Polaritas  
**Repository:** `XZ-Manipulator-Controls`  

**Implementation status:** The host VisionSystem, dual-axis `motion_diagnostics` / `stallguard_cli` / `z_bounce_emulation` firmware profiles, and the portable TinyUSB USB-C CDC transport module (`Apps/usb_cdc/`) are implemented and configured for the **BIGTREETECH SKR Octopus Pro V1.1 (STM32H723ZET6)** board (`MotionFirmware.ioc`, `Core/`, `Drivers/STM32H7xx_HAL_Driver/`). The current motion-control profile uses a cooperative foreground state machine with timer-compare interrupts (`TIM2`/`TIM5`) and has no RTOS implementation. A future systems stage may evaluate an RTOS if more complex kinematics and coordinated real-time responsibilities warrant task scheduling; this is an option, not a current commitment. Target builds default to `MOTION_DIAG_BOUNDS_CONFIRMED=ON` with zero safe-Z and pick-depth settings (`SAFE_Z=0`, `PICK_DEPTH=0` dry-run), and the firmware does not yet actuate the vacuum or orientation servo. Physical AS5600 encoder wiring, hardware qualification, and the complete physical sorting cycle remain in progress.

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
   - Packet assembly with CRC16 and transmission via high-speed UART (or planned migration to the portable TinyUSB USB-C CDC module) to the motion controller.
2. **Motion & Actuation Layer (BTT SKR Octopus Pro V1.1 / STM32H723ZET6):**
   - Physical socket mapping (see [`docs/hardware/wiring.md`](../hardware/wiring.md)): X-axis (`motor=1`, `TIM5`) on `MOTOR0`, Z-axis (`motor=0`, `TIM2`) on `MOTOR1`, and provisioned Rotation ($\theta$-axis, `motor=2`, initialized disabled) on `MOTOR2_1`.
   - Dual AS5600 magnetic encoders (`0x36`) connected via independent `I2C1` (`PB8`/`PB9` on `J73`) and `I2C4` (`PB6`/`PB7` on `J43`, powered from `J73` `3.3V`), plus shared TMC2240 `SPI1` (`PA5`/`PA6`/`PA7`, Mode 3, 3.125 MHz) and `USART2` (`PD5`/`PD6` on `J26` at `115200 8N1`). Native USB-C CDC (`PA11`/`PA12`, `USB1_OTG_HS` via `Apps/usb_cdc/`) is verified standalone and ready for profile drop-in.
   - The existing diagnostic profile records conveyor Y, class, angle, and correction but does not actuate the vacuum gripper, orientation servo, or `PC5` `PROBE` proximity input.
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
        Track --> UART_TX[Framed Packet Encoder (CRC16)]
    end

    subgraph MCU ["Motion Control Hardware (SKR Octopus Pro V1.1 / STM32H723ZET6)"]
        UART_RX["USART2 DMA (J26) / Portable USB-C CDC (PA11/PA12)"] --> Parser[CRC16 Validator & Motion Queue]
        Parser --> MotionTask[Foreground Motion Executor]
        MotionTask --> EncTask[Foreground AS5600 Sampling]
        MotionTask --> Telemetry[Diagnostic Frames TX / SEGGER RTT Logs]

        MotionTask -->|SPI1 PA5-7 & TIM2/TIM5 STEP-DIR| TMC[Dual TMC2240 Drivers + Theta Slot]
        MotionTask -. inactive in current profile .-> Servo[End-Effector Reorientation Servo]
        MotionTask -. inactive in current profile .-> Vac[Vacuum Solenoid Ejector]
        EncTask -->|I2C1 X PB8-9 / I2C4 Z PB6-7| AS5600[Dual AS5600 Magnetic Encoders]
    end

    subgraph PLC ["Material Handling Layer (OMRON PLC)"]
        Sensor[Conveyor Photoelectric Sensors] --> Ladder[Conveyor Ladder Logic]
        Ladder --> Motor[Conveyor Belt Motor Drive]
        Ladder -. intended, not hardware-qualified interlock .-> MCU
    end

    UART_TX -->|115200 UART or USB-C CDC| UART_RX
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
- **Target hardware:** STM32H723ZET6 on the **BIGTREETECH SKR Octopus Pro V1.1** (400 MHz CPU / 200 MHz HCLK / 100 MHz APB1 & APB2 from 25 MHz HSE), defined by `MotionFirmware.ioc`, with TMC2240 drivers on `MOTOR0` (X), `MOTOR1` (Z), and provisioned `MOTOR2_1` ($\theta$, initialized disabled). Physical hardware qualification on the gantry remains outstanding.
- **Selectable firmware profiles** (built side-by-side from a shared CubeMX STM32H7 HAL base):
  - **`motion_diagnostics` (default):** Cooperative foreground state machine with circular `USART2` RX DMA (`DMA1_Stream0`) / TX DMA (`DMA1_Stream1`), timer-compare step generation (`TIM2` for Z / `TIM5` for X), and binary-only metrology replies. It has no RTOS implementation; future task scheduling may be evaluated if the kinematics and coordination workload grows.
  - **`stallguard_cli`:** Interactive ASCII console (`USART2` + SEGGER RTT) for two-end StallGuard4 homing, travel calibration, and `soft`/`bounce` axis cycling.
  - **`z_bounce_emulation`:** Single-axis Z bounce responder (`Z` on `MOTOR1`) and hybrid binary/ASCII UART parser used by the emulation harness and protocol validation tests.
  - **`usb_cdc` (portable module + standalone test app):** Reusable TinyUSB `USB1_OTG_HS` CDC module (`PA11`/`PA12`, `HSI48+CRS`, `PA9` VBUS bypass, NVIC prio `5`) ready to drop into any of the profiles above via `add_usb_cdc_module()`.
- In `motion_diagnostics`, the firmware validates 18-byte binary frames, homes Z upward then X, and executes an absolute X move followed by configured Z down/dwell/up phases only when readiness and bounds allow.
- AS5600 samples (`I2C1` / `I2C4`) and driver evidence (`SPI1`) are collected by the foreground executor; timer-compare ISRs (`TIM2` / `TIM5`) generate the step edges.
- Conveyor Y, class, angle, and servo correction are echoed in diagnostic records; the current profile does not perform conveyor timing prediction, suction, bin selection, or servo control.
- Target builds default in `CMakeLists.txt` to `MOTION_DIAG_BOUNDS_CONFIRMED=ON` (`bounds_confirmed=true`, while the host-test fallback in `motion_config.h` is `0`) with `MOTION_DIAG_SAFE_Z_01MM=0` and `MOTION_DIAG_PICK_DEPTH_01MM=0` (dry run). Firmware compilation and host tests do not qualify physical hardware.

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
3. Spatial tracker logs object into queue ➔ Framed packet sent to STM32 with (X, Y, angle, class).
4. STM32 Motion Task plans X-axis alignment and moves carriage above target conveyor coordinate.
5. Z-axis descends to pickup height ➔ Vacuum solenoid energized (suction cup grips capacitor).
6. Z-axis retracts ➔ Servo rotates gripper by calculated angular correction to reorient polarity.
7. X-axis traverses to destination bin:
   - "Correct Polarity / Passed QC" ➔ Placed onto sorting tray in standardized orientation.
   - "Incorrect / Defective" ➔ Discharged into reject bin.
8. Vacuum released ➔ Manipulator returns to Standby / Home position.
```

---

## 4. Development Objectives, Diagnostics Roadmap & Visualization Tooling

### A. Staged Hardware Bring-Up Objectives

| Stage | Scope & Hardware State | Objectives & Verification Milestones |
| :--- | :--- | :--- |
| **Stage 1 — Core Bring-Up** *(Current)* | Steppers + TMC2240 drivers ready; J-Link OB SWD (`J72`) + USB-C CDC (`PA11`/`PA12`) verified | Validate TMC2240 `SPI1` comms, motor current/polarity, StallGuard4 sensorless homing (`stallguard_cli`), and reliable USB-C Virtual COM (`XZ-MotionControls` via `Apps/usb_cdc/`). |
| **Stage 2 — AS5600 Closed-Loop Diagnostics** | Dual AS5600 encoders installed on X (`I2C1`) and Z (`I2C4`) | Verify magnet health (`MD`/`ML`/`MH`, `AGC` margin), confirm `encoder_sign` and `enc_ratio`, eliminate continuity gaps (`>10 ms`), and capture 3-way move evidence (`motion_diagnostics` + `tools/diag`) to measure bidirectional repeatability (`AP`/`RP`). |
| **Stage 3 — USB-C CDC Migration & High-Rate Telemetry** | Drop [`Apps/usb_cdc/`](../../MotorControls/MotionFirmware/Apps/usb_cdc/README.md) into active motion profiles (`add_usb_cdc_module()`) | Replace physical `USART2` dependency (`115200` baud) with `12 Mbps` Full-Speed USB-C CDC so `J68`-removed 24 V runs keep full bidirectional command/telemetry, plus optional `200–500 Hz` live waveform streaming. |
| **Stage 4 — Full Vision-to-Motion Sorting** | Vacuum solenoid, $\theta$ axis, calibrated camera homography & OMRON PLC interlock | Implement closed-loop residual step trim/recovery, qualify physical `SAFE_Z`/`PICK_DEPTH` and end-effector timing, and execute full end-to-end capacitor pick-rotate-place trials. |

### B. Diagnostic Data Available Once AS5600 Encoders Are Wired

With dual AS5600 encoders polling every `2 ms` alongside TMC2240 SPI telemetry, the firmware produces three cross-checked diagnostic layers (see [`motion_diagnostics_design.md`](motion_diagnostics_design.md) and [`diag_record_fields.h`](../../MotorControls/MotionFirmware/Apps/motion_diagnostics/Inc/diag_record_fields.h)):

1. **Encoder & Magnet Health:** 12-bit raw angle (`0..4095`, `0.0879°/count`), signed multi-turn `unwrap_counts` (continuous up to `250 rev/s`), `AS5600 STATUS` (`MD`/`ML`/`MH`), `AGC` airgap gain (`0..255`), volatile `CONF` (`0x0300`) check, and `max_sample_gap_us` (`I2C` stall detection).
2. **Three-Way Motion Evidence per Phase (`DIAG_COMMAND_RESULT` 356 B / `DIAG_HOME_RESULT` 124 B):**
   - **Step Engine (`TIM2`/`TIM5`):** `commanded_delta_steps`, `emitted_delta_steps`, `quant_residual_nm`, `move_duration_us`.
   - **TMC2240 Electrical (`SPI1`):** `mscnt_start/end/expected/observed`, `driver_mode`, `sg_baseline`, `sg_trigger`, `DRV_STATUS`, `GSTAT`.
   - **AS5600 Mechanical:** `unwrap_start/end/delta`, `displacement_um`, `home_offset_counts`, plus live `follow_err` / `follow_err_peak` and `AX_FAULT_STEP_LOSS` detection in `stallguard_cli`.
3. **Automated Host Metrology (`packages/motion_diagnostics` & `tools/diag`):** Computes bias, MAE, max |error|, RMSE, sample SD, and ISO-style 1D accuracy/repeatability analogues (`AP_1D_adaptation`, `RP_1D_adaptation`) grouped by axis, phase, target, and approach direction (backlash/hysteresis), excluding clamped or faulted trials.

### C. Host Visualization & Observability Tooling Goals

To inspect this telemetry effectively across bring-up, tuning, and full system integration, host visualization uses three complementary tools over USB-C CDC (`Apps/usb_cdc/`) and `tools/diag` outputs:

| Tool | Target Development Role | Data Source / Integration Path | Primary Diagnostic Use Cases |
| :--- | :--- | :--- | :--- |
| **Serial Studio** | Live bench bring-up dashboard *(zero PC code)* | Connects directly to the STM32 USB-C CDC COM port (`XZ-MotionControls`) via lightweight CSV/JSON telemetry frames | Real-time verification of AS5600 magnet airgap (`AGC`/`STATUS`), live `SG4_RESULT` bars during manual jogs, and `cmd vs. enc` tracking during Stage 2 bring-up. |
| **PlotJuggler** | High-rate servo, StallGuard4 & vibration analysis (`200–1000 Hz`) | Imports CSV logs exported by `motion-diag`, or streams live UDP/JSON packets via a small Python serial-to-UDP bridge (`127.0.0.1:9870`) | Overlaying `follow_err`, velocity/accel ramps, `MSCNT`, and `SG4_RESULT` on synchronized time axes; running FFTs on `follow_err`/`AGC` to expose belt tooth pitch resonance or lead-screw eccentricity. |
| **Rerun (`rerun.io`)** | Full-system `VisionSystem` + `MotionFirmware` synchronized replay | Python `rerun-sdk` logging both vision and STM32 USB-C CDC streams into a single `.rrd` timeline | Scrubbing camera frames, YOLO/PCA orientation overlays, a 2D X-Z gantry schematic (`commanded vs. AS5600`), `follow_err`, `SG4`, and fault markers side-by-side to diagnose missed picks and produce thesis visual evidence. |

---

## 5. Related Specifications

- **Host ➔ STM32 Command Wire Format:** [`communication_protocol.md`](communication_protocol.md)
- **STM32 ➔ Host Diagnostic Records & Metrology Design:** [`motion_diagnostics_design.md`](motion_diagnostics_design.md)
- **Portable TinyUSB USB-C CDC Module & Integration Guide:** [`Apps/usb_cdc/README.md`](../../MotorControls/MotionFirmware/Apps/usb_cdc/README.md)
- **SKR Octopus Pro V1.1 Pinout & Harness Wiring:** [`../hardware/wiring.md`](../hardware/wiring.md)
- **Mechanical & Electrical Bill of Materials:** [`../hardware/bom_and_mechanics.md`](../hardware/bom_and_mechanics.md)
- **Verification & Test Plan Index:** [`../testing/README.md`](../testing/README.md)
