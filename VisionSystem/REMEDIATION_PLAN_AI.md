# CRITICAL MANDATE: VisionSystem Remediation & Alignment Plan

> **ATTENTION TO ANY AI ASSISTANT / AGENT WORKING ON THIS REPOSITORY:**  
> You are tasked with remediating the `VisionSystem/` codebase to conform with the official undergraduate thesis engineering specifications in `01_Dokumen_TA/TA_2026_extracted.txt`.  
> You must execute all items below methodically. **DO NOT mark this task as complete or finish your turn until ALL requirements and verification checks pass programmatically.**

---

## 1. Context & Architectural Requirements

- **System Purpose:**  
  Automated Quality Control (QC) and sorting of electrolytic capacitors on an industrial conveyor using a 2-axis (XZ) Cartesian Manipulator with vacuum suction cup and micro-servo reorientation.
- **Hardware Integration Target:**
  - **Vision:** Overhead camera $\to$ PC running Ultralytics YOLO + Continuous Angle Estimation $\to$ Camera-to-Conveyor Homography (pixel to mm).
  - **Microcontroller:** **STM32F446RE** running FreeRTOS with DMA UART (circular buffer).
  - **Motion Controller:** STM32 controls **TMC2240** stepper drivers (SPI / Step-Dir) for X and Z axes, with **AS5600** magnetic encoders (I2C) and vacuum solenoid valve.
  - **Sorting Jig & Conveyor Line:** Coordinated with **OMRON CP2E-N PLC** via Modbus TCP/IP.

---

## 2. Documented Critical Faults (Must Be Remediated)

1. **Quantized Angle Trap in `utils/angle_calculator.py`:**
   - The current code clamps orientation to only 4 discrete angles ($0^\circ, 90^\circ, 180^\circ, 270^\circ$). Continuous angles (e.g. $15^\circ, 30^\circ, 45^\circ, 60^\circ, 135^\circ$) are forcefully corrupted.
   - Polarity detection uses fragile edge-density heuristics that mistake casing text/stripe for lead pins and falls back to hardcoded labels.
2. **Missing Spatial Coordinates in UART Protocol (`4_detect_realtime.py`):**
   - The current UART packet sends only `K<id>,S<angle>,R<correction>`.
   - It omits $(X, Y)$ coordinates, tracking ID, timestamp, start/stop delimiters, and CRC16 checksum. The STM32 Cartesian robot cannot plan motion without physical $(X, Y)$ millimeter positions.
3. **Serial Message Flooding & Lack of Conveyor Tracking:**
   - Every detected box sends UART messages on every camera frame (up to 30 Hz per object).
   - No tracking queue (FIFO) exists to handle moving capacitors along the conveyor.
4. **Hardware Flash Wear Hazard in `firmware_esp32/firmware_esp32.ino`:**
   - Firmware writes to ESP32 Flash Memory (NVS) on every incoming UART packet (`preferences.putString`). This will burn out and permanently destroy the microcontroller's Flash memory within hours.
   - Blocking `delay(300)` causes serial buffer overflow and packet loss.
5. **Portability & Environment Failures:**
   - All `.bat` files hardcode `C:\Users\HUSEN\.conda\envs\depth-obstacle-detector\python.exe`.
   - `1_train.py` references nonexistent `--model yolo26m.pt`.

---

## 3. Strict Step-by-Step Execution Roadmap

### Step 1: Continuous Angle & Polarity Detection (`utils/angle_calculator.py`)
- Implement continuous angle calculation using Principal Component Analysis (PCA) on the capacitor contour or OpenCV Minimum Area Bounded Box with proper version-agnostic angle unwrapping.
- Provide continuous output in range $[0.0^\circ, 360.0^\circ)$ with $\le \pm 2.5^\circ$ error.
- Detect polarity (lead pins vs can base) using brightness/aspect centroid shift or localized polarized template analysis rather than raw boundary strip edge counts.
- Never use class name strings (`if "benar" in class_name`) as a shortcut/fallback for calculating physical angle.

### Step 2: Industrial UART Telemetry Protocol (`utils/uart_protocol.py`)
Define a robust, framed packet specification for STM32 DMA reception:
```text
Byte 0-1 : Header (0xAA 0x55)
Byte 2   : Message Type (0x01 = Detection/Pick Command, 0x02 = Heartbeat/Status)
Byte 3-4 : Sequence / Object ID (uint16_t)
Byte 5   : Class ID (0 = Non-Elco, 1 = Elco Benar, 2 = Elco Salah)
Byte 6-7 : Target X Position on Conveyor in mm (int16_t, e.g. 0.1 mm resolution)
Byte 8-9 : Target Y Position on Conveyor in mm (int16_t, e.g. 0.1 mm resolution)
Byte 10-11: Orientation Angle in degrees * 10 (int16_t, 0 - 3600)
Byte 12-13: Servo Correction in degrees * 10 (int16_t, 0 - 1800)
Byte 14-15: CRC16-CCITT Checksum
Byte 16-17: Delimiter / Tail (0x0D 0x0A -> \r\n)
```
- Provide both binary framing and a human-readable ASCII fallback mode for debug monitoring.
- Include camera-to-conveyor pixel-to-millimeter homography / scaling configuration.

### Step 3: Conveyor Tracking & Event Debouncing
- In `4_detect_realtime.py` (and test scripts), integrate a lightweight spatial tracker (Centroid / IoU Tracker).
- Ensure that as a capacitor travels through the camera's FOV across 30+ frames, it is evaluated once at the inspection line and queued **exactly once** into the pick queue.
- Prevent UART flooding. Ensure frame rate stays $\ge 25$ FPS on standard hardware.

### Step 4: Microcontroller Firmware Rectification (`firmware_esp32/` & STM32 spec)
- In `firmware_esp32.ino`:
  - **COMPLETELY REMOVE** `preferences.putString` / `simpanEntryNVS` from the real-time packet loop. Store logs only in volatile RAM buffers.
  - **REMOVE** all blocking `delay()` calls. Use non-blocking `millis()` state machine.
  - Support the new framed UART packet parsing with CRC16 verification.
- Document the exact C/C++ struct and FreeRTOS queue interface so Moris can integrate it directly into `MotorControls` for the STM32F446RE.

### Step 5: Portability & Launchers
- Modify all `.bat` files in `06_Launchers/` and `TEST_OFFLINE.bat`:
  - Replace `C:\Users\HUSEN\.conda\envs\...` with `python` or `%PYTHON_CMD%` dynamically checking for virtual environments.
  - Ensure scripts run out-of-the-box on any Windows machine where a Python 3.10+ virtual environment is active.
- In `1_train.py`, change default pretrained model to a valid Ultralytics release (e.g. `yolov8n.pt` or `yolo11n.pt`).

### Step 6: Automated Test & Verification Suite
Create an autonomous test script: `tests/verify_vision_system.py` that runs without requiring physical hardware:
1. **Test Angle Calculation:** Feeds synthetic/test images rotated at $15^\circ, 30^\circ, 45^\circ, 60^\circ, 135^\circ$ and asserts continuous angle error $\le 2.5^\circ$.
2. **Test Protocol Serialization & Checksum:** Encodes a packet, verifies CRC16, injects byte corruption to confirm rejection, and decodes successfully.
3. **Test Tracker Debounce:** Simulates 60 frames of a single capacitor moving along the Y-axis; asserts that only 1 UART pick command is triggered.
4. **Test Launcher Syntax:** Validates that batch scripts do not contain hardcoded user directories.

---

## 4. Strict Iteration & Completion Checklist

Do not consider your work done until every checkbox below is verified:

- [ ] `utils/angle_calculator.py` outputs continuous angles across $0^\circ$–$360^\circ$ (verified on non-orthogonal rotations).
- [ ] No hardcoded class name fallbacks for angle logic.
- [ ] Framed UART packet contains conveyor coordinates $(X, Y)$ in mm and CRC16 checksum.
- [ ] Conveyor tracker queues each capacitor exactly once without serial flooding.
- [ ] All flash-wear NVS writes removed from the real-time loop in `firmware_esp32.ino`.
- [ ] All blocking `delay()` calls removed from firmware.
- [ ] All hardcoded user paths (`C:\Users\HUSEN\...`) removed from `.bat` launchers.
- [ ] `python tests/verify_vision_system.py` runs and passes with exit code 0.
