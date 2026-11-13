# VisionSystem Subsystem

Machine vision inspection, continuous capacitor orientation tracking, and real-time UART telemetry pipeline for the XZ Manipulator Quality Control cell.

The host pipeline uses a cooperative STM32 diagnostic firmware profile; it does not depend on a FreeRTOS task scheduler. Pixel coordinates are mapped with configured per-axis scale and origin offsets by default. The mapper supports an optional homography, but the live launcher does not load a calibrated homography file.

---

## 1. Overview & Pipeline Architecture

The **VisionSystem** handles overhead automated optical inspection (AOI) of electrolytic capacitors traveling on an industrial conveyor:
1. **Object Detection:** Detects capacitors using Ultralytics YOLO models.
2. **Orientation & Polarity Estimation:** Calculates continuous physical orientation across $[0^\circ, 360^\circ)$ using Principal Component Analysis (PCA) and localized polarity strip/lead analysis.
3. **Spatial Tracking:** Centroid-based conveyor tracking with debounce thresholds to prevent duplicate triggers on moving parts.
4. **Coordinate Transformation:** Pixel-to-millimeter mapping from camera coordinates $(u, v)$ to conveyor coordinates $(X, Y)$ using configured scale and origin offsets; an optional homography is available through the mapper API.
5. **Framed Telemetry Dispatch:** Packages target data into 18-byte binary frames with CRC16-CCITT and transmits to the **STM32H723ZET6** (`BIGTREETECH SKR Octopus Pro V1.1`) motion controller over `USART2` (`PD5`/`PD6`).

```text
 +---------------------+       +---------------------+       +---------------------+
 |  Overhead Camera    |  ➔    |  YOLO Detection &   |  ➔    |  PCA Orientation &  |
 |  (USB Video Stream) |       |  Bounding Box ROI   |       |  Polarity Analyzer  |
 +---------------------+       +---------------------+       +---------------------+
                                                                        |
 +---------------------+       +---------------------+                 v
 | STM32H723ZET6 (SKR) |      |  Framed UART Packet |      |  Spatial Conveyor   |
 | (Cooperative Diag.) |       |  (18B + CRC16)      |       |  Tracker & Debounce |
 +---------------------+       +---------------------+       +---------------------+
```

---

## 2. Directory Structure

```text
VisionSystem/
├── README.md                      # This subsystem guide
├── REMEDIATION_PLAN_AI.md         # Historical remediation spec & regression guardrails (complete)
├── pyproject.toml                 # Project manifest & editable motion-diagnostics dependency
├── TEST_OFFLINE.bat               # Top-level offline test convenience script
├── 01_Dokumen_TA/                 # Official thesis proposal & specifications (TA_2026_extracted.txt)
├── 04_Source_Code/                # Core Python source code
│   ├── 1_train.py                 # YOLO model training
│   ├── 2_evaluate.py              # Precision, recall, and mAP evaluation
│   ├── 3_test_offline.py          # Offline batch validation on test images
│   ├── 4_detect_realtime.py       # Live camera detection, tracking, and binary UART streaming
│   ├── 5_test_single_image.py     # Single image test with annotated overlay
│   ├── 6_test_video.py            # Video stream simulation test
│   ├── 7_test_frame_read.py       # Camera acquisition latency benchmark
│   ├── diagnostik_uart.py         # Serial port loopback and diagnosis utility
│   ├── test_uart_esp32.py         # Auxiliary ESP32 UART bridge test script
│   ├── firmware_esp32/            # Remediated non-blocking ESP32 serial bridge sketch (RAM log buffer)
│   └── utils/                     # Core vision & transport modules
│       ├── angle_calculator.py    # Continuous PCA orientation [0°, 360°) & polarity estimation
│       ├── conveyor_tracker.py    # Centroid tracker with inspection-line FIFO debounce
│       ├── uart_protocol.py       # 18-byte frame encoder/decoder, CRC16 & pixel-to-mm mapper
│       ├── vision_command.py      # Runtime angle quantization, wrapping & servo sign conversion
│       ├── uart_handler.py        # Bounded non-blocking serial owner (KoneksiUART)
│       └── diag_receiver.py       # Background STM32 binary reply decoder & CSV/raw logger
├── 06_Launchers/                  # Double-clickable portable Windows batch launchers
├── SERVO/                         # Legacy standalone ESP32 servo sketch (see caution below)
└── tests/                         # Automated offline verification suites
    ├── verify_vision_system.py    # Continuous angle, 18B framing, CRC16, debounce & launcher checks
    └── test_diag_transport.py     # Binary transport, correlation, fault bitmask & worker tests
```

> [!CAUTION]
> `VisionSystem/SERVO/SERVO.ino` is a legacy standalone bench sketch that still uses the old ASCII `K<id>,S<angle>,R<corr>` format, blocking `delay()` calls, and NVS `Preferences` flash writes on every packet. For ESP32 bridge testing, use the remediated non-blocking [`04_Source_Code/firmware_esp32/firmware_esp32.ino`](04_Source_Code/firmware_esp32/firmware_esp32.ino) instead.

---

## 3. Environment Setup (using `uv`)

`VisionSystem/pyproject.toml` supports **Python 3.10+** (examples below use Python 3.12). Binary motion diagnostics uses the workspace's shared [`packages/motion_diagnostics`](../packages/motion_diagnostics/README.md) package installed normally via `uv`—never a `sys.path` workaround.

Run from the **repository root**:

```powershell
# 1. Create VisionSystem\.venv and install declared transport, OpenCV, NumPy & test dependencies
uv sync --project VisionSystem --python 3.12

# 2. Install Ultralytics into the same environment for YOLO camera/model inference
uv pip install --python VisionSystem\.venv\Scripts\python.exe ultralytics

# 3. Verify offline suites
uv run --project VisionSystem python VisionSystem\tests\verify_vision_system.py
uv run --project VisionSystem pytest VisionSystem\tests
```

---

## 4. Runtime Transport, Readiness & Angle Contract

### A. Serial Port Ownership & Non-Blocking Worker Queues
- **Single Serial Owner:** Pass an explicit STM32 VCOM/UART port such as `--uart-port COM9` (`AUTO` succeeds only when exactly one serial port is present). Do **not** run the [`motion-diag`](../MotorControls/MotionFirmware/tools/diag/README.md) bench CLI and live vision simultaneously on the same port.
- **Byte-Preserving Admission:** `KoneksiUART.kirim` accepts raw 18-byte `bytes` frames only (text strings are rejected) and reports bounded TX queue admission—**not** confirmed physical motion execution.
- **Decoupled Threads:** Background RX, TX, and CSV worker threads (`DiagnosticReceiver`) own all serial I/O and disk writes; detection never calls `readline`, waits for ASCII ACKs, or blocks on motion completion.
- **Session & Fault Handling:** Queue overflow or sustained serial/disk I/O failure inhibits new picks without crashing inference. Uncertain pick commands are **never** automatically retried. Reconnecting opens a new host session and requires readiness confirmation again.
- **Telemetry Artifacts:** By default, sessions write `results\vision_diag_<UUID>.csv`, a `.health.json` transport health sidecar (`diagnostic_health` exposes CRC/resync counts, missing completions, and serial/disk backpressure), and a raw `.bin` capture. Pass `csv_path` and a qualified run `config` to customize paths; without a matching qualified config, rows are recorded in CSV but excluded from nominal acceptance metrics.

### B. Readiness & Fault Bitmask Policy (`v1`)
- **Gate Before Picks:** An identity-matched homed/`READY` status is required before pick commands are admitted. Only `STATUS` (`0x83`) and `HOME_RESULT` (`0x82`) update readiness; per-command rejections are logged and counted without stalling subsequent objects while awaiting a heartbeat.
- **Fault Mask (`0x0003E7E2`):** Readiness requires both axes homed and zero active fault bits under `0x0003E7E2` (which includes active `TX_BACKPRESSURE`, bit 13).
  - `RX_OVERFLOW` (bit 12) is a per-record event bit and does not latch readiness off.
  - `MSCNT_UNQUALIFIED` (bit 11) is informational and is recorded independently without excluding healthy AS5600 encoder metrics.
- **Control Timeouts:** Unanswered control commands expire cleanly, and heartbeat object IDs rotate while polling for readiness.

### C. Orientation Quantization & Servo Correction
- **Angle Wrapping (`0…3599`):** Runtime command construction quantizes orientation to $0.1^\circ$ and wraps into `0…3599` (so $359.95^\circ$ quantizes to `0`, i.e., $0.0^\circ$).
- **Signed Shortest-Turn Correction (`−180.0°…+180.0°`):** Converts the vision pipeline's clockwise $0^\circ\text{–}360^\circ$ correction into signed $[-180^\circ, +180^\circ]$:
  - Values $> 180^\circ$ map to the negative equivalent (e.g., $270^\circ \to -90^\circ$).
  - The exact half-turn tie is **`+180°`** (including input $-180^\circ$).
- **Input Validation:** The deployed 18-byte wire format and CRC16-CCITT (over bytes `0…13`) are unchanged. Unsupported class IDs and malformed or out-of-range packets are logged and counted as admission failures rather than raising exceptions into the camera loop.

---

## 5. Running the Vision Pipeline

### Live Real-Time Detection & Sorting Stream
```powershell
# Launch real-time detection with explicit STM32 UART port
uv run --project VisionSystem python VisionSystem\04_Source_Code\4_detect_realtime.py --uart-port COM9

# Dry-run vision & tracking overlay without UART hardware
uv run --project VisionSystem python VisionSystem\04_Source_Code\4_detect_realtime.py --no-uart

# Optional calibration & tracker flags
uv run --project VisionSystem python VisionSystem\04_Source_Code\4_detect_realtime.py `
  --uart-port COM9 --source 0 --inspection-line 0.5 --px-per-mm-x 5.0 --px-per-mm-y 5.0

# Or use the portable Windows batch launcher:
.\VisionSystem\06_Launchers\3_UJI_REALTIME.bat
```

### Offline Testing & Model Evaluation
```powershell
# Offline batch validation on static test images
uv run --project VisionSystem python VisionSystem\04_Source_Code\3_test_offline.py

# Precision, recall, and mAP evaluation
uv run --project VisionSystem python VisionSystem\04_Source_Code\2_evaluate.py
```

---

## 6. Verification & Automated Testing

Run both offline test suites without physical camera or MCU hardware connected:

```powershell
# 1. Thesis remediation suite (PCA angle accuracy, 18B CRC16 framing, FIFO debounce, launcher portability)
uv run --project VisionSystem python VisionSystem\tests\verify_vision_system.py

# 2. Binary diagnostics transport, correlation, readiness bitmask & worker thread tests
uv run --project VisionSystem pytest VisionSystem\tests
```

---

## 7. Protocol & Diagnostics References

- **Host ➔ STM32 18-byte command protocol & CRC16:** [`docs/architecture/communication_protocol.md`](../docs/architecture/communication_protocol.md)
- **STM32 ➔ Host binary diagnostic records & metrology contract:** [`docs/architecture/motion_diagnostics_design.md`](../docs/architecture/motion_diagnostics_design.md)
- **Shared Python diagnostic codec & metrology:** [`packages/motion_diagnostics/README.md`](../packages/motion_diagnostics/README.md)
- **Host bench CLI (`motion-diag`):** [`MotorControls/MotionFirmware/tools/diag/README.md`](../MotorControls/MotionFirmware/tools/diag/README.md)
