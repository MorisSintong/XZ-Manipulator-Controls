# VisionSystem Subsystem

Machine vision inspection, continuous capacitor orientation tracking, and real-time UART telemetry pipeline for the XZ Manipulator Quality Control cell.

---

## 1. Overview & Pipeline Architecture

The **VisionSystem** handles overhead automated optical inspection (AOI) of electrolytic capacitors traveling on an industrial conveyor:
1. **Object Detection:** Detects capacitors using Ultralytics YOLO models.
2. **Orientation & Polarity Estimation:** Calculates continuous physical orientation across $[0^\circ, 360^\circ)$ using Principal Component Analysis (PCA) and localized polarity strip/lead analysis.
3. **Spatial Tracking:** Centroid-based conveyor tracking with debounce thresholds to prevent duplicate triggers on moving parts.
4. **Coordinate Transformation:** Homography mapping from camera pixel coordinates $(u, v)$ to conveyor frame physical coordinates $(X, Y)$ in millimeters.
5. **Framed Telemetry Dispatch:** Packages target data into 18-byte binary frames with CRC16-CCITT and transmits to the STM32F446RE motion controller over UART.

```text
 +---------------------+       +---------------------+       +---------------------+
 |  Overhead Camera    |  ➔    |  YOLO Detection &   |  ➔    |  PCA Orientation &  |
 |  (1080p @ 30 FPS)   |       |  Bounding Box ROI   |       |  Polarity Analyzer  |
 +---------------------+       +---------------------+       +---------------------+
                                                                        |
 +---------------------+       +---------------------+                  v
 | STM32 FreeRTOS      |      |  Framed UART Packet |      |  Spatial Conveyor   |
 | Motion Controller   |       |  (18B + CRC16)      |       |  Tracker & Debounce |
 +---------------------+       +---------------------+       +---------------------+
```

---

## 2. Directory Structure

```text
VisionSystem/
├── README.md                      # This subsystem guide
├── REMEDIATION_PLAN_AI.md         # Active engineering audit & thesis compliance roadmap
├── 01_Dokumen_TA/                 # Official thesis proposal & specifications (TA_2026_extracted.txt)
├── 04_Source_Code/                # Core Python source code
│   ├── 1_train.py                 # YOLO model training
│   ├── 2_evaluate.py              # Precision, recall, and mAP evaluation
│   ├── 3_test_offline.py          # Offline batch validation on test images
│   ├── 4_detect_realtime.py       # Live camera detection, tracking, and UART streaming
│   ├── 5_test_single_image.py     # Single image test with annotated overlay
│   ├── 6_test_video.py            # Video stream simulation test
│   ├── 7_test_frame_read.py       # Camera acquisition latency benchmark
│   ├── diagnostik_uart.py         # Serial port loopback and diagnosis utility
│   ├── firmware_esp32/            # Auxiliary ESP32 serial bridge firmware
│   └── utils/                     # Angle calculation, camera homography, UART protocols
├── 06_Launchers/                  # Double-clickable Windows batch launcher scripts
└── tests/                         # Automated programmatic verification suites
    └── verify_vision_system.py    # Angle, framing, CRC16, and debounce tests
```

---

## 3. Environment Setup (using `uv`)

Binary motion diagnostics uses the normal installed shared package, never a
`sys.path` workaround. From repository root:

```powershell
uv sync --project VisionSystem
uv pip install --python VisionSystem\.venv\Scripts\python.exe -e packages\motion_diagnostics
uv run --project VisionSystem python VisionSystem\tests\verify_vision_system.py
uv run --project VisionSystem pytest
```

The manifest includes the verification/transport dependencies. Install the
existing YOLO runtime dependencies into this same environment for camera use.
Use an explicit STM32 VCOM port (AUTO succeeds only with exactly one port).
`KoneksiUART.kirim` accepts original binary `bytes` and reports bounded queue
admission, **not** confirmed physical execution. A matched homed/READY heartbeat
is required before picks. A reader owns all RX; no ASCII ACK/readline path is
used. RX/TX/CSV worker threads never wait on detection. CRC/resync, missing
completions and disk/serial backpressure are exposed by `diagnostic_health`.
Queue overflow or sustained I/O error inhibits new picks; uncertain commands
are never automatically retried. Reconnect creates a new session and requires
readiness again. Default logs are `results\vision_diag_<UUID>.csv` with a health
sidecar and raw `.bin` capture; pass `csv_path` and qualified run `config` to the serial wrapper to use
another local path/config. Without qualified matching config, values are
recorded but excluded from acceptance metrics. Do not run bench sender and
vision simultaneously on the same port.

Runtime command construction quantizes then wraps orientation into 0…3599
(359.95° becomes 0°), and converts the vision algorithm's clockwise 0…360°
correction into an equivalent signed −180…+180° rotation. The exact half-turn
tie is **+180°** (including input −180°); >180° uses the negative equivalent
(270° → −90°). The deployed 18-byte wire format and CRC are unchanged.
Unsupported class IDs and malformed/out-of-range packets are logged and counted
as admission failures, not raised into inference. Readiness is homed with no
fault bits; informational/event bits do not disable it. MSCNT qualification is
recorded independently and does not exclude healthy encoder-based metrics.
The v1 fault mask is `0x0003E7E2`, including active TX backpressure (bit 13);
RX_OVERFLOW (bit 12) is an event and MSCNT_UNQUALIFIED (bit 11) is informational.
Only STATUS and HOME_RESULT update readiness; per-command rejections remain
logged/counted but do not pause the next object while awaiting a heartbeat.
Unanswered control commands expire, and heartbeat IDs rotate while waiting for
readiness; uncertain pick commands are never automatically retried.

See [diagnostic tool usage](../MotorControls/MotionFirmware/tools/diag/README.md)
and [wire/measurement contract](../docs/architecture/motion_diagnostics_design.md).

This repository uses [`uv`](https://github.com/astral-sh/uv) for fast, deterministic Python virtual environment and dependency management.

```powershell
# 1. Create a virtual environment with Python 3.12
uv venv --python 3.12

# 2. Activate the virtual environment
.venv\Scripts\activate

# 3. Install required dependencies
uv pip install ultralytics opencv-python numpy pyserial pytest
```

---

## 4. Running the Vision Pipeline

### Live Real-Time Detection & Sorting Stream
```powershell
# Launch real-time detection on the default camera (Camera 0)
uv run python VisionSystem/04_Source_Code/4_detect_realtime.py

# Or use the launcher script:
.\VisionSystem\06_Launchers\3_UJI_REALTIME.bat
```

### Offline Testing on Static Images
```powershell
uv run python VisionSystem/04_Source_Code/3_test_offline.py
```

### Model Evaluation
```powershell
uv run python VisionSystem/04_Source_Code/2_evaluate.py
```

---

## 5. Verification & Automated Testing

To verify angle calculation accuracy, UART framing, CRC16 checksums, and tracker debouncing without physical hardware connected:

```powershell
uv run python VisionSystem/tests/verify_vision_system.py
```

---

## 6. Communication Protocol

For the complete 18-byte binary frame definition and CRC16 specification connecting to the STM32 controller:  
👉 **[`docs/architecture/communication_protocol.md`](../docs/architecture/communication_protocol.md)**
