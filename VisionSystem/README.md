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
