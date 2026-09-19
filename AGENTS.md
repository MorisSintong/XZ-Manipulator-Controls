# AGENTS.md — Global AI Assistant Instructions

## Active Engineering Objective
The primary active task in this repository is the complete technical overhaul and alignment of the **`VisionSystem/`** module to match the thesis engineering specification documented in [`VisionSystem/01_Dokumen_TA/TA_2026_extracted.txt`](VisionSystem/01_Dokumen_TA/TA_2026_extracted.txt).

Before performing any code modifications or claiming completion, any AI assistant MUST read and strictly adhere to:
👉 **[`VisionSystem/REMEDIATION_PLAN_AI.md`](VisionSystem/REMEDIATION_PLAN_AI.md)**

---

## Critical Directives for AI Agents

1. **Strict Completion Verification:**  
   You are NOT permitted to stop or declare the task done until all criteria in the checklist within `VisionSystem/REMEDIATION_PLAN_AI.md` are verified via programmatic tests.
2. **Key Faults That Must Be Resolved:**
   - **Continuous Angle Calculation:** Fix `VisionSystem/04_Source_Code/utils/angle_calculator.py` to stop quantizing angles into 4 discrete buckets ($0^\circ, 90^\circ, 180^\circ, 270^\circ$). The system must compute continuous physical orientations across $[0^\circ, 360^\circ)$ with $\le \pm 2.5^\circ$ error.
   - **Telemetry Protocol & (X, Y) Coordinates:** Upgrade UART messages in `4_detect_realtime.py` to include conveyor coordinates $(X, Y)$ in mm, continuous angle, object ID, and CRC16 checksum so the STM32 2-axis Cartesian manipulator can execute motion planning.
   - **Concurrency & Flooding:** Implement spatial object tracking so moving capacitors are queued once rather than flooding UART on every camera frame.
   - **Embedded Firmware Safety:** In `firmware_esp32/firmware_esp32.ino`, remove destructive NVS Flash memory writes from the packet reception loop and replace blocking `delay()` with non-blocking timers.
   - **Portability:** Eliminate all hardcoded laptop paths (`C:\Users\HUSEN\...`) from `.bat` launchers and make them run on any standard Windows Python environment.
3. **Automated Test Requirement:**  
   Provide and run `tests/verify_vision_system.py` asserting continuous angle accuracy, packet framing/CRC, and tracking debounce.
