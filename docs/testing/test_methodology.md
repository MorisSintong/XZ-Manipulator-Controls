# Testing Methodology & Acceptance Criteria

**Project:** XZ-Manipulator-Controls Quality Control & Sorting System  
**Test Reference Document:** [`Perencanaan_Test_Realcase_XZ_Manipulator_Lengkap.xlsx`](Perencanaan_Test_Realcase_XZ_Manipulator_Lengkap.xlsx)  

---

## 1. Testing Hierarchy

To ensure system reliability before full-speed conveyor integration, the verification process follows a multi-tier V-model testing architecture:

```text
               +-------------------------------------------+
               | Level 4: End-to-End Sorting Integration   |
               +-------------------------------------------+
                                   ^
               +-------------------------------------------+
               | Level 3: Hardware-in-the-Loop (HIL)       |
               +-------------------------------------------+
                                   ^
               +-------------------------------------------+
               | Level 2: Subsystem Bench Tests            |
               +-------------------------------------------+
                                   ^
               +-------------------------------------------+
               | Level 1: Unit & Driver Verification       |
               +-------------------------------------------+
```

---

## 2. Test Suites & Verification Matrix

### Level 1: Unit & Driver Verification
- **AS5600 Magnetic Encoder Driver:**
  - Fast-read register integrity, magnet status diagnostics (`MD`, `ML`, `MH`), noise variance measurement at standstill.
  - Test harness located at [`MotorControls/Drivers/AS5600/tests/`](../../MotorControls/Drivers/AS5600/tests).
- **TMC2240 Stepper Driver:**
  - SPI communication verify (`IFCNT`, `GCONF`, `DRV_STATUS`).
  - Current scaling (`GLOBALSCALER`, `IHOLD_IRUN`) and microstep accuracy ($1/16$, $1/256$).
  - StallGuard4 homing threshold tuning (`SG4_THRS`) without hard mechanical crashes.
  - Test harness located at [`MotorControls/Drivers/TMC2240-Driver/Tests/`](../../MotorControls/Drivers/TMC2240-Driver/Tests).

### Level 2: Subsystem Bench Tests
- **Vision Pipeline Unit Tests:**
  - Offline image inference against test dataset.
  - Continuous angle calculation error benchmark (asserting $\Delta \theta \le \pm 2.5^\circ$ across all orientations $[0^\circ, 360^\circ)$).
  - Pixel-to-millimeter mapping and optional homography calibration check.
  - Framed UART serialization and CRC16 validation check.
- **Motion Firmware & Protocol Mock Bench Tests:**
  - Dual-axis limit bouncing and homing sequence (`motion_diagnostics`, `stallguard_cli`).
  - STM32H723 UART binary frame reception, CRC16 validation, fault injection, and 10-frame burst stress test ([`MotorControls/tests/emulation/`](../../MotorControls/tests/emulation)).
  - Vacuum suction pull/release timing response ($< 150\ \text{ms}$) *(hardware integration pending)*.

### Level 3: Hardware-in-the-Loop (HIL)
- **Conveyor Tracking & Synchronization:**
  - Conveyor speed tracking using encoder feedback vs proximity sensors.
  - Spatial debounce test: ensuring moving capacitors across the 30 FPS camera feed trigger exactly **one** motion queue command.
- **Repeatability & Accuracy Tests:**
  - Positioning repeatability test ($X$ and $Z$ axes) over 100 consecutive cycles ($\le \pm 0.2\ \text{mm}$).
  - End-effector servo angle resolution and backlash measurement.

### Level 4: Real-Case End-to-End Sorting
- **Sorting Success Rate:**
  - Testing batches of $N = 100$ capacitors with varied orientations and random mixed polarities.
  - Success criteria: $\ge 95\%$ correct classification and placement accuracy.
- **Cycle Time:**
  - Total elapsed time per pick-inspect-rotate-place cycle ($\le 2.5\ \text{seconds}$).

---

## 3. Automated Test Execution Commands

See [`docs/testing/README.md`](README.md) for the complete host and firmware verification command matrix.

### Vision System Programmatic Verification
```powershell
# Run the automated vision verification suite (Angle, CRC, Debounce)
uv run --project VisionSystem python VisionSystem\tests\verify_vision_system.py
# Run binary transport & worker thread tests
uv run --project VisionSystem pytest VisionSystem\tests
# Or run standalone offline validation
uv run --project VisionSystem python VisionSystem\04_Source_Code\3_test_offline.py
```

### STM32 UART Protocol & Driver Verification
```powershell
# 1. STM32H723 UART Protocol Emulation Suite (11 test cases, mock server)
uv run --project VisionSystem python MotorControls\tests\emulation\run_tests.py --mode mock

# 2. AS5600 Host CTest verification
cmake -S MotorControls\Drivers\AS5600 -B MotorControls\Drivers\AS5600\build -G Ninja
cmake --build MotorControls\Drivers\AS5600\build
ctest --test-dir MotorControls\Drivers\AS5600\build --output-on-failure
```

---

## 4. Test Matrix Tracking Sheet

All quantitative measurements, sensor readings, motor current logs, and sorting success tallies are recorded in the official spreadsheet:  
👉 [`Perencanaan_Test_Realcase_XZ_Manipulator_Lengkap.xlsx`](Perencanaan_Test_Realcase_XZ_Manipulator_Lengkap.xlsx)
