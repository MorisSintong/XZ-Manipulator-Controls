# MotionFirmware — STM32F446RE XZ Manipulator Firmware Suite

STM32F446RE (Nucleo-64) firmware suite for the 2-Axis (XZ) Cartesian manipulator, driving two **Trinamic TMC2240** stepper drivers (shared `SPI2` + `STEP`/`DIR`) and reading dual **AS5600** magnetic encoders (`I2C1` / `I2C3`).

This project uses a **single shared CubeMX `Core/` and `Drivers/` layer** (`MotionFirmware.ioc`) with selectable application profiles under [`Apps/`](Apps/):

| Application Profile | Folder | Output Binary (`.elf` / `.hex` / `.bin`) | Description |
| :--- | :--- | :--- | :--- |
| **`motion_diagnostics`** *(default)* | [`Apps/motion_diagnostics/`](Apps/motion_diagnostics/README.md) | `MotionFirmware_Diagnostics.*` *(and default `MotionFirmware.*`)* | Binary-only `USART2` DMA vision pick executor (`0x01..0x04`), single-end StallGuard4 homing (Z then X), 1 MHz timer compare stepping (`TIM2`/`TIM5`), and 356/124/116-byte binary evidence records (`MSCNT` + unwrapped AS5600 + step counts) for the Python [`tools/diag`](tools/diag/README.md) analyzer and live `VisionSystem`. |
| **`stallguard_cli`** | [`Apps/stallguard_cli/`](Apps/stallguard_cli/README.md) | `MotionFirmware_StallGuardCLI.*` | Interactive ASCII CLI (`USART2` + SEGGER RTT) with two-end StallGuard4 sensorless homing, full travel & encoder-ratio calibration, continuous **`soft`** and **`bounce`** cycling modes, and live closed-loop step-loss supervision. |
| **`z_bounce_emulation`** | [`Apps/z_bounce_emulation/`](Apps/z_bounce_emulation/) | `MotionFirmware_ZBounceEmulation.*` | Single-axis Z StallGuard4 bounce demo + `VisionUart` ASCII/binary protocol responder used by the Renode HIL emulation test suite (`MotorControls/tests/emulation/`). |

---

## 1. Directory Layout

```text
MotorControls/MotionFirmware/
├── Core/                         # Shared CubeMX HAL initialization & interrupt vector hooks (fw_app.h)
├── Drivers/                      # Shared STM32F4xx_HAL_Driver (SPI, I2C, TIM, UART, DMA, GPIO) + CMSIS
├── Apps/
│   ├── motion_diagnostics/       # Profile 1: Binary vision executor & metrology evidence firmware
│   │   ├── Inc/ & Src/           # motion_executor, diag_record, step_engine, encoder_sampler, homing...
│   │   ├── Tests/                # 21 host CTest suites + ARM compile check + golden fixture writer
│   │   └── README.md             # Detailed guide for motion_diagnostics
│   ├── stallguard_cli/           # Profile 2: Interactive StallGuard4 homing & soft/bounce CLI
│   │   ├── Inc/ & Src/           # app, axis_ctrl, stall_detect, stepgen, tmc_axis, encoder, console...
│   │   ├── tests/                # Host closed-loop physics/StallGuard/AS5600 simulation CTest suite
│   │   └── README.md             # Detailed guide for stallguard_cli
│   └── z_bounce_emulation/       # Profile 3: Z-bounce demo & Renode UART emulation responder
│       └── Inc/ & Src/           # z_bounce_app.c, vision_uart_protocol.c/.h
├── tools/diag/                   # Host Python CLI (capture, send-sequence, analyze, heartbeat, abort, rehome)
├── MotionFirmware.ioc            # Single source-of-truth STM32CubeMX hardware configuration
├── CMakeLists.txt                # Builds all three firmware targets side-by-side
└── CMakePresets.json             # Presets for Debug/Release and default FIRMWARE_APP selection
```

---

## 2. Building All Firmware Targets

A single CMake build compiles the shared STM32 HAL once and produces `.elf`, `.hex`, and `.bin` images for **all three profiles** side-by-side in `build/<Preset>/`:

```powershell
# Default preset (MotionFirmware.hex = motion_diagnostics, plus all 3 named targets):
cmake --preset Debug
cmake --build --preset Debug

# Or set MotionFirmware.hex to the interactive StallGuard CLI profile:
cmake --preset Debug-StallGuard
cmake --build --preset Debug-StallGuard
```

---

## 3. Running Host Unit Tests & Verification Suites

### A. `motion_diagnostics` Host CTest + Cortex-M4 Compile Check (21 tests)
```powershell
.\Apps\motion_diagnostics\Tests\run_tests.ps1
```

### B. `stallguard_cli` Host Closed-Loop Simulation CTest (3 suites)
```powershell
cmake -S Apps/stallguard_cli/tests -B Apps/stallguard_cli/tests/build-host -G Ninja -DCMAKE_C_COMPILER=gcc
cmake --build Apps/stallguard_cli/tests/build-host
ctest --test-dir Apps/stallguard_cli/tests/build-host --output-on-failure
```

### C. Host Python Diagnostics Library & CLI Tests (run from repository root)
```powershell
uv run --project packages/motion_diagnostics pytest packages/motion_diagnostics/tests
uv run --project MotorControls/MotionFirmware/tools/diag pytest MotorControls/MotionFirmware/tools/diag/tests
uv run --project VisionSystem pytest VisionSystem/tests/test_diag_transport.py
```
