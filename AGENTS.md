# AGENTS.md — Global AI Assistant Instructions

## Engineering Baseline & Guardrails
The **`VisionSystem/`** technical overhaul and alignment with the thesis specification documented in [`VisionSystem/01_Dokumen_TA/TA_2026_extracted.txt`](VisionSystem/01_Dokumen_TA/TA_2026_extracted.txt) has been implemented and verified. Treat [`VisionSystem/REMEDIATION_PLAN_AI.md`](VisionSystem/REMEDIATION_PLAN_AI.md) as the **completed remediation baseline and regression checklist** whenever modifying the vision pipeline or host-to-STM32 protocol:

👉 **[`VisionSystem/REMEDIATION_PLAN_AI.md`](VisionSystem/REMEDIATION_PLAN_AI.md)**

---

## Critical Directives for AI Agents

1. **Strict Regression Verification:**  
   Do not declare any `VisionSystem/` or protocol change complete until all automated checks in `VisionSystem/tests/verify_vision_system.py` and the `VisionSystem/tests` pytest suite pass.
2. **Preserved Remediation Invariants:**
   - **Continuous Angle Calculation:** Keep `VisionSystem/04_Source_Code/utils/angle_calculator.py` computing continuous physical orientations across $[0^\circ, 360^\circ)$ with $\le \pm 2.5^\circ$ error (never revert to 4 discrete $0^\circ/90^\circ/180^\circ/270^\circ$ buckets).
   - **Telemetry Protocol & (X, Y) Coordinates:** Preserve the 18-byte binary `0xAA 0x55` framed UART protocol with conveyor coordinates $(X, Y)$ in mm, continuous angle, object ID, and CRC16-CCITT checksum for the STM32H723 (`SKR Octopus Pro V1.1`) Cartesian manipulator.
   - **Concurrency & Flooding:** Keep spatial object tracking active so moving capacitors are queued once rather than flooding UART on every camera frame.
   - **Embedded Firmware Safety:** In `VisionSystem/04_Source_Code/firmware_esp32/firmware_esp32.ino`, never reintroduce NVS Flash memory writes or blocking `delay()` calls in the packet reception loop.
   - **Portability:** Keep all `.bat` launchers in `VisionSystem/06_Launchers/` and `TEST_OFFLINE.bat` free of hardcoded laptop paths (`C:\Users\HUSEN\...` or fixed `.conda\envs\` paths).
3. **Automated Test Requirement:**  
   Run `VisionSystem/tests/verify_vision_system.py` (and `uv run --project VisionSystem pytest VisionSystem\tests`) after edits affecting vision math, UART framing, tracking, launchers, or ESP32 firmware.


# This is the Embedded Developer toolchains and Setups (ignore if you're not developing embedded systems or TinyML/Edge AI firmware or MotorControls).

# Developer Profile & System Toolchains

## User Preferences & Core Focus
- **Primary Domain**: Embedded Systems Engineering & Machine Learning (TinyML / Edge AI).
- **Core Languages**: C / C++, Python.
- **Secondary Systems Language**: Rust (specifically for modern MCU development using high-level async HALs like Embassy).
- **Embedded Development Style**: 
  - Uses **vendor HALs and SDKs** (e.g., **STM32Cube HAL/LL**, **CMSIS-DSP/NN**, vendor SDKs) rather than raw register-level bare metal from scratch.
  - Standard workflow: **STM32CubeMX** with **CMake export**, built with **CMake + Ninja**, and flashed/debugged via **`probe-rs`** or SEGGER J-Link.
- **Inactive / Not Used**: Java, .NET / C#, Web frameworks. Do not suggest or install .NET/Java tools.
- **Primary Editor**: **Zed** (`C:\Users\Moris\AppData\Local\Programs\Zed\bin`). VS Code and PlatformIO are not used.

## Installed Toolchain Reference

### 1. C & C++ (Host & Embedded)
- **Host Compilers**: LLVM-MinGW (Clang 23.1.1, `clang++`, `clangd`, `lld`) located at `C:\Tools\llvm-mingw\bin`.
- **Embedded ARM**: Arm GNU Toolchain 15.3.Rel1 (`arm-none-eabi-gcc` 15.3.1) at `C:\Tools\arm-gnu-toolchain-15.3.rel1-mingw-w64-x86_64-arm-none-eabi\bin`.
- **Embedded AVR**: AVR-GCC 16.1.0 and Avrdude 8.2 at `C:\Tools\avr-gcc-15.2.0-x64-windows\bin`.
- **Build Systems**: CMake 4.4.3 (`C:\Tools\cmake-4.2.2-windows-x86_64\bin`), Ninja 1.13.2 (`C:\Tools\ninja\ninja.exe`).
- **C/C++ Package Manager**: **vcpkg** (`2026-07-27`) located at `C:\Tools\vcpkg`.
  - Environment: `VCPKG_ROOT = C:\Tools\vcpkg`.
  - CMake Toolchain: `-DCMAKE_TOOLCHAIN_FILE=C:/Tools/vcpkg/scripts/buildsystems/vcpkg.cmake`.

### 2. Python & Machine Learning
- **Package & Environment Manager**: **`uv`** (0.12.19) at `C:\Users\Moris\AppData\Local\Microsoft\WinGet\Links\uv.exe`.
  - Workflow standard: Always use `uv` commands (`uv venv`, `uv add`, `uv run`, `uv pip`). Avoid calling raw `pip`.
- **Linter & Formatter**: **`ruff`** (0.16.9) at `C:\Users\Moris\.local\bin\ruff.exe`.
- **Active Interpreters**: Managed by `uv` (CPython 3.11, 3.12, 3.13; default 3.12.14 at `~/.local/bin/python.exe`).

### 3. Rust (Embedded & Systems)
- **Version**: Rust 1.98.1 / Cargo 1.98.1 located at `C:\Users\Moris\.cargo\bin`.
- **Host ABI**: `x86_64-pc-windows-gnu` (matches the LLVM-MinGW host toolchain).
- **Bare-Metal MCU Targets Installed**:
  - `thumbv7em-none-eabihf` (Cortex-M4F / Cortex-M7F with hardware FPU - TinyML & STM32F4/F7/H7).
  - `thumbv8m.main-none-eabihf` (Cortex-M33 / Cortex-M55 with Arm Helium / SIMD DSP for Edge AI).
  - `thumbv7em-none-eabi` (Cortex-M4/M7 soft-float).
  - `thumbv7m-none-eabi` (Cortex-M3).
  - `thumbv6m-none-eabi` (Cortex-M0 / Cortex-M0+ / RP2040).
- **Embedded Utilities**:
  - **`probe-rs`** (0.32.0): Universal flasher, debugger, and real-time RTT telemetry tool over SWD/JTAG (works with both C/C++ and Rust firmware).
  - **`flip-link`**: Zero-cost stack overflow protection for Cortex-M.
  - **`cargo-generate`**: Scaffolding for Embassy and Cortex-M templates.
  - **`cargo-binstall`**: Fast binary package installer for Cargo tools.
  - **`rust-analyzer`**: Language server installed for Zed.

### 4. CLI, Terminal & Environment
- **Productivity Tools**:
  - `just`: Modern task/recipe runner.
  - `zoxide`: Directory jumper (`z <dir>`, `zi` for interactive).
  - `fzf`: Interactive fuzzy finder.
  - `fd`: Fast file finder.
  - `bat`: Syntax highlighting file viewer (alias `b`).
  - `jq`: Command-line JSON processor.
  - `rg` (ripgrep): Code search.
  - `git` & `gh`: Version control & GitHub CLI.
- **PowerShell Profile**: Located at `C:\Users\Moris\Documents\PowerShell\Microsoft.PowerShell_profile.ps1`.
  - Configured with PSReadLine predictive IntelliSense, F2 view switcher, and Up/Down history substring matching.
- **Environment Status**:
  - System PATH: `C:\msys64\ucrt64\bin` is permanently removed.
  - User PATH: Completely deduplicated and cleaned (15 essential entries, zero dead paths).

### 5. Hardware & Debug Probes
- **Primary Debug Probe**: **ST-LINK converted to SEGGER J-Link** (reflashed with Segger on-board J-Link firmware).
  - **Probe Identification**: Presents to Windows, USB drivers, and debuggers as a **J-Link** probe (not ST-Link).
  - **Tooling Support**: 
    - `probe-rs`: Automatically recognizes it as a J-Link probe for zero-config flashing, DAP debugging, and RTT streaming.
    - Local Utilities: Located at `C:\\tools\\stm32-jlink-starter` (in User PATH) and `C:\\tools\\jlink`.
    - OpenOCD / GDB: Always target `interface/jlink.cfg` or Segger J-Link GDB Server (`-device STM32H723ZE` for SKR Octopus Pro V1.1). Never suggest ST-Link specific tools (like `st-flash` or `stlink.cfg`).
  - **Telemetry**: Full support for high-speed SEGGER RTT (Real-Time Transfer) without halting the MCU core.
