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


# This is the Embedded Developer toolchains and Setups (ignore if you're not developing embedded systems or TinyML/Edge AI firmware or MotorControls).

# Developer Profile & System Toolchains

## User Preferences & Core Focus
- **Primary Domain**: Embedded Systems Engineering & Machine Learning (TinyML / Edge AI).
- **Core Languages**: C / C++, Python.
- **Secondary Systems Language**: Rust (specifically for modern MCU development using high-level async HALs like Embassy).
- **Embedded Development Style**: 
  - Uses **vendor HALs and SDKs** (e.g., **STM32Cube HAL/LL**, **CMSIS-DSP/NN**, vendor SDKs) rather than raw register-level bare metal from scratch.
  - Standard workflow: **STM32CubeMX** with **CMake export**, built with **CMake + Ninja**, and flashed/debugged via **`probe-rs`**.
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
    - Local Utilities: Located at `C:\tools\stm32-jlink-starter` (in User PATH).
    - OpenOCD / GDB: Always target `interface/jlink.cfg` or Segger J-Link GDB Server. Never suggest ST-Link specific tools (like `st-flash` or `stlink.cfg`).
  - **Telemetry**: Full support for high-speed SEGGER RTT (Real-Time Transfer) without halting the MCU core.
