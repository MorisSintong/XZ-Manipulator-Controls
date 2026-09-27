# MotionFirmware – XZ Manipulator Motion Controller

STM32F446RE (Nucleo-64) firmware that drives the two NEMA 17 axes of the 2-Axis (XZ) Cartesian manipulator through **Trinamic TMC2240** drivers (SPI + STEP/DIR) and reads dual **AS5600** magnetic encoders (I2C).

The application features **StallGuard4** sensorless homing, travel measurement, and continuous soft/bounce cycle modes. The motion layer accepts absolute position targets in millimeters, coordinating pick-and-place trajectories commanded by the Vision System.

---

## 1. Hardware Mapping

Full wiring schematics, pinouts, and power distribution rules:  
👉 **[`docs/hardware/wiring.md`](../../docs/hardware/wiring.md)**

### Axis Mapping Summary
*(Configured in `App/Inc/app_config.h` and `App/Src/app_config.c`)*

| Axis | Driver | CS | STEP | DIR | ENN | Step Timer | AS5600 Bus |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **0 = Z** (T8x8 Lead Screw, 400 µsteps/mm) | TMC2240 #0 | `PC0` | `PC1` | `PC2` | `PC6` | `TIM2` | `I2C3` (PA8 SCL, PC9 SDA) |
| **1 = X** (GT2 20T Belt, 80 µsteps/mm) | TMC2240 #1 | `PC3` | `PC4` | `PC5` | `PC7` | `TIM5` | `I2C1` (PB8 SCL, PB9 SDA) |

- **Shared SPI2** (Mode 3, 2.625 MHz): `PB13` (SCK), `PB14` (MISO), `PB15` (MOSI).
- **Console / Telemetry:** Dual broadcast on `USART2` (Virtual COM Port `COM9` @ 115200 8-N-1) and **SEGGER RTT** (Terminal 0).

> [!NOTE]
> The thesis document (TA 2026, Tables 3.8/3.9 and Section 3.6.2) assigns motor 0 to X and swaps the encoder buses. This firmware follows `docs/hardware/wiring.md`. If your physical harness follows the thesis proposal, swap the two entries of `g_app_axis_cfg[]` and the `APP_AXIS*_ENCODER_I2C` macros. Swapped encoders are automatically detected during homing and reported via console (`[ENC] ... swap the encoder buses`).

---

## 2. Toolchain & Prerequisites

- **Arm GNU Toolchain** (`arm-none-eabi-gcc` 14.2+ or 15.3+)
- **CMake** (v3.22+)
- **Ninja** build system
- **Clang / LLVM-MinGW** (for running host simulation unit tests)
- **SEGGER J-Link** or ST-Link utility

---

## 3. Build & Flash

From this directory (`MotorControls/MotionFirmware/`):

### Target Cross-Compilation (STM32F446RE)
```powershell
# Using CMake presets:
cmake --preset Debug            # or Release
cmake --build --preset Debug    # Outputs to build/Debug/MotionFirmware.hex

# Or manual CMake invocation:
cmake -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
ninja -C build
```

### Flashing to Target MCU via J-Link
```powershell
.\flash.cmd
# Or:
.\flash_jlink.bat
```

### Host Simulation Unit Tests
Exhaustive host unit tests verifying the trapezoidal planner, StallGuard4 detector, encoder unwrap, TMC2240 math, console parser, and closed-loop homing/fault simulation:

```powershell
cmake -S tests -B build-host -G Ninja -DCMAKE_C_COMPILER=clang
cmake --build build-host
ctest --test-dir build-host --output-on-failure
```

---

## 4. Operation & User Interface

1. **Power Up:** Power the Nucleo and the 24V motor supply (order does not matter: drivers that do not respond are polled every 0.5 s; LD2 blinks at 1 Hz until detected).
2. **Start Cycle:** Press **Blue Button B1** (or type `start` in console): Z-axis homes first, then X-axis homes, followed by active cycling.
3. **Pause / Stop:** Press **B1** again to decelerate and hold position; press again to resume.
4. **Drivers Off / Emergency:** Long-press **B1** for 1.5 s to immediately disable both driver power stages (ENN high). Short press clears active faults.

### Onboard Status LED (LD2) Signals
| LD2 Pattern | Meaning |
| :--- | :--- |
| **1 Hz Blink** | Waiting for TMC2240 power/SPI (VM supply off or disconnected) |
| **Short flash every 2 s** | Standby / Idle |
| **4 Hz Blink** | Homing in progress |
| **Solid ON** | Cycling / Active motion |
| **0.5 Hz Blink** | Paused, homed, and holding position |
| **10 Hz Rapid Blink** | Fault state (check console for error code) |

---

## 5. Console & Telemetry Commands

Connect to `USART2` (`COM9` @ 115200) or open SEGGER RTT Terminal 0:

| Command | Action |
| :--- | :--- |
| `help` | List available commands |
| `status` | State, position, encoder readings, SG4 load, CS_ACTUAL, and temperatures |
| `start` / `stop` | Home if needed and cycle / decelerate and hold |
| `home` | Re-home all axes |
| `off` | Disable driver stages (ENN HIGH) |
| `clear` | Clear fault state; drivers are re-initialized automatically |
| `mode soft\|bounce` | Toggle between soft-limit step cycling and StallGuard4 hard-stop bouncing |
| `move <axis> <mm>` | Move a homed axis to an absolute position (e.g. `move z 42.5`) |
| `set <axis> <param> <val>` | Runtime tuning (e.g. `set x hspeed 80`). Typing `set` lists all tunable parameters |
| `get [axis]` | Display all active configuration parameters |
| `diag [axis]` | Dump TMC2240 registers, StallGuard load values, and encoder registers |
| `log 0\|1\|2` | Set telemetry level: 0 = events only, 1 = +status every 250 ms, 2 = +SG4 trace |

---

## 6. How It Works

### A. StallGuard4 Sensorless Homing (Z first, then X)
1. **Prep (Homing Profile):** StealthChop2 enabled (`TPWMTHRS=0`), reduced current with `IHOLD=IRUN`, CoolStep off, SG4 active above half-seek speed via `TCOOLTHRS`, and compensation filter enabled. Standstill for 250 ms for StealthChop auto-tuning (AT#1).
2. **Back-off:** Moves a few millimeters away from the first end (running start, AT#2).
3. **Seek 1:** Constant speed toward the first end (Z: MAX/up, X: MIN). Stall confirmed by: TMC2240 SPI status byte (`sg2`), absolute SG4 threshold, or SG4 drop below `sgratio × baseline`. AS5600 encoder step-loss detection acts as redundant backup.
4. **Retract & Seek 2:** Traverses full axis to the opposite end. Calibrates encoder ratio and records true travel distance.
5. **Coordinate Zeroing:** MIN end set to $0.0\ \text{mm}$; soft limits set to `[margin, travel - margin]`. Switches to RUN current profile.

### B. Cycle Modes
- **`soft` (Default):** Smooth trapezoidal moves between soft limits, strictly counted in microsteps without contacting hard stops. Continuous AS5600 following error check (2 fullsteps threshold) trips fault on lost steps.
- **`bounce`:** Repeatedly seeks into hard stops using StallGuard4. Logs deviation from homed travel, providing automated repeatability verification (Thesis Section 3.7.3).

---

## 7. Firmware Architecture & Source Modules

```text
MotionFirmware/
├── App/
│   ├── Inc/ & Src/
│   │   ├── motion_profile.c/.h  # Integer trapezoidal planner & step sequencer (Host tested)
│   │   ├── stepgen.c/.h         # TIM2/TIM5 update ISRs for step pulse generation
│   │   ├── stall_detect.c/.h    # StallGuard4 blanking, baseline EMA, and confirmation logic
│   │   ├── axis_ctrl.c/.h       # Homing, cycling, positioning, and fault state machines
│   │   ├── tmc_calc.c/.h        # Current scaling, register math, and chopper configs
│   │   ├── tmc_axis.c/.h        # TMC2240 register configuration and health diagnostics
│   │   ├── enc_track.c/.h       # AS5600 multi-turn angle unwrapping
│   │   ├── encoder.c/.h         # Dual I2C driver integration & bus recovery
│   │   ├── console.c/.h         # DMA UART & SEGGER RTT CLI console
│   │   ├── cmd_parse.c/.h       # Tokenizer & command line interpreter
│   │   └── app.c / app_config.c # Sequencing, B1 button debouncing, telemetry loop
│   └── tests/                   # Complete host CTest simulation harness
├── Core/                        # CubeMX initialization (clock, GPIO, SPI2, USART2)
└── cmake/                       # Arm toolchain definitions and CubeMX build scripts
```

---

## 8. Safety & Interlocks

- **Power-Stage Standby:** Driver `ENN` is asserted HIGH (outputs disabled) upon reset, in `Error_Handler()`, and during fault conditions.
- **Over-Travel Bounds:** All motion is constrained by `max_travel_mm` inside the step interrupt service routine; missed stalls trigger software timeouts before mechanical damage can occur.
- **Hot-Plugging Prohibition:** Never disconnect or reconnect stepper motors while the 24V supply is energized.
