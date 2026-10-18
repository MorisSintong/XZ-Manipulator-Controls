# MotionFirmware – XZ Manipulator Motion Controller

STM32F446RE (Nucleo-64 `MB1136`) firmware that drives the two NEMA 17 axes of the 2-Axis (XZ) Cartesian manipulator through **BIGTREETECH (BTT) / MKS TMC2240 v1.0** drivers (SPI + STEP/DIR), reads dual **AS5600** magnetic encoder modules (I2C), and interfaces with a **5V 3-wire inductive proximity sensor** on the Morpho header (`PC8`).

The application features **StallGuard4** sensorless homing, travel measurement, and continuous soft/bounce cycle modes. The motion layer accepts absolute position targets in millimeters, coordinating pick-and-place trajectories commanded by the Vision System.

---

## 1. Hardware Mapping

Full wiring schematics, pinouts, and power distribution rules:  
👉 **[`docs/hardware/wiring.md`](../../docs/hardware/wiring.md)**

### Axis & Sensor Mapping Summary

| Axis / Sensor | Module | CS | STEP | DIR | ENN | Step Timer | Bus / Morpho Pin |
| :--- | :--- | :---: | :---: | :---: | :---: | :---: | :--- |
| **0 = Z** (T8x8 Lead Screw, 400 µsteps/mm) | BTT/MKS TMC2240 #0 | `PC0` (`CN7-38`) | `PC1` (`CN7-36`) | `PC2` (`CN7-35`) | `PC6` (`CN10-4`) | `TIM2` | `I2C3` (`PA8` SCL, `PC9` SDA) |
| **1 = X** (GT2 20T Belt, 80 µsteps/mm) | BTT/MKS TMC2240 #1 | `PC3` (`CN7-37`) | `PC4` (`CN10-34`) | `PC5` (`CN10-6`) | `PC7` (`CN10-19`) | `TIM5` | `I2C1` (`PB8` SCL, `PB9` SDA) |
| **Proximity Sensor** (5V 3-Wire Inductive) | NPN / PNP Sensor | — | — | — | — | — | `PC8` (`CN10-2` OUT), `+5V` (`CN7-18`), `GND` (`CN7-19`) |

- **Shared SPI2** (Mode 3, 2.625 MHz @ 84 MHz SYSCLK): `PB13` (`CN10-30` SCK), `PB14` (`CN10-28` MISO), `PB15` (`CN10-26` MOSI).
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

## 3. Build, Flash & Emulation Tests

From this directory (`MotorControls/MotionFirmware/`):

### Target Cross-Compilation (STM32F446RE)
```powershell
# Using CMake presets:
cmake --preset Debug            # or Release
cmake --build --preset Debug    # Outputs to build/Debug/MotionFirmware.hex

# Or manual CMake invocation:
cmake -B build/Debug -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake -DCMAKE_BUILD_TYPE=Debug
ninja -C build/Debug
```

### Flashing to Target MCU via J-Link
```powershell
.\flash.cmd
# Or:
.\flash_jlink.bat
```

### Automated UART Protocol & Renode Emulation Suite
From the repository root:
```powershell
uv run python MotorControls/tests/emulation/run_tests.py
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

## 7. Safety & Interlocks

- **Power-Stage Standby:** Driver `ENN` is asserted HIGH (outputs disabled) upon reset, in `Error_Handler()`, and during fault conditions.
- **Over-Travel Bounds:** All motion is constrained by `max_travel_mm`; missed stalls trigger software timeouts before mechanical damage can occur.
- **Hot-Plugging Prohibition:** Never disconnect or reconnect stepper motors while the 24V supply is energized.
