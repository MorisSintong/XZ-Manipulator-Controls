# MotorControls Subsystem

The **MotorControls** subsystem contains all embedded STM32H723 (`BIGTREETECH SKR Octopus Pro V1.1`) firmware, modular C drivers, host/UART emulation, and embedded verification suites for the 2-Axis (XZ) Cartesian Manipulator.

---

## 1. Directory Structure

```text
MotorControls/
├── Drivers/
│   ├── AS5600/                      # MISRA C-oriented 12-bit magnetic rotary encoder driver
│   ├── SEGGER_RTT/                  # SEGGER Real-Time Transfer telemetry library
│   ├── TMC2240-Driver/              # Trinamic TMC2240 SPI stepper driver package
│   └── tmc2209_stm32_driver_v1.0.0/ # Legacy / unused TMC2209 UART driver package
├── Emulation/                       # Python MockFirmwareServer + legacy Renode STM32F4 .resc bridge
├── MotionFirmware/                  # STM32CubeMX + CMake multi-profile H723 firmware suite
│   ├── Apps/                        # motion_diagnostics, stallguard_cli & z_bounce_emulation
│   └── tools/                       # Host Python bench CLI (diag/) & BTT bootloader binary (btt_bootloader/)
└── tests/
    └── emulation/                   # Automated STM32 UART protocol verification test suite
```

---

## 2. Key Components

### A. Modular Drivers (`Drivers/`)
- **[`AS5600/`](Drivers/AS5600/README.md):**
  - High-precision 12-bit contactless magnetic rotary angle measurement over I2C (`I2C1` & `I2C4` on H723).
  - Used for closed-loop gantry shaft feedback and step-tracking verification.
  - Features MISRA C:2012-oriented coding, optional CMSIS-RTOS2 mutex port, and host CTest suites.
- **[`TMC2240-Driver/`](Drivers/TMC2240-Driver/README.md):**
  - Smart stepper driver operating over high-speed SPI (`SPI1` Mode 3 at 3.125 MHz on SKR Octopus Pro V1.1).
  - Features StealthChop2, SpreadCycle, and StallGuard4 (sensorless mechanical homing and stall detection).
  - Shared `tmc2240_hal.h` supports both `stm32h7xx_hal.h` and `stm32f4xx_hal.h` (legacy standalone examples remain NUCLEO-F446RE).
- **[`SEGGER_RTT/`](Drivers/SEGGER_RTT/README.md):** Minimal RTT debug channel with fixed up/down buffers for J-Link host logs and optional polled input.

### B. Motion Firmware & Host Diagnostics (`MotionFirmware/`)
- Built for the **STM32H723ZET6 (BIGTREETECH SKR Octopus Pro V1.1)** from a shared CubeMX STM32H7 HAL base (`MotionFirmware.ioc`) with **three selectable application profiles** under [`MotionFirmware/Apps/`](MotionFirmware/Apps/):
  1. **[`motion_diagnostics/`](MotionFirmware/Apps/motion_diagnostics/README.md) *(default)*:** Cooperative foreground state machine with circular `USART2` (`PD5`/`PD6`) RX DMA, 1 MHz timer-compare stepping (`TIM2`/`TIM5`), Z-then-X StallGuard4 homing, binary-only `USART2` metrology records (`356/124/116` bytes), and RTT-only human logs.
  2. **[`stallguard_cli/`](MotionFirmware/Apps/stallguard_cli/README.md):** Interactive ASCII console (`USART2` + SEGGER RTT) with two-end StallGuard4 homing, travel/encoder calibration, and `soft`/`bounce` cycling modes.
  3. **[`z_bounce_emulation/`](MotionFirmware/Apps/z_bounce_emulation/README.md):** Single-axis Z StallGuard4 bounce demo and hybrid binary/ASCII `VisionUart` protocol responder (`MotionFirmware_ZBounceEmulation.*`).
- **[`MotionFirmware/tools/diag/`](MotionFirmware/tools/diag/README.md):** Host Python `motion-diag` CLI (built on shared [`packages/motion_diagnostics/`](../packages/motion_diagnostics/)) for capturing binary records, running scored positioning sequences, and generating metrology reports.

### C. Emulation & Verification (`Emulation/` & `tests/emulation/`)
- **[`Emulation/`](Emulation/README.md):** Host emulation runner (`renode_runner.py`), in-process `MockFirmwareServer`, and legacy `stm32f446_uart.resc` script for USART2 TCP bridge testing (note: Renode's built-in `stm32f4.repl` targets STM32F4, whereas compiled firmware targets STM32H723; use `--mode mock` for offline protocol checks).
- **[`tests/emulation/`](tests/emulation/run_tests.py):** 11-test automated verification suite covering binary CRC16 framing, fault injection, burst stress testing, legacy ASCII compatibility, and Vision System protocol symmetry.

---

## 3. Hardware Interconnect & Pinouts

Refer to the complete master wiring specification:  
👉 **[`docs/hardware/wiring.md`](../docs/hardware/wiring.md)**

---

## 4. Getting Started & Building Firmware

Follow the setup and flashing guide in:  
👉 **[`MotionFirmware/README.md`](MotionFirmware/README.md)**
