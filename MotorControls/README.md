# MotorControls Subsystem

The **MotorControls** subsystem contains the firmware, modular embedded drivers, and kinematic execution code for the 2-Axis (XZ) Cartesian Manipulator.

---

## 1. Directory Structure

```text
MotorControls/
├── Drivers/
│   ├── AS5600/                     # MISRA C-oriented 12-bit magnetic rotary encoder driver
│   ├── TMC2240-Driver/             # Trinamic TMC2240 SPI stepper driver package
│   └── tmc2209_stm32_driver_v1.0.0/ # Legacy / reference TMC2209 driver
└── MotionFirmware/                  # STM32CubeMX + FreeRTOS application for STM32F446RE
```

---

## 2. Key Components

### A. Modular Drivers (`Drivers/`)
- **[`AS5600/`](Drivers/AS5600/README.md):**
  - High-precision 12-bit contactless magnetic rotary angle measurement over I2C.
  - Used for closed-loop gantry position feedback and lost-step detection.
  - Features MISRA C:2012-oriented coding, CMSIS-RTOS2 mutex port, and comprehensive host CTest suites.
- **[`TMC2240-Driver/`](Drivers/TMC2240-Driver/README.md):**
  - High-performance smart stepper driver operating over high-speed SPI.
  - Features StealthChop2 (silent operation), SpreadCycle, and StallGuard4 (sensorless mechanical homing and stall detection).
  - Out-of-the-box hardware abstraction layer for STM32 HAL.

### B. Motion Firmware (`MotionFirmware/`)
- Built for the **STM32F446RE Nucleo-64** board running FreeRTOS.
- Implements:
  - Multi-axis trapezoidal motion profile generation.
  - StallGuard4 sensorless homing on $Z$ and $X$ axes.
  - Ring-buffer UART parser for incoming Vision System target pick coordinates.
  - Dual telemetry stream over **USART2** (`COM9` @ 115200 baud) and **SEGGER RTT** (Terminal 0).

---

## 3. Hardware Interconnect & Pinouts

Refer to the complete master wiring specification:  
👉 **[`docs/hardware/wiring.md`](../docs/hardware/wiring.md)**

---

## 4. Getting Started & Building Firmware

Follow the setup and flashing guide in:  
👉 **[`MotionFirmware/README.md`](MotionFirmware/README.md)**
