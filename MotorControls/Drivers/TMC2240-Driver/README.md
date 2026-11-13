# TMC2240 Stepper Motor Driver & Example Program for STM32

Driver and example package for the **Analog Devices / Trinamic TMC2240**.

**Current physical controller:** BIGTREETECH SKR Octopus Pro V1.1 with STM32H723ZET6 and TMC2240 modules; see the [current board wiring](../../../docs/hardware/wiring.md).

**Software compatibility:** The reusable register core is portable, but the bundled STM32 HAL adapter, CubeIDE project, and firmware images target the legacy NUCLEO-F446RE / STM32F446RE. They are not an H723 port and must not be used as firmware on the SKR. The H723 port remains pending.

Host simulations do **not** establish electrical, motor, timing, thermal, or mechanical readiness.

**Release readiness must be established from the validation gates, not the
historical claims in this package.** No hardware qualification was performed
in the current software-only audit.

This package contains a reusable TMC2240 register core, an STM32F4 HAL adapter, legacy F446 demonstrations/projects, and F446 diagnostic images. It does not include SKR/H723 firmware.

---

## 📦 Package Structure

```
TMC2240_STM32_Driver_Package/
├── README.md                           # Main package guide & hardware reference
│
├── driver/                             # Standalone, reusable TMC2240 C driver
│   ├── README.md                       # Driver integration instructions for any project
│   ├── inc/                            # Complete C headers for include paths (-I driver/inc)
│   │   ├── tmc2240.h                   # Master umbrella header
│   │   ├── tmc2240_hal.h               # STM32 HAL SPI & GPIO platform abstraction
│   │   ├── tmc2240_core.h              # Register access, pipelining, and cache
│   │   ├── TMC2240_HW_Abstraction.h    # Register map & bitfields (verified against Rev 2)
│   │   ├── API_Header.h                # Trinamic API core header
│   │   ├── Bits.h                      # Bit manipulation macros
│   │   ├── CRC.h                       # CRC utility definitions
│   │   ├── Config.h                    # Driver configuration defines
│   │   ├── Constants.h                 # Driver constants
│   │   ├── Functions.h                 # Helper function prototypes
│   │   ├── Macros.h                    # Register packing/unpacking macros
│   │   ├── RegisterAccess.h            # Read/Write access flags
│   │   ├── Types.h                     # Trinamic standard typedefs
│   │   └── helpers/                    # Forwarding headers for legacy include compatibility
│   └── src/                            # C source implementation files
│       ├── tmc2240_hal.c               # STM32 HAL SPI transport & hardware configuration
│       ├── TMC2240.c                   # Core register read/write & shadow cache
│       ├── CRC.c                       # CRC calculation functions
│       └── Functions.c                 # Utility helper functions; compile once
│
├── examples/
│   ├── StallGuard4_Homing_Demo/        # Legacy STM32F446RE demo code & documentation
│   │   ├── README.md                   # State machine explanation, UART logs, tuning
│   │   ├── main.c                      # Complete demonstration application
│   │   └── main.h                      # Pin definitions & hardware configurations
│   │
│   └── STM32CubeIDE_Project/           # Legacy STM32F446RE CubeIDE project
│       ├── README.md                   # Step-by-step import and build guide
│       ├── .cproject                   # Eclipse CDT project configuration
│       ├── .project                    # Eclipse project descriptor
│       ├── .mxproject                  # STM32CubeMX configuration
│       ├── Test-TMC2240.ioc            # STM32CubeMX device model
│       ├── STM32F446RETX_FLASH.ld      # Linker script for Flash execution
│       ├── STM32F446RETX_RAM.ld        # Linker script for RAM execution
│       ├── Core/                       # Application & driver source code
│       └── Drivers/                    # STMicroelectronics HAL & CMSIS libraries
│
├── firmware/                           # Legacy diagnostic images for NUCLEO-F446RE only
│   ├── README.md                       # Flashing instructions
│   ├── Test-TMC2240.bin                # Raw binary image (direct drag-to-drive)
│   ├── Test-TMC2240.hex                # Intel HEX image (STM32CubeProgrammer)
│   └── Test-TMC2240.elf                # ELF binary with full debug symbols
│
└── docs/                               # Technical documentation
    ├── README.md                       # Documentation index (vendor PDF datasheet is not bundled)
    ├── API_Reference.md                # Comprehensive API and register documentation
    ├── Hardware_Guide.md               # Pinouts, motor wiring, and power supply notes
    ├── Audit_and_Fix_Log.md            # Hardware register verification & audit history
    └── Dual_Motor_and_Encoder_Config.md# Dual motor + dual AS5600 encoder pinout guide
```

---

## ⚡ Quick Start

### Option 1: Pre-Compiled Diagnostic Firmware (legacy F446 only)
These regenerated diagnostics-only images target the legacy F446 firmware. They are build-verified but hardware-unverified; **do not flash them to the SKR/H723**. Read `firmware\README.md` before any legacy F446 hardware test.

### Option 2: Legacy STM32CubeIDE Project (F446 only)
1. In STM32CubeIDE, click **File** -> **Import...** -> **Existing Projects into Workspace**.
2. Select the `examples/STM32CubeIDE_Project` folder.
3. Build both Debug and Release; do not rely on old build-result claims.
4. Review the diagnostics-only default and explicit motion opt-in configuration
   in `examples\StallGuard4_Homing_Demo\README.md`. Do not automatically flash.

This project targets NUCLEO-F446RE and is not compatible with the current SKR/H723.

### Option 3: Integrate Driver Into an Existing Project
1. Copy `driver/inc/` and `driver/src/` into your project directory.
2. Add `driver/inc/` to your compiler include directories.
3. Add the `.c` files in `driver/src/` to your build sources.
4. Include `tmc2240_core.h` for the portable core. `tmc2240.h` is the STM32F4 HAL umbrella; using this driver on the SKR/H723 requires an H723-specific platform adapter.

---

## Legacy Standalone Example Pinout (NUCLEO-F446RE to Single MKS TMC2240)

> [!IMPORTANT]
> The table below applies **only** to the legacy single-motor `Test-TMC2240` example. The current XZ gantry uses the SKR Octopus Pro V1.1 / STM32H723ZET6; use the [current SKR wiring guide](../../../docs/hardware/wiring.md). The F446 pinout and firmware are not applicable to that board.

| STM32 Nucleo Pin | MKS TMC2240 Pin | Signal | Description |
|:----------------:|:---------------:|:------:|:------------|
| **PB10** | 14 | **SCK** | SPI2 Clock |
| **PC1** | 15 | **MOSI** (SDI) | SPI2 Master Out |
| **PC2** | 12 | **MISO** (SDO) | SPI2 Master In |
| **PA4** | 13 | **CS** | Chip Select (Active Low) |
| **PA0** | 10 | **STEP** | Step pulse input |
| **PA1** | 9 | **DIR** | Direction control input |
| Application-selected GPIO | 16 | **ENN** | Hold inactive-high during setup and on a fault |
| — | 8 | **VMOT** | Motor power (12V – 36V DC) |
| — | 2 | **VDD** | Logic power (3.3V from Nucleo) |
| — | 1, 7 | **GND** | System ground |

---

## Driver Features

- **STEP/DIR Control**: Requires explicit application configuration and activation.
- **Pipelined SPI Read/Write**: Accurate 40-bit frame communication (Mode 3, CPOL=1, CPHA=1).
- **Current Control**: Integrated current sensing via `GLOBAL_SCALER`, `IHOLD_IRUN`, and `rRef`/`KIFS` calculations.
- **Microstepping**: Register configuration for the hardware microstep sequencer.
- **StealthChop2 & SpreadCycle**: Dynamically switchable via velocity threshold (`TPWMTHRS`).
- **StallGuard4 Sensorless Detection**: Real-time mechanical load monitoring with baseline EMA tracking.
- **CoolStep**: Automatic load-adaptive current scaling.
- **Diagnostics**: Built-in temperature (°C), supply voltage (mV), and over-current fault detection.
- **DIAG Pin Routing**: Hardware routing for stall, error, and index signals.
