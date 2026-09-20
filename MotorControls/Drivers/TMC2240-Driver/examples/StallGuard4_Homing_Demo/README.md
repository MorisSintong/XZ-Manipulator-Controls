# StallGuard4 Homing & Bounce Demo

This example demonstrates sensorless StallGuard4 homing and bouncing using the
STM32 HAL and TMC2240 driver. **The default build does not enable motion.**
Motor configuration and motion require explicit application settings.

Hardware qualification is pending. Software position counts commanded steps,
not measured motion; a StallGuard threshold crossing is not proof of a physical
endpoint. Provide an independent stop/inhibit appropriate to the mechanism.

## Features
- **Opt-in Homing**: Uses a software StallGuard load criterion; its effectiveness must be established on the actual motor and mechanism.
- **Span Estimate**: Counts commanded step pulses between software-detected endpoints.
- **Bounce Traverse**: Changes nominal pulse rate between travel and endpoint-search phases; this is not a qualified acceleration profile.
- **Travel and Time Bounds**: Each leg, including initial ramp-up, has enforced limits. A limit violation is a latched stop, never a fabricated endpoint.
- **Real-Time Telemetry**: Transmits status, step count, SG load value, temperature, and supply voltage over UART.

## Hardware Connections (STM32 Nucleo-F446RE to MKS TMC2240)

| STM32 Nucleo Pin | MKS TMC2240 Pin | Signal | Notes |
|:----------------:|:---------------:|:------:|:------|
| **PB10** | 14 | **SCK** | SPI2 Clock |
| **PC1** | 15 | **MOSI** (SDI) | SPI2 Master Out |
| **PC2** | 12 | **MISO** (SDO) | SPI2 Master In |
| **PA4** | 13 | **CS** | Chip Select (Active Low) |
| **PA0** | 10 | **STEP** | Step pulses |
| **PA1** | 9 | **DIR** | Direction control |
| Application-selected GPIO | 16 | **ENN** | Hold inactive-high during setup and on faults |
| — | 8 | **VMOT** | 12V – 36V Motor power |
| — | 2 | **VDD** | 3.3V Logic power |
| — | 1, 7 | **GND** | Common ground |

Verify the actual module pin numbering. ENN needs an external inactive-high
bias during MCU reset. Do not tie it to ground when using GPIO fault shutdown.
The chosen ENN GPIO must not overlap SPI, UART, STEP, DIR, CS, LED, or debug pins.

## UART Configuration
- **Baud Rate**: 115200 bps
- **Format**: 8 Data bits, No parity, 1 Stop bit (8N1)
- **Port**: USART2 (Connected to ST-LINK Virtual COM Port via PA2/PA3)

## Explicit Motion Configuration

Define `TMC2240_DEMO_ENABLE_MOTION=1` only after reviewing the wiring and limits.
An enabled build rejects missing settings at compile time:

| Required definition | Meaning |
|---|---|
| `TMC2240_DEMO_RUN_CURRENT_MA` | Requested run current matched to the motor and module |
| `TMC2240_DEMO_RREF_OHM` | The module's actual reference resistor, in ohms |
| `TMC2240_DEMO_MAX_STEPS` | Maximum pulses per leg and absolute software-position bound |
| `TMC2240_DEMO_TIMEOUT_MS` | Maximum elapsed time per leg, including telemetry/settling |
| `TMC2240_DEMO_EN_PORT` | GPIO port connected to ENN |
| `TMC2240_DEMO_EN_PIN` | A single unused GPIO pin connected to ENN |
| `TMC2240_DEMO_EN_CLOCK_ENABLE` | Its clock-enable macro name, without parentheses |
| `TMC2240_DEMO_ACK_STARTUP_FLAGS` | GSTAT flags the application deliberately permits acknowledging at startup |

For example, `GPIOB`, `GPIO_PIN_0`, and `__HAL_RCC_GPIOB_CLK_ENABLE` form one
consistent ENN configuration if PB0 is available and wired accordingly.
There is no universally safe default current or travel distance.

The datasheet's GSTAT reset word is `0x1D`, not just the reset-notification bit.
The compile-only fixture permits acknowledging those startup flags with
`TMC2240_DEMO_ACK_STARTUP_FLAGS=0x1D`; this excludes `DRV_ERR`. Select this policy
only after addressing the actual fault causes. A mask of zero acknowledges
nothing. Unselected or persistent flags abort startup, and no late/runtime
fault is automatically cleared and retried.

The demo holds ENN high before configuring the chip. Communication failures,
driver faults, or travel/time limit violations halt operation. GPIO shutdown
still depends on correct wiring and functioning hardware.

## Tuning StallGuard4
The busy-wait pulse rates are nominal, not measured guarantees. Validate STEP
timing, direction setup, mode thresholds, StallGuard behavior, current, and
temperature on the target before using motion. Configuration defines in
`main.c` under `USER CODE PD` include:

| Parameter | Default | Purpose |
|:---|:---:|:---|
| `TMC_HOME_SPEED_HZ` | 8000 | Nominal endpoint-search pulse rate; requires bench tuning |
| `TMC_TRAVEL_SPEED_HZ` | 24000 | High traverse speed between ends |
| `TMC_SLOW_ZONE_STEPS` | 2000 | Deceleration zone step count before expected ends |
| `TMC_SG_DROP_MARGIN` | 80 | Stall sensitivity: SG drop below running baseline to trigger stall |
| `TMC_SG_CONFIRM_COUNT` | 2 | Consecutive low SG readings required to confirm stall |
| `TMC_MAX_RAIL_STEPS` | Application-defined | Alias of `TMC2240_DEMO_MAX_STEPS`; exceeding it stops the demo |
