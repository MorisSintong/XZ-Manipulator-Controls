# SKR Octopus Pro V1.1 Wiring & Interconnect

- **Controller board:** BIGTREETECH SKR Octopus Pro V1.1, STM32H723ZET6
- **Stepper drivers:** BTT/MKS TMC2240 modules installed in the board's driver sockets (`MOTOR0` for X-axis, `MOTOR1` for Z-axis, `MOTOR2_1` for Rotation / $\theta$-axis)
- **Supply:** 24 V DC

> [!NOTE]
> `MotorControls/MotionFirmware/` (`MotionFirmware.ioc`, `Core/`, `Drivers/STM32H7xx_HAL_Driver/`, and all three application profiles) is configured directly for the **BIGTREETECH SKR Octopus Pro V1.1 (STM32H723ZET6)** pinout below.

## 1. Board and motor-driver assignment

Use the board's silkscreened driver sockets and matching motor output terminals. The assignments below follow the BTT Octopus Pro V1.1 schematic (`Hardware/BIGTREETECH-OCTOPUS-Pro/Hardware (V1.1)/BIGTREETECH Octopus Pro V1.1-sch.pdf`).

| Axis | Firmware Motor Index | Driver socket / motor output | STEP | DIR | Enable (active low) | SPI CS | DIAG / endstop |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| X | `motor=1` (`TMC_*1`, `TIM5`) | Driver 0 / `MOTOR0` | `PF13` | `PF12` | `PF14` | `PC4` | `PG6` (`STOP0`) |
| Z | `motor=0` (`TMC_*0`, `TIM2`) | Driver 1 / `MOTOR1` | `PG0` | `PG1` | `PF15` | `PD11` | `PG9` (`STOP1`) |
| Rotation ($\theta$) | `motor=2` (`TMC_*2`, open-loop) | Driver 2 / `MOTOR2_1` | `PF11` | `PG3` | `PG5` | `PC6` | `PG10` (`STOP2`) |

`!` in a Klipper pin definition means active-low (for example, `!PF14`). In `MotionFirmware`, `TMC_*0` (`motor=0`, `TIM2`) drives the **Z-axis** on socket `MOTOR1` (Driver 1), `TMC_*1` (`motor=1`, `TIM5`) drives the **X-axis** on socket `MOTOR0` (Driver 0), and `TMC_*2` (`motor=2`) is provisioned for the **Rotation ($\theta$) axis** on socket `MOTOR2_1` (Driver 2) with `TMC_CS2` (`PC6`) and `TMC_ENN2` (`PG5`) initialized HIGH (`GPIO_PIN_SET`) at boot and de-asserted in `Error_Handler` / `fault_safe_shutdown`.

The board routes driver SPI internally. `MotionFirmware` configures the shared **`SPI1`** bus (`PA5` SCK, `PA6` MISO, `PA7` MOSI, Mode 3, 3.125 MHz) and the per-socket chip-select pins in the table; do not run extra SPI wires between the sockets and expansion headers. Configure the installed TMC2240 modules for SPI mode via the under-driver jumpers.

With power disconnected, insert each module in the socket for its axis and align its pin-1/orientation marking with the board and module documentation. Connect each motor to the matching board output (`MOTOR0` for X, `MOTOR1` for Z, optional `MOTOR2` for theta). Never insert or remove a driver or motor connector while the board is powered.

## 2. Power distribution

```text
24 V supply (+) ──> SKR MAIN POWER (+)
                 └─> external loads only through separately rated protection
24 V supply (-) ──> SKR MAIN POWER (-) ──> common system ground

SKR MOTOR0 ──> X-axis NEMA 17
SKR MOTOR1 ──> Z-axis NEMA 17
SKR MOTOR2 ──> optional theta motor
```

- Feed the board from its `MAIN POWER` input. BTT documents `MAIN POWER` for up to 28 V; 24 V is within that limit.
- Set the motor-supply selector for every used driver socket to `MAIN POWER`. Do not select the separate `MOTOR_POWER` rail for this 24 V setup unless its supply and the installed driver ratings have been checked. The board's higher-voltage option is not a reason to exceed the TMC2240 module rating.
- Keep the 24 V return, board ground, sensor ground, and any external actuator-supply return at a common reference. Fuse and size wiring for the actual supply and load currents.
- Do not power a driver from both board and external supplies. With the drivers seated in the SKR sockets, the board supplies the driver modules and motor outputs.
- Power AS5600 logic from a regulated 3.3 V rail (such as the `3.3V` pin on the `J73` `I2C` header). Do **not** connect 5 V (such as the `5V` pin on `J43` `BL_Touch`) to an AS5600 module configured for 3.3 V operation.

## 3. AS5600 encoder connections

The AS5600 modules share the fixed I2C address `0x36`, so X and Z are connected to two independent hardware I2C peripherals on the STM32H723 (`I2C1` and `I2C4`):

| Encoder | Signal | Board header | MCU pin | Notes |
| :--- | :--- | :--- | :--- | :--- |
| X (`AXIS_X`) | SCL | `J73` (`I2C` header) | `PB8` | Hardware `I2C1_SCL` (AF4, on-board 4.7 kΩ pull-up `R164` to 3.3V) |
| X (`AXIS_X`) | SDA | `J73` (`I2C` header) | `PB9` | Hardware `I2C1_SDA` (AF4, on-board 4.7 kΩ pull-up `R165` to 3.3V) |
| Z (`AXIS_Z`) | SCL | `J43` (`BL_Touch` header) | `PB6` | Hardware `I2C4_SCL` (AF6, direct MCU pin 136; uses AS5600 breakout 3.3V pull-ups) |
| Z (`AXIS_Z`) | SDA | `J43` (`BL_Touch` header) | `PB7` | Hardware `I2C4_SDA` (AF6, direct MCU pin 137; uses AS5600 breakout 3.3V pull-ups) |
| Both | 3.3 V & GND | `J73` (`I2C` header) | `3.3V` / `GND` | Power both AS5600 breakouts from `J73` `3.3V` and `GND` (never `J43` `5V`) |

Provide a regulated 3.3 V supply to each module's `VCC` and `3V3` as required by that breakout's `R1` configuration. If `R1` ties `VCC` to `3V3`, power that rail with 3.3 V only; if `R1` is open, connect both module supply pins to 3.3 V for this setup. Keep the I2C pull-ups at 3.3 V. Confirm the installed breakout's `R1` and wiring against its own schematic before powering it.

## 4. Endstops, E-Stop button, and status LED

- The board's X (`Driver 0`) and Z (`Driver 1`) endstop inputs are `PG6` (`STOP0`) and `PG9` (`STOP1`), respectively. These are also the corresponding driver `DIAG` signals. Configure sensorless homing or mechanical endstops deliberately; do not connect both to the same input without checking the board jumper arrangement.
- **E-Stop / User Button (`B1`)**: Mapped to `PG15` (`STOP7` header `J34`). `STOP7` has an on-board 10 kΩ pull-up (`R28`) to 3.3 V and a 100 Ω / 0.1 µF hardware RC filter (`R32`/`C16`), so it idles safely HIGH (`GPIO_PIN_SET`) even with nothing plugged in and triggers `EXTI15_10_IRQn` when shorted to `GND`.
- **Status / Heartbeat LED (`LD2`)**: Mapped to `PD15` (`FAN5` gate indicator LED `D27`). Note that the board's `PA13` (`WORK_LED`) is intentionally left on `JTMS-SWDIO` so SEGGER J-Link SWD debugging and RTT telemetry are never interrupted.
- The dedicated `PROBE` connector uses MCU input `PC5`. BTT documents NPN and PNP proximity-switch support on the Octopus Pro (**note:** `PC5` is not yet configured or read in `MotionFirmware`).

## 5. Host serial connection (`USART2`)

The board provides a 2x5 Raspberry Pi `UART2` header (`J26`) wired to `PD5` (`USART2_TX`) and `PD6` (`USART2_RX`), with on-board 120 Ω series protection resistors (`R22`/`R23`). For a 3.3 V USB-UART adapter (or ESP32/host vision link at `115200 8N1`), cross TX/RX (`PD5` -> host RX, `PD6` <- host TX) and connect `GND`; leave the 5 V pins unconnected.

## 6. Vacuum valve and auxiliary loads

Keep the 12/24 V vacuum valve on a suitably rated external MOSFET or relay driver with the required flyback protection and a safe default-off state. Do not connect it to a heater or fan output based only on connector similarity.

## 7. Board references

- [BIGTREETECH Octopus Pro repository](https://github.com/bigtreetech/BIGTREETECH-OCTOPUS-Pro) - V1.1 pinout, schematic, manual, power-selection notes, and revision differences.
- [Klipper Octopus Pro V1.1 pin configuration](https://github.com/Klipper3d/klipper/blob/master/config/generic-bigtreetech-octopus-pro-v1.1.cfg) - H723 motor, endstop, SPI, and expansion-header mappings.
- [Octopus Pro V1.1 H723 reference aliases](https://github.com/Frix-x/klippain/blob/c8b3197c6ea2f54bb20a9fff336d4d8fa21e6266/user_templates/mcu_defaults/main/BTT_Octopus_Pro_v1.1.cfg) - SPI driver alias reference.
