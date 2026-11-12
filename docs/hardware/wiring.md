# SKR Octopus Pro V1.1 Wiring & Interconnect

- **Controller board:** BIGTREETECH SKR Octopus Pro V1.1, STM32H723
- **Stepper drivers:** BTT/MKS TMC2240 modules installed in the board's driver sockets
- **Supply:** 24 V DC

> [!IMPORTANT]
> This is the intended physical-board wiring. The checked-in `MotionFirmware` still targets a NUCLEO-F446RE and has not been ported to the STM32H723. Its GPIO, SPI, I2C, timer, and UART configuration cannot be used on this board as-is. Do not energize motors or actuators until the H723 firmware port and hardware checks are complete.

## 1. Board and motor-driver assignment

Use the board's silkscreened driver sockets and matching motor output terminals. The assignments below follow the BTT/Klipper Octopus Pro V1.1 pin map.

| Axis | Driver socket / motor output | STEP | DIR | Enable (active low) | SPI CS | DIAG / endstop |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| X | Driver 0 / MOTOR0 | `PF13` | `PF12` | `PF14` | `PC4` | `PG6` |
| Z | Driver 2 / MOTOR2 | `PF11` | `PG3` | `PG5` | `PC6` | `PG10` |
| Optional theta | Driver 3 / MOTOR3 (E0) | `PG4` | `PC1` | `PA2` | `PC7` | `PG11` |

`!` in a firmware pin definition means active-low (for example, `!PF14`). Driver 1 / MOTOR1 is the board's Y channel and is unused by the current XZ design. The optional theta channel is not actuated by the current firmware profile.

The board routes driver SPI internally. Configure the H723 firmware for the board's shared SPI1 bus (`PA5` SCK, `PA6` MISO, `PA7` MOSI) and the per-socket chip-select pins in the table; do not run extra SPI wires between the sockets and expansion headers. Configure the installed TMC2240 modules for SPI.

With power disconnected, insert each module in the socket for its axis and align its pin-1/orientation marking with the board and module documentation. Connect each motor to the matching board output (`MOTOR0` for X, `MOTOR2` for Z, optional `MOTOR3` for theta). Never insert or remove a driver or motor connector while the board is powered.

## 2. Power distribution

```text
24 V supply (+) ──> SKR MAIN POWER (+)
                 └─> external loads only through separately rated protection
24 V supply (-) ──> SKR MAIN POWER (-) ──> common system ground

SKR MOTOR0 ──> X-axis NEMA 17
SKR MOTOR2 ──> Z-axis NEMA 17
SKR MOTOR3 ──> optional theta motor
```

- Feed the board from its `MAIN POWER` input. BTT documents `MAIN POWER` for up to 28 V; 24 V is within that limit.
- Set the motor-supply selector for every used driver socket to `MAIN POWER`. Do not select the separate `MOTOR_POWER` rail for this 24 V setup unless its supply and the installed driver ratings have been checked. The board's higher-voltage option is not a reason to exceed the TMC2240 module rating.
- Keep the 24 V return, board ground, sensor ground, and any external actuator-supply return at a common reference. Fuse and size wiring for the actual supply and load currents.
- Do not power a driver from both board and external supplies. With the drivers seated in the SKR sockets, the board supplies the driver modules and motor outputs.
- Power AS5600 logic from a regulated 3.3 V rail. The expansion headers expose 5 V, not a documented 3.3 V supply pin; do not connect 5 V to an AS5600 module configured for 3.3 V operation.

## 3. AS5600 encoder connections

The AS5600 modules share the fixed I2C address `0x36`, so keep X and Z on independent buses. The EXP header aliases and pin numbers below follow the Octopus Pro V1.1 board definition. The proposed second bus requires GPIO/software-I2C support in the H723 firmware.

| Encoder | Signal | Board connector pin | MCU pin | Notes |
| :--- | :--- | :--- | :--- | :--- |
| X | SCL | EXP1-1 | `PE8` | I2C4 SCL |
| X | SDA | EXP1-3 | `PE9` | I2C4 SDA |
| Z | SCL | EXP1-2 | `PE7` | Configure as software-I2C GPIO |
| Z | SDA | EXP1-4 | `PE10` | Configure as software-I2C GPIO |
| Both | GND | EXP1-9 | GND | Common ground |

Provide a regulated 3.3 V supply to each module's `VCC` and `3V3` as required by that breakout's `R1` configuration. If `R1` ties `VCC` to `3V3`, power that rail with 3.3 V only; if `R1` is open, connect both module supply pins to 3.3 V for this setup. Keep the I2C pull-ups at 3.3 V. Do not use EXP1-10 (+5 V) for these modules. Confirm the installed breakout's `R1` and wiring against its own schematic before powering it.

## 4. Endstops and proximity sensor

- The board's X and Z endstop inputs are `PG6` and `PG10`, respectively. These are also the corresponding driver `DIAG` signals. Configure sensorless homing or mechanical endstops deliberately; do not connect both to the same input without checking the board jumper arrangement.
- The dedicated `PROBE` connector uses MCU input `PC5`. BTT documents NPN and PNP proximity-switch support on the Octopus Pro. Use the connector's documented supply, ground, and signal terminals for the selected sensor type.
- Do not connect a sensor's 5 V push-pull output directly to an H723 expansion-header GPIO. Use the board's probe input circuit or a suitable level-shifting / isolation interface and verify the sensor output type first.

## 5. Host serial connection

The board provides a USB-C connection and a five-position Raspberry Pi UART header. BTT documents the header order as `5V, 5V, GND, RX, TX` (top to bottom). For a 3.3 V USB-UART adapter, cross TX/RX and connect ground; leave both 5 V pins unconnected.

Select USB CDC or the header UART in the H723 firmware and host configuration. The existing firmware uses NUCLEO-F446RE `USART2` and its ST-LINK virtual COM port; neither that pinout nor the Nucleo's `COM9` connection transfers to the SKR.

## 6. Vacuum valve and auxiliary loads

Keep the 12/24 V vacuum valve on a suitably rated external MOSFET or relay driver with the required flyback protection and a safe default-off state. Do not connect it to a heater or fan output based only on connector similarity. No SKR output is assigned to the valve until the H723 firmware and interlocks are designed and tested.

## 7. Board references

- [BIGTREETECH Octopus Pro repository](https://github.com/bigtreetech/BIGTREETECH-OCTOPUS-Pro) - V1.1 pinout, schematic, manual, power-selection notes, and revision differences.
- [Klipper Octopus Pro V1.1 pin configuration](https://github.com/Klipper3d/klipper/blob/master/config/generic-bigtreetech-octopus-pro-v1.1.cfg) - H723 motor, endstop, SPI, and expansion-header mappings.
- [Octopus Pro V1.1 H723 reference aliases](https://github.com/Frix-x/klippain/blob/c8b3197c6ea2f54bb20a9fff336d4d8fa21e6266/user_templates/mcu_defaults/main/BTT_Octopus_Pro_v1.1.cfg) - SPI driver alias reference.
