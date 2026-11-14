# SKR Octopus Pro V1.1 Wiring & Interconnect

- **Controller board:** BIGTREETECH SKR Octopus Pro V1.1, STM32H723ZET6
- **Stepper drivers:** BTT/MKS TMC2240 modules installed in the board's driver sockets (`MOTOR0` for X-axis, `MOTOR1` for Z-axis, `MOTOR2_1` for Rotation / $\theta$-axis)
- **Supply:** 24 V DC

> [!NOTE]
> `MotorControls/MotionFirmware/` (`MotionFirmware.ioc`, `Core/`, `Drivers/STM32H7xx_HAL_Driver/`, and all three application profiles) is configured directly for the **BIGTREETECH SKR Octopus Pro V1.1 (STM32H723ZET6)** pinout below.

## 1. Board and motor-driver assignment

Use the board's silkscreened driver sockets and matching motor output terminals. The assignments below follow the BTT Octopus Pro V1.1 schematic (`Hardware/BIGTREETECH-OCTOPUS-Pro/Hardware/BIGTREETECH Octopus Pro V1.1-sch.pdf`).

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

## 5. Host serial connection (`USART2` / planned USB-C CDC)

- **Current hardware interface (`USART2`):** The board provides a 2x5 Raspberry Pi `UART2` header (`J26`) wired to `PD5` (`USART2_TX`) and `PD6` (`USART2_RX`), with on-board 120 Ω series protection resistors (`R22`/`R23`). For a 3.3 V USB-UART adapter (or ESP32/host vision link at `115200 8N1`), cross TX/RX (`PD5` -> host RX, `PD6` <- host TX) and connect `GND`; leave the 5 V pins unconnected.
- **Native USB-C port (`PA11`/`PA12`):** `USB_OTG_HS` is not enabled in the current `MotionFirmware` builds (so the USB-C port provides `5V` logic power via `J68` only, with no COM port). See [Section 5 of `MotionFirmware/README.md`](../../MotorControls/MotionFirmware/README.md#5-planned-upgrade-native-usb-c-cdc-transport-via-tinyusb-usb_otg_hs) for the planned TinyUSB `USB_OTG_HS` CDC migration notes.

## 6. SWD debug probe connection (`J72`)

For flashing, GDB debugging, and high-speed SEGGER RTT telemetry via a SEGGER J-Link (or ST-LINK converted to J-Link) probe, connect to the 5-pin 2.54 mm single-row header **`J72`** (silkscreened **`SWD`**).

- **PCB Location & Orientation:** Located immediately below the bottom-right corner of the `STM32H723ZET6` package, between the `STATUS` LED (`D39`) and the white `BIGTREETECH Octopus Pro` silkscreen logo. Looking at the board with the motor driver sockets at the top, pins run left-to-right from **Pin 1** (marked by a **square solder pad** closest to `STATUS`/`D39`) to **Pin 5**.
- **Color Pinout Diagram Omission:** `J72` (`SWD`) is omitted from the colored callout diagram (`BIGTREETECH Octopus Pro V1.1-Pin.jpg`). Use `BIGTREETECH Octopus Pro - PIN.pdf` (`J72(SWD)`) and `BIGTREETECH Octopus Pro V1.1-sch.pdf` (Sheet 5) as the authoritative reference.

| Pin | Silkscreen / Net | Probe Signal | MCU Pin (`STM32H723ZET6`) | On-board Circuit & Sharing Notes |
| :---: | :---: | :--- | :--- | :--- |
| **1** | `3V3` (`VCC_3V3`) | `VTref` | `VDD` (3.3 V) | Target reference voltage sense for J-Link/ST-LINK (do **not** back-power the board from the probe) |
| **2** | `SWDIO` | `SWDIO` | Pin 105 (`PA13` / `JTMS-SWDIO`) | On-board 10 kΩ pull-up `R167` to `VCC_3V3`; shared with `WORK_LED` (`D39`) |
| **3** | `GND` | `GND` | `VSS` | Common debug ground |
| **4** | `SWCLK` | `SWCLK` | Pin 109 (`PA14` / `JTCK-SWCLK`) | On-board 10 kΩ pull-up `R168` to `VCC_3V3`; shared with `DRIVER7_DIR` |
| **5** | `RST` | `NRST` | Pin 25 (`NRST`) | Hardware reset (active low) |

- **Firmware Configuration:** `MotorControls/MotionFirmware/MotionFirmware.ioc` enables `DEBUG` in `Serial_Wire` mode (`PA13` -> `TMS`, `PA14` -> `TCK`), leaving both pins in their default `AF0` SWD function.
- **Pin Conflict Warnings:**
  - Do **not** install a stepper driver in socket **`MOTOR7` (`DRIVER7`)** when using SWD—`PA14` (`SWCLK`) also drives `DRIVER7_DIR` through the on-board buffer stage.
  - Do **not** drive `PA13` (`WORK_LED`) as a general-purpose LED output in firmware (use `LD2` on `PD15` instead).
  - `SWO` (`PB3`) is not broken out on `J72`; use **SEGGER RTT** over `SWDIO`/`SWCLK` with `probe-rs` or J-Link (`-device STM32H723ZE`).

### Using a NUCLEO-F446RE (`MB1136`) On-Board ST-LINK (Converted to J-Link OB)

If you use the ST-LINK/V2-1 section of a **NUCLEO-F446RE (`MB1136`)** reflashed with SEGGER J-Link OB firmware (`STLinkReflash`) to flash and debug the SKR Octopus Pro V1.1:

1. **Disconnect both `CN2` jumpers on the Nucleo (`ST-LINK` mode selection):**
   - Pull **both black jumpers off header `CN2`** (located beside the `CN4` `SWD` header on the ST-LINK half of the Nucleo).
   - Removing `CN2` disconnects the J-Link OB `SWCLK` and `SWDIO` lines from the Nucleo's on-board `STM32F446RE` and routes them out to **`CN4`** for an external target.
   - *(Tip: park the two removed jumpers sideways on single pins of `CN11`/`CN12` or store them safely so they can be restored if you ever debug the Nucleo again.)*
2. **Optional reset isolation (`SB12`) for `NRST`:**
   - Removing `CN2` isolates `SWDIO` and `SWCLK`, **not** `NRST`. Solder bridge **`SB12`** (on the bottom of the Nucleo PCB) still ties `CN4` Pin 5 (`NRST`) to the Nucleo's on-board `STM32F446RE`.
   - For standard flashing and RTT, **leave `CN4` Pin 5 (`NRST`) disconnected** (4-wire connection: `3V3`, `SWDIO`, `GND`, `SWCLK`) unless you have opened/desoldered `SB12`. Only wire `CN4` Pin 5 to `J72` Pin 5 (`RST`) when `SB12` is **OFF** (needed for connect-under-reset recovery).
3. **Power both boards properly:**
   - Plug the Nucleo's Mini-B USB port (**`CN1`**) into the PC—Windows enumerates it as a **J-Link** device.
   - Power the SKR Octopus Pro V1.1 separately so its `3.3 V` LED (`D51`) lights up and `J72` Pin 1 (`3V3`) is live (`CN4` Pin 1 `VDD_TARGET` only *senses* 3.3 V and does not power the Octopus Pro):
     - For **USB-only bench flashing** (no motors): install the **`J68` (`V_BUS`)** jumper and plug USB-C into the Octopus Pro.
     - For **24 V operation**: supply `MAIN POWER` (`24 V`) and remove **`J68` (`V_BUS`)** per BTT's manual so USB-C stays data-only.
4. **Wire `Nucleo CN4 (SWD)` to `SKR Octopus Pro J72 (SWD)`:**
   - On the Nucleo `CN4` 6-pin single-row header, **Pin 1** is the pin closest to the Mini-B USB connector `CN1` (marked with a dot on the silkscreen).
   - **Caution — NOT a 1-to-1 cable:** `SWDIO` and `SWCLK` are **swapped** between Pin 2 and Pin 4 on `CN4` vs. `J72`:

| Nucleo `CN4` Pin | Nucleo `CN4` Signal | Wire | SKR `J72` Pin | SKR `J72` Signal | Notes |
| :---: | :--- | :---: | :---: | :--- | :--- |
| **1** | `VDD_TARGET` | ──> | **1** | `3V3` | Target 3.3 V sense (`VTref`) — **required** for J-Link OB level sensing |
| **2** | `SWCLK` | ──> | **4** | `SWCLK` (`PA14`) | Crossed over (Pin 2 -> Pin 4) |
| **3** | `GND` | ──> | **3** | `GND` | Common ground reference |
| **4** | `SWDIO` | ──> | **2** | `SWDIO` (`PA13`) | Crossed over (Pin 4 -> Pin 2) |
| **5** | `NRST` *(optional)* | ──> | **5** | `RST` (`NRST`) | Connect **only** if `SB12` is opened/desoldered; otherwise leave disconnected |
| **6** | `SWO` | — | — | *(Not connected)* | Leave unconnected (`J72` has no `SWO` pin; use SEGGER RTT instead) |

5. **Flash and view RTT logs from `MotorControls\MotionFirmware`:**
   ```powershell
   # Run from MotorControls\MotionFirmware (flash.jlink uses relative build\Debug\ paths)
   cd MotorControls\MotionFirmware
   .\flash.cmd diagnostics   # Or omit arg for MotionFirmware.hex, or pass stallguard | zbounce

   # Or flash / run / stream RTT via probe-rs (works from repo root or firmware dir; recognizes J-Link OB)
   probe-rs download --chip STM32H723ZETx build\Debug\MotionFirmware_Diagnostics.elf
   probe-rs reset --chip STM32H723ZETx
   probe-rs attach --chip STM32H723ZETx build\Debug\MotionFirmware_Diagnostics.elf
   ```
   *(Never use ST-LINK tools such as `st-flash` or `openocd -f interface/stlink.cfg` while J-Link OB firmware is installed on the Nucleo probe.)*

## 7. Vacuum valve and auxiliary loads

Keep the 12/24 V vacuum valve on a suitably rated external MOSFET or relay driver with the required flyback protection and a safe default-off state. Do not connect it to a heater or fan output based only on connector similarity.

## 8. Board references

- [BIGTREETECH Octopus Pro repository](https://github.com/bigtreetech/BIGTREETECH-OCTOPUS-Pro) - V1.1 pinout, schematic, manual, power-selection notes, and revision differences.
- [Klipper Octopus Pro V1.1 pin configuration](https://github.com/Klipper3d/klipper/blob/master/config/generic-bigtreetech-octopus-pro-v1.1.cfg) - H723 motor, endstop, SPI, and expansion-header mappings.
- [Octopus Pro V1.1 H723 reference aliases](https://github.com/Frix-x/klippain/blob/c8b3197c6ea2f54bb20a9fff336d4d8fa21e6266/user_templates/mcu_defaults/main/BTT_Octopus_Pro_v1.1.cfg) - SPI driver alias reference.
