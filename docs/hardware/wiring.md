# Complete Wiring & Hardware Interconnect Specification
## STM32F446RE Nucleo-64 ➔ Dual BTT/MKS TMC2240 Modules, Dual AS5600 Modules & 5V Inductive Proximity Sensor

**Project:** XZ-Manipulator-Controls  
**Target Board:** STM32F446RE Nucleo-64 (`MB1136`)  
**Actuators:** 2× NEMA 17 Stepper Motors driven by **BIGTREETECH (BTT) / MKS TMC2240 v1.0** StepStick Modules (SPI Mode, $R_{\text{REF}} = 12\,\text{k}\Omega$)  
**Sensors:** 2× **AS5600 Breakout Modules** (12-bit Magnetic Absolute Rotary Encoders, Dual I2C), 1× **5V 3-Wire Inductive Proximity Sensor** (Morpho GPIO)  
**Telemetry:** Dual Broadcast on USART2 (115200 baud, ST-LINK VCOM `COM9`) and SEGGER RTT Terminal 0  

---

## 1. System Architecture Diagram

```text
                             +-----------------------------------+
                             |     STM32F446RE Nucleo-64         |
                             +-----------------------------------+
                                    |          |          |
                 [SPI2 Bus]         |          |          |         [I2C Buses]
             PB13: SCK              |          |          |      I2C1 (PB8/PB9): Enc 1 (X)
             PB14: MISO             |          |          |      I2C3 (PA8/PC9): Enc 2 (Z)
             PB15: MOSI             |          |          |
                  +-----------------+          |          +-----------------+
                  |                            | [Morpho GPIO]              |
                  |                            | PC8 (CN10 Pin 2)           |
                  v                            v                            v
         +------------------+   +------------------+       +------------------+   +------------------+
         | BTT/MKS TMC2240  |   | BTT/MKS TMC2240  |       | AS5600 Module 1  |   | AS5600 Module 2  |
         | (Z-Axis/Motor 0) |   | (X-Axis/Motor 1) |       | (X-Axis)         |   | (Z-Axis)         |
         |  CS:   PC0       |   |  CS:   PC3       |       | Addr: 0x36       |   | Addr: 0x36       |
         |  STEP: PC1       |   |  STEP: PC4       |       | Bus:  I2C1       |   | Bus:  I2C3       |
         |  DIR:  PC2       |   |  DIR:  PC5       |       +------------------+   +------------------+
         |  ENN:  PC6       |   |  ENN:  PC7       |
         +------------------+   +------------------+                +--------------------------+
                  |                      |                          | 5V 3-Wire Inductive      |
                  v                      v                          | Proximity Sensor         |
            [NEMA 17 (Z)]          [NEMA 17 (X)]                    | BN (+V)  : +5V (CN7-18)  |
                                                                    | BU (GND) : GND (CN7-19)  |
                                                                    | BK (OUT) : PC8 (CN10-2)  |
                                                                    +--------------------------+
```

---

## 2. Master Pinout Table (STM32 Nucleo-64 `MB1136`)

Both **Arduino Header Pin** (female headers `CN5`/`CN6`/`CN8`/`CN9`) and **ST Morpho Pin** (double-row male headers `CN7`/`CN10`, verified against `UM1724` Table 29) are specified below:

| STM32 Pin | Arduino / Morpho Header | Peripheral Function | Connected Module | Module Pin / Net | Description & Notes |
| :--- | :--- | :---: | :--- | :--- | :--- |
| **PB13** | Morpho **CN10 Pin 30** | `SPI2_SCK` (AF5) | **Both TMC2240 Modules** | Pin 14 (`CLK` / `SCK`) | Shared SPI Clock (2.625 MHz @ 84 MHz SYSCLK, SPI Mode 3) |
| **PB14** | Morpho **CN10 Pin 28** | `SPI2_MISO` (AF5) | **Both TMC2240 Modules** | Pin 12 (`MISO` / `SDO`) | Shared SPI Data Input to MCU (internal pull-up enabled) |
| **PB15** | Morpho **CN10 Pin 26** | `SPI2_MOSI` (AF5) | **Both TMC2240 Modules** | Pin 15 (`MOSI` / `SDI`) | Shared SPI Data Output from MCU |
| **PC0** | Arduino **A5** (`CN8-6`) / Morpho **CN7 Pin 38** | `GPIO_Output` | **TMC2240 (Z / Motor 0)** | Pin 13 (`CS`) | Chip Select 0 (Active LOW, pull-up). Requires `SB51` **ON**, `SB52` **OFF** |
| **PC1** | Arduino **A4** (`CN8-5`) / Morpho **CN7 Pin 36** | `GPIO_Output` | **TMC2240 (Z / Motor 0)** | Pin 10 (`STEP`) | Step pulse output for Z-axis. Requires `SB56` **ON**, `SB46` **OFF** |
| **PC2** | Morpho **CN7 Pin 35** *(odd row)* | `GPIO_Output` | **TMC2240 (Z / Motor 0)** | Pin 9 (`DIR`) | Direction control for Z-axis *(Note: `CN7 Pin 34` is `PB0`, NOT `PC2`!)* |
| **PC6** | Morpho **CN10 Pin 4** | `GPIO_Output` | **TMC2240 (Z / Motor 0)** | Pin 16 (`EN` / `ENN`) | Enable 0 (Active LOW; HIGH = disabled) |
| **PC3** | Morpho **CN7 Pin 37** *(odd row)* | `GPIO_Output` | **TMC2240 (X / Motor 1)** | Pin 13 (`CS`) | Chip Select 1 (Active LOW, pull-up) |
| **PC4** | Morpho **CN10 Pin 34** | `GPIO_Output` | **TMC2240 (X / Motor 1)** | Pin 10 (`STEP`) | Step pulse output for X-axis |
| **PC5** | Morpho **CN10 Pin 6** | `GPIO_Output` | **TMC2240 (X / Motor 1)** | Pin 9 (`DIR`) | Direction control for X-axis |
| **PC7** | Arduino **D9** (`CN5-2`) / Morpho **CN10 Pin 19** | `GPIO_Output` | **TMC2240 (X / Motor 1)** | Pin 16 (`EN` / `ENN`) | Enable 1 (Active LOW; HIGH = disabled) |
| **PB8** | Arduino **D15** (`CN5-10`) / Morpho **CN10 Pin 3** | `I2C1_SCL` (AF4) | **AS5600 Module 1 (X-Axis)** | `SCL` | I2C1 Clock (100–400 kHz, onboard 4.7k pull-up on module) |
| **PB9** | Arduino **D14** (`CN5-9`) / Morpho **CN10 Pin 5** | `I2C1_SDA` (AF4) | **AS5600 Module 1 (X-Axis)** | `SDA` | I2C1 Data (100–400 kHz, onboard 4.7k pull-up on module) |
| **PA8** | Arduino **D7** (`CN9-8`) / Morpho **CN10 Pin 23** | `I2C3_SCL` (AF4) | **AS5600 Module 2 (Z-Axis)** | `SCL` | I2C3 Clock (100–400 kHz, onboard 4.7k pull-up on module) |
| **PC9** | Morpho **CN10 Pin 1** | `I2C3_SDA` (AF4) | **AS5600 Module 2 (Z-Axis)** | `SDA` | I2C3 Data (100–400 kHz, onboard 4.7k pull-up on module) |
| **PC8** | Morpho **CN10 Pin 2** | `GPIO_Input` / `GPIO_EXTI8` | **5V Proximity Sensor** | `BK` (Black / `OUT`) | 3-wire 5V proximity sensor signal (5V-tolerant `FT` pin) |
| **PC13** | Onboard **Blue Button B1** / Morpho **CN7 Pin 23** | `GPIO_EXTI13` / Input | **User Interface** | — | Active LOW (`SB17` ON): start / stop motion cycle |
| **PA5** | Arduino **D13** (`CN5-6`) / Morpho **CN10 Pin 11** | `GPIO_Output` | **Status LED (`LD2`)** | — | Green user LED (`SB21` ON): ON = cycling, fast blink = fault |
| **PA2** | Internal **ST-LINK VCOM** *(CN10 Pin 35 if `SB62` ON)* | `USART2_TX` (AF7) | **CDC Serial / COM9** | ST-LINK RX | Routed to USB VCOM via `SB13` ON (`SB62` OFF by default) |
| **PA3** | Internal **ST-LINK VCOM** *(CN10 Pin 37 if `SB63` ON)* | `USART2_RX` (AF7) | **CDC Serial / COM9** | ST-LINK TX | Routed to USB VCOM via `SB14` ON (`SB63` OFF by default) |
| **PA13** | Morpho **CN7 Pin 13** (`SWDIO`) | `SYS_JTMS-SWDIO` | **ST-Link / J-Link OB** | `SWDIO` | Debug & SEGGER RTT channel — **do not use as GPIO** |
| **PA14** | Morpho **CN7 Pin 15** (`SWCLK`) | `SYS_JTCK-SWCLK` | **ST-Link / J-Link OB** | `SWCLK` | Debug Clock — **do not use as GPIO** |
| **PB3** | Arduino **D3** (`CN9-4`) / Morpho **CN10 Pin 31** | `SYS_JTDO-SWO` | **ST-Link Trace (`SWO`)** | `SWO` | Reserved for SWO trace via `SB15` ON (`MotionFirmware.ioc`) |
| **+5V** | Morpho **CN7 Pin 18** / Arduino **5V** (`CN6 Pin 5`) | 5V Supply (+5.0V) | **5V Proximity Sensor** | `BN` (Brown / `+V`) | 5V rail (max 300 mA total when USB-powered; or 5V buck) |
| **+3V3** | Morpho **CN7 Pin 16** / Arduino **3V3** (`CN6 Pin 4`) | Logic Supply (+3.3V) | **All Drivers & Encoders**| `VIO` / `VCC` | 3.3V from Nucleo `LD39050PU33R` LDO (max 500 mA) |
| **GND** | Morpho **CN7 Pin 19, 20** / **CN10 Pin 9, 20** | Power / Logic Ground| **All Modules & 24V PSU**| `GND` / `BU` (Blue) | Common System Star Ground |

> [!NOTE]
> **Nucleo-64 (`MB1136`) Solder Bridge Checklist (`UM1724` Table 10):**
> - **`SB51` & `SB56` MUST be ON** (and **`SB46` & `SB52` MUST be OFF** — factory default): Keeps `PC0` and `PC1` routed to `CN7 Pin 38`/`36` (`A5`/`A4`) and prevents shorting `I2C1` (`PB8`/`PB9`) onto Motor 0 `CS`/`STEP`.
> - **`SB13` & `SB14` MUST be ON** (and **`SB62` & `SB63` MUST be OFF** — factory default): Routes `USART2` (`PA2`/`PA3`) internally to the ST-LINK USB Virtual COM port (`COM9`). Note that `CN10 Pin 35`/`37` are disconnected in this default state.
> - **Correction Notice on `PA13`/`PA14`:** On `MB1136`, `PA13` (`SWDIO`) and `PA14` (`SWCLK`) are on **`CN7 Pin 13` and `CN7 Pin 15`** (not `CN10`; `CN10 Pin 13`/`15` are `PA6`/`PA7`).

---

## 3. BIGTREETECH (BTT) / MKS TMC2240 v1.0 Stepper Driver Modules

### A. Module Pinout & Silkscreen Mapping (BTT vs. MKS StepStick Footprint)

Both **BIGTREETECH TMC2240 v1.0** and **MKS TMC2240 v1.0** share the identical 16-pin StepStick electrical footprint, onboard **$R_{\text{REF}} = 12\,\text{k}\Omega$** current reference resistor, and factory-default **SPI Mode** (`UART_EN` tied LOW). However, their PCB silkscreen labels for the motor coil pins and Pin 11 differ slightly:

```text
                       [DIAG0] [AIN]  <-- Top-edge header pins (BTT TMC2240 v1.0)
                   +------------------+
         EN (ENN) -| 16            1  |- GND
       MOSI (SDI) -| 15            2  |- VIO / VDD_IO (3.3V)
        CLK (SCK) -| 14            3  |- B1 (BTT) / 1B (MKS)  [TMC2240 OUT1B: Coil B1]
         CS (CSN) -| 13            4  |- B2 (BTT) / 1A (MKS)  [TMC2240 OUT2B: Coil B2]
       MISO (SDO) -| 12            5  |- A2 (BTT) / 2A (MKS)  [TMC2240 OUT2A: Coil A2]
         CLK / NC -| 11            6  |- A1 (BTT) / 2B (MKS)  [TMC2240 OUT1A: Coil A1]
             STEP -| 10            7  |- GND
              DIR -| 9             8  |- VM / VMOT (4.5V - 36V; 24V nominal)
                   +------------------+
```

- **Pin 11 (`CLK` on BTT / `NC` on MKS):** Connected to the TMC2240 external `CLK` input with an onboard pull-down to `GND` so the internal oscillator runs automatically. **Leave Pin 11 unconnected.**
- **Top-Edge `DIAG0` / `AIN` Pins (BTT TMC2240 v1.0):** `DIAG0` is an open-drain hardware stall/fault output. Because this firmware polls `SG4_RESULT` (`0x74`) over SPI2, `DIAG0` and `AIN` can be left unconnected.
- **Coil Naming vs. `DRV_STATUS` (`0x6F`) Diagnostics:** Internally, the TMC2240 IC defines **Pins 5 & 6 (`OUT2A`/`OUT1A`, BTT `A2`/`A1`, MKS `2A`/`2B`) as Bridge/Phase A** (`s2ga`, `ola`, `s2vsa` flags) and **Pins 3 & 4 (`OUT1B`/`OUT2B`, BTT `B1`/`B2`, MKS `1B`/`1A`) as Bridge/Phase B** (`s2gb`, `olb`, `s2vsb` flags).

### B. Driver 0: Z-Axis Motor
| StepStick Pin | BTT Silkscreen | MKS Silkscreen | Connects to STM32 / System | Wire Type / Recommendation |
| :---: | :---: | :---: | :--- | :--- |
| **1** | `GND` | `GND` | Common Ground (Nucleo `GND` + 24V PSU `-`) | Heavy gauge wire to Star Ground |
| **2** | `VIO` | `VDD_IO` | Nucleo **+3V3** (`CN7 Pin 16` / `CN6 Pin 4`) | Logic Power (3.3V) |
| **3** | `B1` (`2B`) | `1B` | Stepper Coil B, Lead 1 (`OUT1B`) | Twisted pair with Pin 4 |
| **4** | `B2` (`2A`) | `1A` | Stepper Coil B, Lead 2 (`OUT2B`) | Twisted pair with Pin 3 |
| **5** | `A2` (`1A`) | `2A` | Stepper Coil A, Lead 2 (`OUT2A`) | Twisted pair with Pin 6 |
| **6** | `A1` (`1B`) | `2B` | Stepper Coil A, Lead 1 (`OUT1A`) | Twisted pair with Pin 5 |
| **7** | `GND` | `GND` | Common Ground (Bridge to Pin 1) | Heavy gauge ground |
| **8** | `VM` | `VMOT` | **+24V DC** Power Supply | Equip with $\ge 100\,\mu\text{F}$ 35V/50V low-ESR cap to GND |
| **9** | `DIR` | `DIR` | STM32 **PC2** (Morpho **CN7 Pin 35**) | Signal Jumper *(Verify `CN7 Pin 35`, NOT `Pin 34`)* |
| **10** | `STEP` | `STEP` | STM32 **PC1** (Arduino `A4` / **CN7 Pin 36**) | Signal Jumper |
| **11** | `CLK` | `NC` | *Leave Unconnected* | Pulled to GND onboard for internal clock |
| **12** | `MISO` | `SDO` | STM32 **PB14** (Morpho **CN10 Pin 28**) | Shared `SPI2_MISO` |
| **13** | `CS` | `CS` | STM32 **PC0** (Arduino `A5` / **CN7 Pin 38**) | Dedicated `CS0` (Motor 0, Active LOW) |
| **14** | `CLK` | `SCK` | STM32 **PB13** (Morpho **CN10 Pin 30**) | Shared `SPI2_SCK` |
| **15** | `MOSI` | `SDI` | STM32 **PB15** (Morpho **CN10 Pin 26**) | Shared `SPI2_MOSI` |
| **16** | `EN` | `ENN` | STM32 **PC6** (Morpho **CN10 Pin 4**) | Active LOW enable |

---

### C. Driver 1: X-Axis Motor
| StepStick Pin | BTT Silkscreen | MKS Silkscreen | Connects to STM32 / System | Wire Type / Recommendation |
| :---: | :---: | :---: | :--- | :--- |
| **1** | `GND` | `GND` | Common Ground (Nucleo `GND` + 24V PSU `-`) | Heavy gauge wire to Star Ground |
| **2** | `VIO` | `VDD_IO` | Nucleo **+3V3** (`CN7 Pin 16` / `CN6 Pin 4`) | Logic Power (3.3V) |
| **3** | `B1` (`2B`) | `1B` | Stepper Coil B, Lead 1 (`OUT1B`) | Twisted pair with Pin 4 |
| **4** | `B2` (`2A`) | `1A` | Stepper Coil B, Lead 2 (`OUT2B`) | Twisted pair with Pin 3 |
| **5** | `A2` (`1A`) | `2A` | Stepper Coil A, Lead 2 (`OUT2A`) | Twisted pair with Pin 6 |
| **6** | `A1` (`1B`) | `2B` | Stepper Coil A, Lead 1 (`OUT1A`) | Twisted pair with Pin 5 |
| **7** | `GND` | `GND` | Common Ground (Bridge to Pin 1) | Heavy gauge ground |
| **8** | `VM` | `VMOT` | **+24V DC** Power Supply | Equip with $\ge 100\,\mu\text{F}$ 35V/50V low-ESR cap to GND |
| **9** | `DIR` | `DIR` | STM32 **PC5** (Morpho **CN10 Pin 6**) | Signal Jumper |
| **10** | `STEP` | `STEP` | STM32 **PC4** (Morpho **CN10 Pin 34**) | Signal Jumper |
| **11** | `CLK` | `NC` | *Leave Unconnected* | Pulled to GND onboard for internal clock |
| **12** | `MISO` | `SDO` | STM32 **PB14** (Morpho **CN10 Pin 28**) | Shared `SPI2_MISO` |
| **13** | `CS` | `CS` | STM32 **PC3** (Morpho **CN7 Pin 37**) | Dedicated `CS1` (Motor 1, Active LOW) |
| **14** | `CLK` | `SCK` | STM32 **PB13** (Morpho **CN10 Pin 30**) | Shared `SPI2_SCK` |
| **15** | `MOSI` | `SDI` | STM32 **PB15** (Morpho **CN10 Pin 26**) | Shared `SPI2_MOSI` |
| **16** | `EN` | `ENN` | STM32 **PC7** (Arduino `D9` / **CN10 Pin 19**) | Active LOW enable |

> [!WARNING]
> **TMC2240 Hardware & Power Protection (`TMC2240 Datasheet` p. 124–125):**
> 1. **Bulk Decoupling Capacitor:** Always install a low-ESR electrolytic capacitor ($\ge 100\,\mu\text{F}$, $35\text{V}$ or $50\text{V}$) directly across `VM` (`Pin 8`) and `GND` (`Pin 7`) within $\le 3\,\text{cm}$ of each driver module to absorb chopper current ripple.
> 2. **No Hot-Plugging:** **NEVER** connect or disconnect stepper motor leads while the 24V supply is energized.
> 3. **ESD / Frame Grounding:** Belt-driven axes (especially GT2 timing belts on the X-axis) can build up static charge; ground the stepper motor chassis/frame to system ground to prevent ESD resets.

---

## 4. AS5600 Magnetic Rotary Encoder Breakout Modules Wiring

The AS5600 has a factory-fixed 7-bit I2C slave address of **`0x36`**. Each module is routed to an independent hardware I2C peripheral on the STM32F446RE (`I2C1` and `I2C3`).

### A. Crucial 3.3V Mode Hardware Check on AS5600 Breakout Modules (`R1` Jumper)

> [!IMPORTANT]
> **3.3V Supply Strapping (`AS5600 Datasheet` Figure 13, p. 9):**
> The AS5600 IC has two supply pins: **`VDD5V`** (module `VCC` pin) and **`VDD3V3`** (module `3V3` pin). In **3.3V operation**, `VDD5V` and `VDD3V3` **must be tied together** to bypass the internal 5V$\to$3.3V LDO regulator:
> - **Check Resistor `R1` on your AS5600 Module:** Standard 7-pin AS5600 breakout modules have a $0\,\Omega$ resistor pad labeled **`R1`** that bridges `VCC` and `3V3`:
>   - **If `R1` ($0\,\Omega$) is installed on the module:** `VCC` and `3V3` are already tied together onboard. Connect Nucleo **+3V3** to `VCC` (and **never** apply 5V while `R1` is installed).
>   - **If `R1` is open / unpopulated on the module:** Feeding $+3.3\text{V}$ into `VCC` alone will starve the internal LDO (which needs $\ge 4.5\text{V}$) and cause I2C dropouts. You must **either bridge `R1` with solder OR connect both the `VCC` and `3V3` pins on the module to Nucleo `+3V3`**.
> - **Onboard Pull-Ups & Decoupling:** The AS5600 breakout module already includes onboard $100\text{ nF}$ / $1\,\mu\text{F}$ decoupling capacitors and $4.7\,\text{k}\Omega$ I2C pull-up resistors (`R2`, `R3`) tied to `VCC`.

### B. Encoder 1: X-Axis (Bus `I2C1`)
| Module Pin Silkscreen | AS5600 IC Pin | Nucleo Header & Pin | Function & Notes |
| :---: | :---: | :--- | :--- |
| **VCC** | `VDD5V` (Pin 1) | Nucleo **+3V3** (`CN7 Pin 16` / `CN6 Pin 4`) | 3.3V Logic Supply |
| **3V3** | `VDD3V3` (Pin 2)| Tie to **+3V3** *(or leave open if `R1` $0\,\Omega$ is bridged onboard)* | Required in 3.3V mode (`Datasheet` Fig. 13) |
| **GND** | `GND` (Pin 4) | Nucleo **GND** (`CN6 Pin 6` / `CN7 Pin 19`) | Common Ground |
| **SCL** | `SCL` (Pin 7) | Arduino **D15** / Morpho **CN10 Pin 3** (**PB8**) | `I2C1_SCL` (onboard $4.7\,\text{k}\Omega$ pull-up) |
| **SDA** | `SDA` (Pin 6) | Arduino **D14** / Morpho **CN10 Pin 5** (**PB9**) | `I2C1_SDA` (onboard $4.7\,\text{k}\Omega$ pull-up) |
| **DIR** | `DIR` (Pin 8) | Tie to **GND** *(or **+3V3** for CCW)* | `GND` = CW rotation increases angle; **never leave floating** |
| **OUT** | `OUT` (Pin 3) | *Leave Unconnected* | Analog / PWM output (unused in I2C mode) |
| **GPO / PGO / GPR** | `PGO` (Pin 5) | *Leave Unconnected* | OTP Program Option (internal pull-up; **do not ground**) |

### C. Encoder 2: Z-Axis (Bus `I2C3`)
| Module Pin Silkscreen | AS5600 IC Pin | Nucleo Header & Pin | Function & Notes |
| :---: | :---: | :--- | :--- |
| **VCC** | `VDD5V` (Pin 1) | Nucleo **+3V3** (`CN7 Pin 16` / `CN6 Pin 4`) | 3.3V Logic Supply |
| **3V3** | `VDD3V3` (Pin 2)| Tie to **+3V3** *(or leave open if `R1` $0\,\Omega$ is bridged onboard)* | Required in 3.3V mode (`Datasheet` Fig. 13) |
| **GND** | `GND` (Pin 4) | Nucleo **GND** (`CN6 Pin 7` / `CN7 Pin 20`) | Common Ground |
| **SCL** | `SCL` (Pin 7) | Arduino **D7** / Morpho **CN10 Pin 23** (**PA8**) | `I2C3_SCL` (onboard $4.7\,\text{k}\Omega$ pull-up) |
| **SDA** | `SDA` (Pin 6) | Morpho **CN10 Pin 1** (**PC9**) | `I2C3_SDA` (onboard $4.7\,\text{k}\Omega$ pull-up) |
| **DIR** | `DIR` (Pin 8) | Tie to **GND** *(or **+3V3** for CCW)* | `GND` = CW rotation increases angle; **never leave floating** |
| **OUT** | `OUT` (Pin 3) | *Leave Unconnected* | Analog / PWM output (unused in I2C mode) |
| **GPO / PGO / GPR** | `PGO` (Pin 5) | *Leave Unconnected* | OTP Program Option (internal pull-up; **do not ground**) |

---

## 5. 5V 3-Wire Inductive Proximity Sensor Wiring (Morpho Header)

The 5V 3-wire inductive proximity sensor connects **entirely to the Morpho headers** using **`PC8`** (**CN10 Pin 2**) for the signal output, **`CN7 Pin 18`** for `+5V`, and **`CN7 Pin 19`** (or **`CN10 Pin 9`**) for `GND`.

### A. Sensor Pinout & Morpho Interconnect Table (IEC Standard 3-Wire Color Code)

| Wire Color | IEC Label | Sensor Function | Connects to STM32 Morpho Header | Alternate Header / Supply | Notes |
| :---: | :---: | :--- | :--- | :--- | :--- |
| **Brown** | `BN` | `+5V` Supply | Morpho **CN7 Pin 18** (`+5V`) | Arduino **5V** (`CN6 Pin 5`) or 5V Buck | 5V DC power rail |
| **Blue** | `BU` | `0V` Ground | Morpho **CN7 Pin 19** (`GND`) | Morpho **CN10 Pin 9** / **Pin 20** (`GND`) | Common System Ground |
| **Black** | `BK` | Signal (`OUT`) | Morpho **CN10 Pin 2** (**PC8**) | — | `GPIO_Input` / `GPIO_EXTI8` (5V-tolerant `FT` pin) |

### B. Electrical Interface & GPIO Configuration Notes (`PC8`, `DS10693` Table 10)

Because **`PC8`** on the STM32F446RE has a **5V-Tolerant (`FT`)** I/O structure ($V_{IN(\max)} = V_{DD} + 4.0\text{V} = 5.5\text{V}$ when internal pull-up/pull-down is disabled), it interfaces cleanly with a 5V proximity sensor:

1. **NPN 5V Sensor (Sinking / Active LOW — Most Common):**
   - **Open-Collector NPN:** Enable the STM32 internal pull-up resistor (`GPIO_PULLUP`, which pulls `PC8` to $+3.3\text{V}$) or connect an external $4.7\,\text{k}\Omega - 10\,\text{k}\Omega$ pull-up resistor from `PC8` to **+3V3** (`CN7 Pin 16`).
   - **NPN with Internal Pull-Up to +5V:** Many 3-wire NPN sensors include an internal pull-up resistor from `BK` to `BN` ($+5\text{V}$) for their indicator LED. Per `DS10693`, true 5V tolerance on `FT` pins requires the internal pull-up to be disabled (`GPIO_NOPULL`) so current is not injected into the $3.3\text{V}$ rail:
     - If your NPN sensor outputs $\approx 5\text{V}$ on `BK` when untriggered, configure `PC8` as **`GPIO_NOPULL`** (direct connection is safe up to $5.5\text{V}$), **or** place a `1N4148` / `1N5819` diode in series (cathode toward `BK`, anode toward `PC8`) and enable `GPIO_PULLUP` to $+3.3\text{V}$.
2. **PNP 5V Sensor (Sourcing / Active HIGH):**
   - Outputs $+5\text{V}$ when triggered and floats (or pulls low) when untriggered.
   - Configure `PC8` as **`GPIO_NOPULL`** and add an external $10\,\text{k}\Omega$ pull-down resistor between `PC8` (`CN10 Pin 2`) and `GND` (`CN10 Pin 9`), or use a $4.7\,\text{k}\Omega / 10\,\text{k}\Omega$ voltage divider to scale $+5\text{V} \to +3.4\text{V}$.

---

## 6. Stepper Motor Phase Wiring (4-Lead Bipolar)

Standard bipolar stepper motors have two independent isolated coils: **Coil B** (StepStick Pins 3 & 4) and **Coil A** (StepStick Pins 5 & 6).

| StepStick Pin | BTT Label | MKS Label | TMC2240 IC Pin | Typical Wire Color (Standard) | Alternate Color Code |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **Pin 3** | `B1` (`2B`) | `1B` | `OUT1B` (Coil B1) | **Green** | Blue |
| **Pin 4** | `B2` (`2A`) | `1A` | `OUT2B` (Coil B2) | **Black** | Red |
| **Pin 5** | `A2` (`1A`) | `2A` | `OUT2A` (Coil A2) | **Red** | Green |
| **Pin 6** | `A1` (`1B`) | `2B` | `OUT1A` (Coil A1) | **Blue** | Black |

> [!TIP]
> **Identifying Stepper Coils with Multimeter:**
> - Set multimeter to resistance mode ($\Omega$).
> - Probe pairs of motor wires. Two wires showing $\approx 1.5 - 4\,\Omega$ belong to the **same coil** — connect one pair to **Pins 3 & 4** (`B1`/`B2`) and the other pair to **Pins 5 & 6** (`A2`/`A1`).
> - There must be infinite resistance (open circuit) between the two coils.
> - If the motor turns backwards during operation, invert direction in software (`GCONF.shaft`) or swap the two wires of **one coil only** (e.g. swap `B1` and `B2`).

---

## 7. Power & Ground Distribution Rules

```text
     +-----------------------------------+          +-----------------------------------+
     |        24V DC Power Supply        |          |  Nucleo +5V (CN7-18) / 5V Buck    |
     +-----------------------------------+          +-----------------------------------+
        (+) 24V                    (-) GND             (+) 5V                     (-) GND
           |                          |                   |                          |
           +----------+               |                   v                          |
           |          |               |                +------+                      |
           v          v               |                |  BN  |                      |
        +------+   +------+           |                |  5V  |                      |
        |  VM  |   |  VM  |           |                | Prox |                      |
        |      |   |      |           |                |Sensor|                      |
        | TMC0 |   | TMC1 |           |                |  BU  |                      |
        |      |   |      |           |                +------+                      |
        | GND  |   | GND  |           |                   |                          |
        +------+   +------+           |                   |                          |
           |          |               |                   |                          |
           +-----+----+---------------+-------------------+--------------------------+
                 |
                 v
              [Star Ground Splice Point] <------- Nucleo GND (CN7-19 / CN10-9)
```

1. **Common Star Ground:** The negative terminal (`-`) of the 24V power supply, any external 5V buck converter `GND`, the Blue (`BU`) ground lead of the 5V proximity sensor, and the `GND` pins of the Nucleo board **MUST** be solidly connected together at a single common ground reference.
2. **Decoupling Capacitors:** Keep lead lengths between the 24V electrolytic capacitors ($\ge 100\,\mu\text{F}$) and the TMC2240 `VM`/`GND` pins as short as possible ($\le 3\,\text{cm}$).
3. **Power Rail Current Limits (`UM1724` Section 7.5):**
   - **Nucleo `+3V3` Rail (`CN7 Pin 16` / `CN6 Pin 4`):** Supplied by the onboard `LD39050PU33R` LDO (500 mA max). Powers `VIO` on both TMC2240 modules and `VCC`/`3V3` on both AS5600 modules ($\approx 20\,\text{mA}$ total).
   - **Nucleo `+5V` Rail (`CN7 Pin 18` / `CN6 Pin 5`):** When powered via ST-LINK USB (`JP5` on `U5V`), total board +5V current is limited to **300 mA max**. This easily powers the 5V proximity sensor ($\approx 10\text{–}20\,\text{mA}$), but any high-current 5V actuator (such as the end-effector `MG90S`/`SG90` servo) **must** be powered from the external **24V $\to$ 5V (3A) Buck Converter** (with common `GND`).
4. **Motor Cable Routing:** Keep high-current motor lead wires (`B1`, `B2`, `A2`, `A1`) physically separated from SPI, I2C, and proximity sensor signal wires to prevent inductive noise coupling.
