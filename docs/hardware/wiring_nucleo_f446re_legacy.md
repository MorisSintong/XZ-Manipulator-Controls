# Complete Wiring & Hardware Interconnect Specification
## STM32F446RE Nucleo-64 ➔ Triple BTT/MKS TMC2240 Modules (2× NEMA 17 + 1× NEMA 11), Dual AS5600 Modules & 5V Inductive Proximity Sensor

> Historical wiring reference for the former NUCLEO-F446RE setup. The current physical board is documented in [`wiring.md`](wiring.md); do not use the Nucleo pin assignments for the SKR Octopus Pro.

**Project:** XZ-Manipulator-Controls

**Target Board:** STM32F446RE Nucleo-64 (`MB1136` — **ST Morpho Headers `CN7` & `CN10` Only**)

**Actuators:**

- **Z-Axis & X-Axis (Closed-Loop):** 2× NEMA 17 Stepper Motors driven by **BIGTREETECH (BTT) / MKS TMC2240 v1.0** StepStick Modules (SPI Mode, $R_{\text{REF}} = 12\,\text{k}\Omega$)
- **End-Effector Rotational / $\theta$-Axis (Open-Loop Harness Provision):** 1× NEMA 11 Stepper Motor driven by a 3rd **BTT / MKS TMC2240 v1.0** StepStick Module (SPI Mode, $R_{\text{REF}} = 12\,\text{k}\Omega$, no rotary encoder; provisioned on `CN7 Pins 1–4`, inactive in current dual-axis `X`/`Z` firmware profiles)

**Sensors:** 2× **AS5600 Breakout Modules** (12-bit Magnetic Absolute Rotary Encoders on X & Z axes, Dual I2C), 1× **5V 3-Wire Inductive Proximity Sensor** (Morpho GPIO)

**Telemetry:** Profile-dependent on `USART2` (115200 baud, ST-LINK VCOM `COM9`, if present on probe) and **SEGGER RTT Terminal 0**: `motion_diagnostics` uses binary-only `USART2` frames + RTT human logs, whereas `stallguard_cli` broadcasts interactive ASCII on both `USART2` and RTT

---

## 1. System Architecture Diagram

```text
                                         +-----------------------------------+
                                         |     STM32F446RE Nucleo-64         |
                                         |   (ST Morpho CN7 & CN10 Only)     |
                                         +-----------------------------------+
                                                |          |          |
                     [SPI2 Bus - CN10]          |          |          |         [I2C Buses - CN10]
                 PB13 (CN10-30): SCK            |          |          |      I2C1 (PB8/PB9): Enc 1 (X)
                 PB14 (CN10-28): MISO           |          |          |      I2C3 (PA8/PC9): Enc 2 (Z)
                 PB15 (CN10-26): MOSI           |          |          |
          +--------------------+----------------+          |          +-----------------+
          |                    |                           | [Morpho GPIO]              |
          |                    |                           | PC8 (CN10 Pin 2)           |
          v                    v                           v                            v
 +------------------+ +------------------+ +------------------+      +------------------+ +------------------+
 | BTT/MKS TMC2240  | | BTT/MKS TMC2240  | | BTT/MKS TMC2240  |      | AS5600 Module 1  | | AS5600 Module 2  |
 | (Z-Axis/Motor 0) | | (X-Axis/Motor 1) | | (Rot. / Motor 2) |      | (X-Axis)         | | (Z-Axis)         |
 |  CS0:  PC0       | |  CS1:  PC3       | |  CS2:  PC10      |      | Addr: 0x36       | | Addr: 0x36       |
 |  STEP: PC1       | |  STEP: PC4       | |  STEP: PC11      |      | Bus:  I2C1       | | Bus:  I2C3       |
 |  DIR:  PC2       | |  DIR:  PC5       | |  DIR:  PC12      |      +------------------+ +------------------+
 |  ENN:  PC6       | |  ENN:  PC7       | |  ENN:  PD2       |
 +------------------+ +------------------+ +------------------+               +--------------------------+
          |                    |                    |                         | 5V 3-Wire Inductive      |
          v                    v                    v                         | Proximity Sensor         |
    [NEMA 17 (Z)]        [NEMA 17 (X)]       [NEMA 11 (Theta)]                | BN (+V)  : +5V (CN7-18)  |
    (Closed-Loop)        (Closed-Loop)      (End-Effector Rot.)               | BU (GND) : GND (CN7-19)  |
                                            (Open-Loop, No Enc)               | BK (OUT) : PC8 (CN10-2)  |
                                                                              +--------------------------+
```

---

## 2. Master Pinout Table (STM32 Nucleo-64 `MB1136` — ST Morpho Headers Only)

All external modules connect **exclusively via the ST Morpho headers** (double-row male headers **`CN7`** and **`CN10`**, verified against `UM1724` Table 29). Do not route signals to the female Arduino headers (`CN5`/`CN6`/`CN8`/`CN9`).

| STM32 Pin | ST Morpho Header Pin | Peripheral Function | Connected Module | Module Pin / Net | Description & Notes |
| :--- | :--- | :---: | :--- | :--- | :--- |
| **PB13** | Morpho **CN10 Pin 30** *(even row)* | `SPI2_SCK` (AF5) | **All 3 TMC2240 Modules** | Pin 14 (`CLK` / `SCK`) | Shared SPI Clock (2.625 MHz @ 84 MHz SYSCLK, SPI Mode 3) |
| **PB14** | Morpho **CN10 Pin 28** *(even row)* | `SPI2_MISO` (AF5) | **All 3 TMC2240 Modules** | Pin 12 (`MISO` / `SDO`) | Shared SPI Data Input to MCU (internal pull-up enabled) |
| **PB15** | Morpho **CN10 Pin 26** *(even row)* | `SPI2_MOSI` (AF5) | **All 3 TMC2240 Modules** | Pin 15 (`MOSI` / `SDI`) | Shared SPI Data Output from MCU |
| **PC0** | Morpho **CN7 Pin 38** *(even row)* | `GPIO_Output` | **TMC2240 (Z / Motor 0)** | Pin 13 (`CS`) | Chip Select 0 (Active LOW, pull-up). Requires `SB51` **ON**, `SB52` **OFF** |
| **PC1** | Morpho **CN7 Pin 36** *(even row)* | `GPIO_Output` | **TMC2240 (Z / Motor 0)** | Pin 10 (`STEP`) | Step pulse output for Z-axis (NEMA 17). Requires `SB56` **ON**, `SB46` **OFF** |
| **PC2** | Morpho **CN7 Pin 35** *(odd row)* | `GPIO_Output` | **TMC2240 (Z / Motor 0)** | Pin 9 (`DIR`) | Direction control for Z-axis *(Note: `CN7 Pin 34` is `PB0`, NOT `PC2`!)* |
| **PC6** | Morpho **CN10 Pin 4** *(even row)* | `GPIO_Output` | **TMC2240 (Z / Motor 0)** | Pin 16 (`EN` / `ENN`) | Enable 0 (Active LOW; HIGH = disabled) |
| **PC3** | Morpho **CN7 Pin 37** *(odd row)* | `GPIO_Output` | **TMC2240 (X / Motor 1)** | Pin 13 (`CS`) | Chip Select 1 (Active LOW, pull-up) |
| **PC4** | Morpho **CN10 Pin 34** *(even row)* | `GPIO_Output` | **TMC2240 (X / Motor 1)** | Pin 10 (`STEP`) | Step pulse output for X-axis (NEMA 17) |
| **PC5** | Morpho **CN10 Pin 6** *(even row)* | `GPIO_Output` | **TMC2240 (X / Motor 1)** | Pin 9 (`DIR`) | Direction control for X-axis |
| **PC7** | Morpho **CN10 Pin 19** *(odd row)* | `GPIO_Output` | **TMC2240 (X / Motor 1)** | Pin 16 (`EN` / `ENN`) | Enable 1 (Active LOW; HIGH = disabled) |
| **PC10** | Morpho **CN7 Pin 1** *(odd row)* | `GPIO_Output` | **TMC2240 ($\theta$ / Motor 2)** | Pin 13 (`CS`) | Chip Select 2 (Active LOW, pull-up) — End-Effector Rotational Axis |
| **PC11** | Morpho **CN7 Pin 2** *(even row)* | `GPIO_Output` | **TMC2240 ($\theta$ / Motor 2)** | Pin 10 (`STEP`) | Step pulse output for Rotational axis (NEMA 11, Open-Loop) |
| **PC12** | Morpho **CN7 Pin 3** *(odd row)* | `GPIO_Output` | **TMC2240 ($\theta$ / Motor 2)** | Pin 9 (`DIR`) | Direction control for Rotational axis (NEMA 11, Open-Loop) |
| **PD2** | Morpho **CN7 Pin 4** *(even row)* | `GPIO_Output` | **TMC2240 ($\theta$ / Motor 2)** | Pin 16 (`EN` / `ENN`) | Enable 2 (Active LOW; HIGH = disabled) — Top 2×2 block on `CN7` |
| **PB8** | Morpho **CN10 Pin 3** *(odd row)* | `I2C1_SCL` (AF4) | **AS5600 Module 1 (X-Axis)** | `SCL` | I2C1 Clock (100–400 kHz, onboard 4.7k pull-up on module) |
| **PB9** | Morpho **CN10 Pin 5** *(odd row)* | `I2C1_SDA` (AF4) | **AS5600 Module 1 (X-Axis)** | `SDA` | I2C1 Data (100–400 kHz, onboard 4.7k pull-up on module) |
| **PA8** | Morpho **CN10 Pin 23** *(odd row)* | `I2C3_SCL` (AF4) | **AS5600 Module 2 (Z-Axis)** | `SCL` | I2C3 Clock (100–400 kHz, onboard 4.7k pull-up on module) |
| **PC9** | Morpho **CN10 Pin 1** *(odd row)* | `I2C3_SDA` (AF4) | **AS5600 Module 2 (Z-Axis)** | `SDA` | I2C3 Data (100–400 kHz, onboard 4.7k pull-up on module) |
| **PC8** | Morpho **CN10 Pin 2** *(even row)* | `GPIO_Input` / `GPIO_EXTI8` | **5V Proximity Sensor** | `BK` (Black / `OUT`) | 3-wire 5V proximity sensor signal (5V-tolerant `FT` pin) |
| **PC13** | Morpho **CN7 Pin 23** *(Onboard `B1`)*| `GPIO_EXTI13` / Input | **User Interface** | — | Active LOW (`SB17` ON): start / stop motion cycle |
| **PA5** | Morpho **CN10 Pin 11** *(Onboard `LD2`)*| `GPIO_Output` | **Status LED (`LD2`)** | — | Green user LED (`SB21` ON): ON = cycling, fast blink = fault |
| **PA2** | Internal **ST-LINK VCOM** *(CN10 Pin 35 if `SB62` ON)* | `USART2_TX` (AF7) | **CDC Serial / COM9** | ST-LINK RX | Routed to USB VCOM via `SB13` ON (`SB62` OFF by default) |
| **PA3** | Internal **ST-LINK VCOM** *(CN10 Pin 37 if `SB63` ON)* | `USART2_RX` (AF7) | **CDC Serial / COM9** | ST-LINK TX | Routed to USB VCOM via `SB14` ON (`SB63` OFF by default) |
| **PA13** | Morpho **CN7 Pin 13** (`SWDIO`) | `SYS_JTMS-SWDIO` | **ST-Link / J-Link OB** | `SWDIO` | Debug & SEGGER RTT channel — **do not use as GPIO** |
| **PA14** | Morpho **CN7 Pin 15** (`SWCLK`) | `SYS_JTCK-SWCLK` | **ST-Link / J-Link OB** | `SWCLK` | Debug Clock — **do not use as GPIO** |
| **PB3** | Morpho **CN10 Pin 31** (`SWO`) | `SYS_JTDO-SWO` | **ST-Link Trace (`SWO`)** | `SWO` | Reserved for SWO trace via `SB15` ON (`MotionFirmware.ioc`) |
| **+5V** | Morpho **CN7 Pin 18** *(even row)* | 5V Supply (+5.0V) | **5V Proximity Sensor** | `BN` (Brown / `+V`) | 5V rail (max 300 mA total when USB-powered; or 5V buck) |
| **+3V3** | Morpho **CN7 Pin 16** *(even row)* | Logic Supply (+3.3V) | **All 3 Drivers & 2 Encoders**| `VIO` / `VCC` | 3.3V from Nucleo `LD39050PU33R` LDO (max 500 mA) |
| **GND** | Morpho **CN7 Pin 8, 19, 20, 22** / **CN10 Pin 9, 20** | Power / Logic Ground| **All Modules & 24V PSU**| `GND` / `BU` (Blue) | Common System Star Ground |

> [!NOTE]
> **Nucleo-64 (`MB1136`) Solder Bridge & Morpho Header Checklist (`UM1724` Table 10 & Table 29):**
> - **ST Morpho-Only Interconnect:** All signals and power rails are routed exclusively through the double-row **ST Morpho headers (`CN7` and `CN10`)**. Notice that **`PC10` (`CN7 Pin 1`)**, **`PC11` (`CN7 Pin 2`)**, **`PC12` (`CN7 Pin 3`)**, and **`PD2` (`CN7 Pin 4`)** form a dedicated 2×2 block at the very top of `CN7` that is not routed to any Arduino header and has zero onboard solder-bridge conflicts.
> - **`SB51` & `SB56` MUST be ON** (and **`SB46` & `SB52` MUST be OFF** — factory default): Keeps `PC0` and `PC1` routed to `CN7 Pin 38`/`36` and prevents shorting `I2C1` (`PB8`/`PB9`) onto Motor 0 `CS`/`STEP`.
> - **`SB13` & `SB14` MUST be ON** (and **`SB62` & `SB63` MUST be OFF** — factory default): Routes `USART2` (`PA2`/`PA3`) internally to the ST-LINK USB Virtual COM port (`COM9`). Note that `CN10 Pin 35`/`37` are disconnected in this default state.
> - **Correction Notice on `PA13`/`PA14`:** On `MB1136`, `PA13` (`SWDIO`) and `PA14` (`SWCLK`) are on **`CN7 Pin 13` and `CN7 Pin 15`** (not `CN10`; `CN10 Pin 13`/`15` are `PA6`/`PA7`).

---

## 3. BIGTREETECH (BTT) / MKS TMC2240 v1.0 Stepper Driver Modules

### A. Module Pinout & Silkscreen Mapping (BTT vs. MKS StepStick Footprint)

All three **BIGTREETECH TMC2240 v1.0** / **MKS TMC2240 v1.0** modules share the identical 16-pin StepStick electrical footprint, onboard **$R_{\text{REF}} = 12\,\text{k}\Omega$** current reference resistor, and factory-default **SPI Mode** (`UART_EN` tied LOW). However, their PCB silkscreen labels for the motor coil pins and Pin 11 differ slightly:

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

### B. Driver 0: Z-Axis Motor (NEMA 17 — Closed-Loop via AS5600 Encoder 2)
| StepStick Pin | BTT Silkscreen | MKS Silkscreen | Connects to STM32 Morpho / System | Wire Type / Recommendation |
| :---: | :---: | :---: | :--- | :--- |
| **1** | `GND` | `GND` | Common Ground (Morpho `GND` + 24V PSU `-`) | Heavy gauge wire to Star Ground |
| **2** | `VIO` | `VDD_IO` | Morpho **CN7 Pin 16** (**+3V3**) | Logic Power (3.3V) |
| **3** | `B1` (`2B`) | `1B` | NEMA 17 (Z) Coil B, Lead 1 (`OUT1B`) | Twisted pair with Pin 4 |
| **4** | `B2` (`2A`) | `1A` | NEMA 17 (Z) Coil B, Lead 2 (`OUT2B`) | Twisted pair with Pin 3 |
| **5** | `A2` (`1A`) | `2A` | NEMA 17 (Z) Coil A, Lead 2 (`OUT2A`) | Twisted pair with Pin 6 |
| **6** | `A1` (`1B`) | `2B` | NEMA 17 (Z) Coil A, Lead 1 (`OUT1A`) | Twisted pair with Pin 5 |
| **7** | `GND` | `GND` | Common Ground (Bridge to Pin 1) | Heavy gauge ground |
| **8** | `VM` | `VMOT` | **+24V DC** Power Supply | Equip with $\ge 100\,\mu\text{F}$ 35V/50V low-ESR cap to GND |
| **9** | `DIR` | `DIR` | STM32 **PC2** (Morpho **CN7 Pin 35**) | Signal Jumper *(Verify `CN7 Pin 35`, NOT `Pin 34`)* |
| **10** | `STEP` | `STEP` | STM32 **PC1** (Morpho **CN7 Pin 36**) | Signal Jumper |
| **11** | `CLK` | `NC` | *Leave Unconnected* | Pulled to GND onboard for internal clock |
| **12** | `MISO` | `SDO` | STM32 **PB14** (Morpho **CN10 Pin 28**) | Shared `SPI2_MISO` |
| **13** | `CS` | `CS` | STM32 **PC0** (Morpho **CN7 Pin 38**) | Dedicated `CS0` (Motor 0, Active LOW) |
| **14** | `CLK` | `SCK` | STM32 **PB13** (Morpho **CN10 Pin 30**) | Shared `SPI2_SCK` |
| **15** | `MOSI` | `SDI` | STM32 **PB15** (Morpho **CN10 Pin 26**) | Shared `SPI2_MOSI` |
| **16** | `EN` | `ENN` | STM32 **PC6** (Morpho **CN10 Pin 4**) | Active LOW enable |

---

### C. Driver 1: X-Axis Motor (NEMA 17 — Closed-Loop via AS5600 Encoder 1)
| StepStick Pin | BTT Silkscreen | MKS Silkscreen | Connects to STM32 Morpho / System | Wire Type / Recommendation |
| :---: | :---: | :---: | :--- | :--- |
| **1** | `GND` | `GND` | Common Ground (Morpho `GND` + 24V PSU `-`) | Heavy gauge wire to Star Ground |
| **2** | `VIO` | `VDD_IO` | Morpho **CN7 Pin 16** (**+3V3**) | Logic Power (3.3V) |
| **3** | `B1` (`2B`) | `1B` | NEMA 17 (X) Coil B, Lead 1 (`OUT1B`) | Twisted pair with Pin 4 |
| **4** | `B2` (`2A`) | `1A` | NEMA 17 (X) Coil B, Lead 2 (`OUT2B`) | Twisted pair with Pin 3 |
| **5** | `A2` (`1A`) | `2A` | NEMA 17 (X) Coil A, Lead 2 (`OUT2A`) | Twisted pair with Pin 6 |
| **6** | `A1` (`1B`) | `2B` | NEMA 17 (X) Coil A, Lead 1 (`OUT1A`) | Twisted pair with Pin 5 |
| **7** | `GND` | `GND` | Common Ground (Bridge to Pin 1) | Heavy gauge ground |
| **8** | `VM` | `VMOT` | **+24V DC** Power Supply | Equip with $\ge 100\,\mu\text{F}$ 35V/50V low-ESR cap to GND |
| **9** | `DIR` | `DIR` | STM32 **PC5** (Morpho **CN10 Pin 6**) | Signal Jumper |
| **10** | `STEP` | `STEP` | STM32 **PC4** (Morpho **CN10 Pin 34**) | Signal Jumper |
| **11** | `CLK` | `NC` | *Leave Unconnected* | Pulled to GND onboard for internal clock |
| **12** | `MISO` | `SDO` | STM32 **PB14** (Morpho **CN10 Pin 28**) | Shared `SPI2_MISO` |
| **13** | `CS` | `CS` | STM32 **PC3** (Morpho **CN7 Pin 37**) | Dedicated `CS1` (Motor 1, Active LOW) |
| **14** | `CLK` | `SCK` | STM32 **PB13** (Morpho **CN10 Pin 30**) | Shared `SPI2_SCK` |
| **15** | `MOSI` | `SDI` | STM32 **PB15** (Morpho **CN10 Pin 26**) | Shared `SPI2_MOSI` |
| **16** | `EN` | `ENN` | STM32 **PC7** (Morpho **CN10 Pin 19**) | Active LOW enable |

---

### D. Driver 2: End-Effector Rotational Axis / $\theta$-Axis Motor (NEMA 11 — Open-Loop)

The end-effector rotational axis ($\theta$-axis / Motor 2) uses a compact **NEMA 11** 4-lead bipolar stepper motor driven by a 3rd **BTT / MKS TMC2240 v1.0** module in an **open-loop configuration** (no rotary encoder is installed on this axis). Its 4 dedicated control signals (`CS2`, `STEP2`, `DIR2`, `ENN2`) occupy the contiguous pure-Morpho top block **`CN7 Pins 1–4`** (`PC10`, `PC11`, `PC12`, `PD2`), while sharing the `SPI2` bus (`PB13`/`PB14`/`PB15` on `CN10`).

| StepStick Pin | BTT Silkscreen | MKS Silkscreen | Connects to STM32 Morpho / System | Wire Type / Recommendation |
| :---: | :---: | :---: | :--- | :--- |
| **1** | `GND` | `GND` | Common Ground (Morpho `GND` + 24V PSU `-`) | Heavy gauge wire to Star Ground |
| **2** | `VIO` | `VDD_IO` | Morpho **CN7 Pin 16** (**+3V3**) | Logic Power (3.3V) |
| **3** | `B1` (`2B`) | `1B` | NEMA 11 ($\theta$) Coil B, Lead 1 (`OUT1B`) | Flexible drag-chain twisted pair with Pin 4 |
| **4** | `B2` (`2A`) | `1A` | NEMA 11 ($\theta$) Coil B, Lead 2 (`OUT2B`) | Flexible drag-chain twisted pair with Pin 3 |
| **5** | `A2` (`1A`) | `2A` | NEMA 11 ($\theta$) Coil A, Lead 2 (`OUT2A`) | Flexible drag-chain twisted pair with Pin 6 |
| **6** | `A1` (`1B`) | `2B` | NEMA 11 ($\theta$) Coil A, Lead 1 (`OUT1A`) | Flexible drag-chain twisted pair with Pin 5 |
| **7** | `GND` | `GND` | Common Ground (Bridge to Pin 1) | Heavy gauge ground |
| **8** | `VM` | `VMOT` | **+24V DC** Power Supply | Equip with $\ge 100\,\mu\text{F}$ 35V/50V low-ESR cap to GND |
| **9** | `DIR` | `DIR` | STM32 **PC12** (Morpho **CN7 Pin 3**) | Signal Jumper (Pure Morpho odd row) |
| **10** | `STEP` | `STEP` | STM32 **PC11** (Morpho **CN7 Pin 2**) | Signal Jumper (Pure Morpho even row) |
| **11** | `CLK` | `NC` | *Leave Unconnected* | Pulled to GND onboard for internal clock |
| **12** | `MISO` | `SDO` | STM32 **PB14** (Morpho **CN10 Pin 28**) | Shared `SPI2_MISO` |
| **13** | `CS` | `CS` | STM32 **PC10** (Morpho **CN7 Pin 1**) | Dedicated `CS2` (Motor 2, Active LOW) |
| **14** | `CLK` | `SCK` | STM32 **PB13** (Morpho **CN10 Pin 30**) | Shared `SPI2_SCK` |
| **15** | `MOSI` | `SDI` | STM32 **PB15** (Morpho **CN10 Pin 26**) | Shared `SPI2_MOSI` |
| **16** | `EN` | `ENN` | STM32 **PD2** (Morpho **CN7 Pin 4**) | Active LOW enable (or tie to `GND` if software-enabled via `CHOPCONF.TOFF`) |

> [!IMPORTANT]
> **Datasheet-Verified Pinout & NEMA 11 Current Scaling (`UM1724 Rev 17`, `DS10693 Rev 11`, `TMC2240 Datasheet`):**
> 1. **Morpho Header & LQFP64 Verification (`UM1724 Table 29` p. 59, `DS10693 Table 10` p. 54):**
>    - `PC10` (`CN7 Pin 1`, LQFP64 Pin 51), `PC11` (`CN7 Pin 2`, LQFP64 Pin 52), `PC12` (`CN7 Pin 3`, LQFP64 Pin 53), and `PD2` (`CN7 Pin 4`, LQFP64 Pin 54) are all **5V-tolerant (`FT`)** standard I/Os wired directly to the `CN7` Morpho header with **zero solder-bridge disconnects** (unlike `PC14`/`PC15` on `CN7 Pins 25/27` or `PH0`/`PH1` on `CN7 Pins 29/31`).
> 2. **Open-Loop Control (No 3rd Encoder):** Because the NEMA 11 end-effector rotational axis operates under low inertial load (rotating only the vacuum suction cup and picked capacitor), it runs in **open-loop microstepping** ($1.8^\circ$ full step = $3200\text{ microsteps/rev}$ or $\approx 8.89\text{ steps/deg}$ at $1/16$ microstepping) without an AS5600 encoder.
> 3. **Exact TMC2240 Current Scaling for NEMA 11 (`TMC2240 Datasheet Table 16-18`, p. 46-48 & `DRV_CONF`, p. 85-87):**
>    - With the standard BTT/MKS onboard reference resistor $R_{\text{REF}} = 12\,\text{k}\Omega$, setting `DRV_CONF.CURRENT_RANGE = 0b00` (`0x0A` bits `1:0`, $K_{\text{IFS}} = 11.75\,\text{A}\cdot\text{k}\Omega$) selects the lowest full-scale range of **$I_{\text{FS}} = 1.0\,\text{A}_{\text{peak}}$ ($0.69\,\text{A}_{\text{RMS}}$)** (`Table 17` & `Table 18`, p. 47-48), whereas the NEMA 17 axes (`Drivers 0 & 1`) use `CURRENT_RANGE = 0b01` ($2.0\,\text{A}_{\text{peak}} / 1.41\,\text{A}_{\text{RMS}}$) or `0b10` ($3.0\,\text{A}_{\text{peak}} / 2.12\,\text{A}_{\text{RMS}}$).
>    - To set a target phase current of **$400\text{-}600\,\text{mA}_{\text{RMS}}$** for the NEMA 11 motor while preserving microstep linearity, keep `IHOLD_IRUN.IRUN` (`0x10` bits `12:8`) in the recommended **`16..31`** range (`Table 16`, p. 46) and scale via `GLOBAL_SCALER` (`0x0B` bits `7:0`, recommended **`> 128`**, p. 87).
> 4. **Enable Pin (`DRV_ENN` / Pin 16) Strapping (`TMC2240 Datasheet` p. 10, 16):** The TMC2240 `DRV_ENN` pin has an internal $100\,\text{k}\Omega$ pull-up to `VCC_IO` (outputs disabled/floating when unconnected). Connect Pin 16 either to its dedicated Morpho GPIO (`PC6`, `PC7`, `PD2`) or directly to `GND` on the carrier PCB (as implemented on `U2`, `U3`, and `U6` in `Hardware/XZ-Controller/XZ-Controller.kicad_sch`, using `CHOPCONF.TOFF = 0` over SPI for software enable/disable).

### E. TMC2240 Hardware & Power Protection (`TMC2240 Datasheet` p. 124-125)

> [!WARNING]
> **Critical Hardware Protection Rules:**
> 1. **Bulk Decoupling Capacitor:** Always install a low-ESR electrolytic capacitor ($\ge 100\,\mu\text{F}$, $35\text{V}$ or $50\text{V}$) directly across `VM` (`Pin 8`) and `GND` (`Pin 7`) within $\le 3\,\text{cm}$ of **each of the 3 driver modules** to absorb chopper current ripple.
> 2. **No Hot-Plugging:** **NEVER** connect or disconnect stepper motor leads while the 24V supply is energized.
> 3. **ESD / Frame Grounding:** Belt-driven axes (especially GT2 timing belts on the X-axis) and moving drag-chain cables to the Z-stage NEMA 11 can build up static charge; ground the stepper motor chassis/frame to system ground to prevent ESD resets.

---

## 4. AS5600 Magnetic Rotary Encoder Breakout Modules Wiring (X & Z Axes Only)

Only the **X-Axis (Motor 1)** and **Z-Axis (Motor 0)** are equipped with **AS5600** 12-bit magnetic rotary encoders for closed-loop position verification (the NEMA 11 end-effector rotational axis is open-loop). Because the AS5600 has a factory-fixed 7-bit I2C slave address of **`0x36`**, each of the two modules is routed to an independent hardware I2C peripheral on the STM32F446RE (`I2C1` and `I2C3`) via the **ST Morpho `CN10`/`CN7` headers**.

### A. Crucial 3.3V Mode Hardware Check on AS5600 Breakout Modules (`R1` Jumper)

> [!IMPORTANT]
> **3.3V Supply Strapping (`AS5600 Datasheet` Figure 13, p. 9):**
> The AS5600 IC has two supply pins: **`VDD5V`** (module `VCC` pin) and **`VDD3V3`** (module `3V3` pin). In **3.3V operation**, `VDD5V` and `VDD3V3` **must be tied together** to bypass the internal 5V$\to$3.3V LDO regulator:
> - **Check Resistor `R1` on your AS5600 Module:** Standard 7-pin AS5600 breakout modules have a $0\,\Omega$ resistor pad labeled **`R1`** that bridges `VCC` and `3V3`:
>   - **If `R1` ($0\,\Omega$) is installed on the module:** `VCC` and `3V3` are already tied together onboard. Connect Nucleo **+3V3** (`CN7 Pin 16`) to `VCC` (and **never** apply 5V while `R1` is installed).
>   - **If `R1` is open / unpopulated on the module:** Feeding $+3.3\text{V}$ into `VCC` alone will starve the internal LDO (which needs $\ge 4.5\text{V}$) and cause I2C dropouts. You must **either bridge `R1` with solder OR connect both the `VCC` and `3V3` pins on the module to Nucleo `+3V3` (`CN7 Pin 16`)**.
> - **Onboard Pull-Ups & Decoupling:** The AS5600 breakout module already includes onboard $100\text{ nF}$ / $1\,\mu\text{F}$ decoupling capacitors and $4.7\,\text{k}\Omega$ I2C pull-up resistors (`R2`, `R3`) tied to `VCC`.

### B. Encoder 1: X-Axis (Bus `I2C1`)
| Module Pin Silkscreen | AS5600 IC Pin | STM32 Morpho Header & Pin | Function & Notes |
| :---: | :---: | :--- | :--- |
| **VCC** | `VDD5V` (Pin 1) | Morpho **CN7 Pin 16** (**+3V3**) | 3.3V Logic Supply |
| **3V3** | `VDD3V3` (Pin 2)| Tie to **+3V3** (`CN7 Pin 16`) *(or leave open if `R1` $0\,\Omega$ is bridged onboard)* | Required in 3.3V mode (`Datasheet` Fig. 13) |
| **GND** | `GND` (Pin 4) | Morpho **CN7 Pin 19** / **CN10 Pin 9** (**GND**) | Common Ground |
| **SCL** | `SCL` (Pin 7) | Morpho **CN10 Pin 3** (**PB8**) | `I2C1_SCL` (onboard $4.7\,\text{k}\Omega$ pull-up) |
| **SDA** | `SDA` (Pin 6) | Morpho **CN10 Pin 5** (**PB9**) | `I2C1_SDA` (onboard $4.7\,\text{k}\Omega$ pull-up) |
| **DIR** | `DIR` (Pin 8) | Tie to **GND** *(or **+3V3** for CCW)* | `GND` = CW rotation increases angle; **never leave floating** |
| **OUT** | `OUT` (Pin 3) | *Leave Unconnected* | Analog / PWM output (unused in I2C mode) |
| **GPO / PGO / GPR** | `PGO` (Pin 5) | *Leave Unconnected* | OTP Program Option (internal pull-up; **do not ground**) |

### C. Encoder 2: Z-Axis (Bus `I2C3`)
| Module Pin Silkscreen | AS5600 IC Pin | STM32 Morpho Header & Pin | Function & Notes |
| :---: | :---: | :--- | :--- |
| **VCC** | `VDD5V` (Pin 1) | Morpho **CN7 Pin 16** (**+3V3**) | 3.3V Logic Supply |
| **3V3** | `VDD3V3` (Pin 2)| Tie to **+3V3** (`CN7 Pin 16`) *(or leave open if `R1` $0\,\Omega$ is bridged onboard)* | Required in 3.3V mode (`Datasheet` Fig. 13) |
| **GND** | `GND` (Pin 4) | Morpho **CN7 Pin 20** / **CN10 Pin 20** (**GND**) | Common Ground |
| **SCL** | `SCL` (Pin 7) | Morpho **CN10 Pin 23** (**PA8**) | `I2C3_SCL` (onboard $4.7\,\text{k}\Omega$ pull-up) |
| **SDA** | `SDA` (Pin 6) | Morpho **CN10 Pin 1** (**PC9**) | `I2C3_SDA` (onboard $4.7\,\text{k}\Omega$ pull-up) |
| **DIR** | `DIR` (Pin 8) | Tie to **GND** *(or **+3V3** for CCW)* | `GND` = CW rotation increases angle; **never leave floating** |
| **OUT** | `OUT` (Pin 3) | *Leave Unconnected* | Analog / PWM output (unused in I2C mode) |
| **GPO / PGO / GPR** | `PGO` (Pin 5) | *Leave Unconnected* | OTP Program Option (internal pull-up; **do not ground**) |

---

## 5. 5V 3-Wire Inductive Proximity Sensor Wiring (Morpho Header)

The 5V 3-wire inductive proximity sensor connects **entirely to the Morpho headers** using **`PC8`** (**CN10 Pin 2**) for the signal output, **`CN7 Pin 18`** for `+5V`, and **`CN7 Pin 19`** (or **`CN10 Pin 9`**) for `GND`.

### A. Sensor Pinout & Morpho Interconnect Table (IEC Standard 3-Wire Color Code)

| Wire Color | IEC Label | Sensor Function | Connects to STM32 Morpho Header | Alternate Morpho / Supply | Notes |
| :---: | :---: | :--- | :--- | :--- | :--- |
| **Brown** | `BN` | `+5V` Supply | Morpho **CN7 Pin 18** (`+5V`) | External 5V Buck Converter (`+5V`) | 5V DC power rail |
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

## 6. Stepper Motor Phase Wiring (4-Lead Bipolar — NEMA 17 & NEMA 11)

Both the **NEMA 17** motors (Z-Axis & X-Axis) and the **NEMA 11** motor (End-Effector Rotational Axis) are 4-lead bipolar steppers with two independent isolated coils: **Coil B** (StepStick Pins 3 & 4) and **Coil A** (StepStick Pins 5 & 6).

| StepStick Pin | BTT Label | MKS Label | TMC2240 IC Pin | Typical Wire Color (Standard) | Alternate Color Code |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **Pin 3** | `B1` (`2B`) | `1B` | `OUT1B` (Coil B1) | **Green** | Blue |
| **Pin 4** | `B2` (`2A`) | `1A` | `OUT2B` (Coil B2) | **Black** | Red |
| **Pin 5** | `A2` (`1A`) | `2A` | `OUT2A` (Coil A2) | **Red** | Green |
| **Pin 6** | `A1` (`1B`) | `2B` | `OUT1A` (Coil A1) | **Blue** | Black |

> [!TIP]
> **Identifying Stepper Coils with Multimeter:**
> - Set multimeter to resistance mode ($\Omega$).
> - Probe pairs of motor wires. Two wires showing continuity ($\approx 1.5 - 4\,\Omega$ for NEMA 17; $\approx 3 - 8\,\Omega$ for NEMA 11) belong to the **same coil** — connect one pair to **Pins 3 & 4** (`B1`/`B2`) and the other pair to **Pins 5 & 6** (`A2`/`A1`).
> - There must be infinite resistance (open circuit) between the two coils.
> - If a motor turns backwards during operation, invert direction in software (`GCONF.shaft`) or swap the two wires of **one coil only** (e.g. swap `B1` and `B2`).

---

## 7. Power & Ground Distribution Rules

```text
     +---------------------------------------------+          +-----------------------------------+
     |             24V DC Power Supply             |          |  Nucleo +5V (CN7-18) / 5V Buck    |
     +---------------------------------------------+          +-----------------------------------+
        (+) 24V                              (-) GND             (+) 5V                     (-) GND
           |                                    |                   |                          |
           +----------+----------+              |                   v                          |
           |          |          |              |                +------+                      |
           v          v          v              |                |  BN  |                      |
        +------+   +------+   +------+          |                |  5V  |                      |
        |  VM  |   |  VM  |   |  VM  |          |                | Prox |                      |
        |      |   |      |   |      |          |                |Sensor|                      |
        | TMC0 |   | TMC1 |   | TMC2 |          |                |  BU  |                      |
        | (Z)  |   | (X)  |   |(Rot.)|          |                +------+                      |
        | GND  |   | GND  |   | GND  |          |                   |                          |
        +------+   +------+   +------+          |                   |                          |
           |          |          |              |                   |                          |
           +-----+----+----------+--------------+-------------------+--------------------------+
                 |
                 v
              [Star Ground Splice Point] <------- Nucleo Morpho GND (CN7-8/19/20/22, CN10-9/20)
```

1. **Common Star Ground:** The negative terminal (`-`) of the 24V power supply, any external 5V buck converter `GND`, the Blue (`BU`) ground lead of the 5V proximity sensor, and the Morpho `GND` pins of the Nucleo board (`CN7 Pin 8, 19, 20, 22` and `CN10 Pin 9, 20`) **MUST** be solidly connected together at a single common ground reference.
2. **Decoupling Capacitors:** Keep lead lengths between the 24V electrolytic capacitors ($\ge 100\,\mu\text{F}$) and the TMC2240 `VM`/`GND` pins as short as possible ($\le 3\,\text{cm}$) on all three drivers (`TMC0`, `TMC1`, `TMC2`).
3. **Power Rail Current Limits (`UM1724` Section 7.5):**
   - **Nucleo `+3V3` Rail (Morpho `CN7 Pin 16`):** Supplied by the onboard `LD39050PU33R` LDO (500 mA max). Powers `VIO` on all three TMC2240 modules and `VCC`/`3V3` on both AS5600 modules ($\approx 25\text{–}30\,\text{mA}$ total).
   - **Nucleo `+5V` Rail (Morpho `CN7 Pin 18`):** When powered via ST-LINK USB (`JP5` on `U5V`), total board +5V current is limited to **300 mA max**, which easily powers the 5V proximity sensor ($\approx 10\text{–}20\,\text{mA}$).
4. **Motor Cable Routing:** Keep high-current motor lead wires (`B1`, `B2`, `A2`, `A1`) for all three axes physically separated from `SPI2`, `I2C1`/`I2C3`, and proximity sensor signal wires to prevent inductive noise coupling—especially along the Z-stage cable bundle carrying the NEMA 11 motor leads to the end-effector.
