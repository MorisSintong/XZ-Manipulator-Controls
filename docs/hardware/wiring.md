# Complete Wiring & Hardware Interconnect Specification
## STM32F446RE Nucleo-64 ➔ Dual TMC2240 Drivers & Dual AS5600 Encoders

**Project:** XZ-Manipulator-Controls  
**Target Board:** STM32F446RE Nucleo-64  
**Actuators:** 2× NEMA 17 Stepper Motors driven by MKS TMC2240 v1.0 (SPI Mode)  
**Sensors:** 2× AS5600 12-bit Magnetic Absolute Rotary Encoders (Dual I2C)  
**Telemetry:** Dual Broadcast on USART2 (115200 baud, `COM9`) and SEGGER RTT Terminal 0  

---

## 1. System Architecture Diagram

```text
                             +-----------------------------------+
                             |     STM32F446RE Nucleo-64         |
                             +-----------------------------------+
                                    |                     |
                 [SPI2 Bus]         |                     |         [I2C Buses]
             PB13: SCK              |                     |      I2C1 (PB8/PB9): Enc 1 (X)
             PB14: MISO             |                     |      I2C3 (PA8/PC9): Enc 2 (Z)
             PB15: MOSI             |                     |
                  +-----------------+                     +-----------------+
                  |                                                         |
                  v                                                         v
         +------------------+   +------------------+       +------------------+   +------------------+
         |  TMC2240 (Z-Axis)|   |  TMC2240 (X-Axis)|       | AS5600 Enc 1 (X) |   | AS5600 Enc 2 (Z) |
         |  (Motor 0)       |   |  (Motor 1)       |       | Addr: 0x36       |   | Addr: 0x36       |
         |  CS:   PC0       |   |  CS:   PC3       |       | Bus:  I2C1       |   | Bus:  I2C3       |
         |  STEP: PC1       |   |  STEP: PC4       |       +------------------+   +------------------+
         |  DIR:  PC2       |   |  DIR:  PC5       |
         |  ENN:  PC6       |   |  ENN:  PC7       |
         +------------------+   +------------------+
                  |                      |
                  v                      v
            [NEMA 17 (Z)]          [NEMA 17 (X)]
```

---

## 2. Master Pinout Table (STM32 Nucleo-64)

Both **Arduino Header Pin** (female headers) and **Morpho Pin** (double-row male headers) are specified below:

| STM32 Pin | Arduino / Morpho Header | Peripheral Function | Connected Module | Module Pin / Net | Description & Notes |
| :--- | :--- | :---: | :--- | :--- | :--- |
| **PB13** | Morpho **CN10 Pin 30** | `SPI2_SCK` (AF5) | **Both TMC2240** | Pin 14 (`SCK`) | Shared SPI Clock (2.625 MHz, SPI Mode 3) |
| **PB14** | Morpho **CN10 Pin 28** | `SPI2_MISO` (AF5) | **Both TMC2240** | Pin 12 (`SDO` / `MISO`) | Shared SPI Data Input to MCU |
| **PB15** | Morpho **CN10 Pin 26** | `SPI2_MOSI` (AF5) | **Both TMC2240** | Pin 15 (`SDI` / `MOSI`) | Shared SPI Data Output from MCU |
| **PC0** | Arduino **A5** / CN7 Pin 38 | `GPIO_Output` | **TMC2240 (Z / Motor 0)** | Pin 13 (`CS`) | Chip Select 0 (Active LOW, pull-up) |
| **PC1** | Arduino **A4** / CN7 Pin 36 | `GPIO_Output` | **TMC2240 (Z / Motor 0)** | Pin 10 (`STEP`) | Step pulse output for Z-axis |
| **PC2** | Morpho **CN7 Pin 34** | `GPIO_Output` | **TMC2240 (Z / Motor 0)** | Pin 9 (`DIR`) | Direction control for Z-axis |
| **PC6** | Morpho **CN10 Pin 4** | `GPIO_Output` | **TMC2240 (Z / Motor 0)** | Pin 16 (`ENN`) | Enable 0 (Active LOW; HIGH = disabled) |
| **PC3** | Morpho **CN7 Pin 37** | `GPIO_Output` | **TMC2240 (X / Motor 1)** | Pin 13 (`CS`) | Chip Select 1 (Active LOW, pull-up) |
| **PC4** | Morpho **CN10 Pin 34** | `GPIO_Output` | **TMC2240 (X / Motor 1)** | Pin 10 (`STEP`) | Step pulse output for X-axis |
| **PC5** | Morpho **CN10 Pin 6** | `GPIO_Output` | **TMC2240 (X / Motor 1)** | Pin 9 (`DIR`) | Direction control for X-axis |
| **PC7** | Arduino **D9** / CN10 Pin 19 | `GPIO_Output` | **TMC2240 (X / Motor 1)** | Pin 16 (`ENN`) | Enable 1 (Active LOW; HIGH = disabled) |
| **PB8** | Arduino **D15** / CN10 Pin 3 | `I2C1_SCL` (AF4) | **AS5600 Enc 1 (X-Axis)** | `SCL` | I2C Clock (100–400 kHz, ~4.7k pull-up) |
| **PB9** | Arduino **D14** / CN10 Pin 5 | `I2C1_SDA` (AF4) | **AS5600 Enc 1 (X-Axis)** | `SDA` | I2C Data (100–400 kHz, ~4.7k pull-up) |
| **PA8** | Arduino **D7** / CN10 Pin 23 | `I2C3_SCL` (AF4) | **AS5600 Enc 2 (Z-Axis)** | `SCL` | I2C Clock (100–400 kHz, ~4.7k pull-up) |
| **PC9** | Morpho **CN10 Pin 1** | `I2C3_SDA` (AF4) | **AS5600 Enc 2 (Z-Axis)** | `SDA` | I2C Data (100–400 kHz, ~4.7k pull-up) |
| **PC13** | Onboard **Blue Button B1** | `GPIO_EXTI13` | **User Interface** | — | Short press: start (home + cycle) / stop; hold 1.5 s: drivers off |
| **PA5** | Arduino **D13** / Green LD2 | `GPIO_Output` | **Status LED** | — | ON = cycling, 4 Hz = homing, 10 Hz = fault (see MotionFirmware README) |
| **PA2** | CN10 Pin 35 (ST-Link VCOM) | `USART2_TX` (AF7) | **CDC Serial / COM9** | RX | Telemetry TX @ 115200 baud (to PC) |
| **PA3** | CN10 Pin 37 (ST-Link VCOM) | `USART2_RX` (AF7) | **CDC Serial / COM9** | TX | Telemetry RX @ 115200 baud (from PC) |
| **PA13** | CN10 Pin 13 (SWDIO) | `SYS_JTMS-SWDIO` | **ST-Link / J-Link OB** | SWDIO | Debug & SEGGER RTT channel |
| **PA14** | CN10 Pin 15 (SWCLK) | `SYS_JTCK-SWCLK` | **ST-Link / J-Link OB** | SWCLK | Debug Clock |
| **+3V3** | Arduino **3V3** (CN6 Pin 4) | Logic Supply (+3.3V) | **All Drivers & Encoders**| `VDD` / `VIO` | Logic voltage from Nucleo LDO |
| **GND** | Arduino **GND** / Morpho GND | Power / Logic Ground| **All Modules & 24V PSU**| `GND` | Common System Ground |

---

## 3. TMC2240 Stepper Driver Modules Wiring

### A. Module Pinout (MKS TMC2240 v1.0 standard step-stick form factor)

```text
                   +------------------+
              ENN -| 16            1  |- GND
         SDI(MOSI)-| 15            2  |- VDD_IO (3.3V)
              SCK -| 14            3  |- 1B (Phase A-)
               CS -| 13            4  |- 1A (Phase A+)
        SDO(MISO) -| 12            5  |- 2A (Phase B+)
               NC -| 11            6  |- 2B (Phase B-)
             STEP -| 10            7  |- GND
              DIR -| 9             8  |- VMOT (12V - 36V)
                   +------------------+
```

### B. Driver 0: Z-Axis Motor
| TMC2240 Pin | Label | Connects to STM32 / System | Wire Type / Recommendation |
| :---: | :--- | :--- | :--- |
| **1** | `GND` | Common Ground (Nucleo `GND` + 24V PSU `-`) | Heavy gauge wire to Star Ground |
| **2** | `VDD_IO` | Nucleo **+3V3** (Arduino 3V3 / CN6 Pin 4) | Logic Power (3.3V only) |
| **3** | `1B` | Stepper Coil Phase A- (Green) | Twisted pair with 1A |
| **4** | `1A` | Stepper Coil Phase A+ (Black) | Twisted pair with 1B |
| **5** | `2A` | Stepper Coil Phase B+ (Red) | Twisted pair with 2B |
| **6** | `2B` | Stepper Coil Phase B- (Blue) | Twisted pair with 2A |
| **7** | `GND` | Common Ground (Bridge to Pin 1) | Heavy gauge ground |
| **8** | `VMOT` | **+24V DC** Power Supply | Equip with 100 µF 35V/50V cap to GND |
| **9** | `DIR` | STM32 **PC2** (CN7 Pin 34) | Signal Jumper |
| **10** | `STEP` | STM32 **PC1** (Arduino A4 / CN7 Pin 36) | Signal Jumper |
| **11** | `NC` | *Not Connected* | — |
| **12** | `SDO` (`MISO`)| STM32 **PB14** (CN10 Pin 28) | Shared SPI2 MISO |
| **13** | `CS` | STM32 **PC0** (Arduino A5 / CN7 Pin 38) | Dedicated CS0 (Motor 0) |
| **14** | `SCK` | STM32 **PB13** (CN10 Pin 30) | Shared SPI2 SCK |
| **15** | `SDI` (`MOSI`)| STM32 **PB15** (CN10 Pin 26) | Shared SPI2 MOSI |
| **16** | `ENN` | STM32 **PC6** (CN10 Pin 4) | Active LOW enable |

---

### C. Driver 1: X-Axis Motor
| TMC2240 Pin | Label | Connects to STM32 / System | Wire Type / Recommendation |
| :---: | :--- | :--- | :--- |
| **1** | `GND` | Common Ground (Nucleo `GND` + 24V PSU `-`) | Heavy gauge wire to Star Ground |
| **2** | `VDD_IO` | Nucleo **+3V3** (Arduino 3V3 / CN6 Pin 4) | Logic Power (3.3V only) |
| **3** | `1B` | Stepper Coil Phase A- | Twisted pair with 1A |
| **4** | `1A` | Stepper Coil Phase A+ | Twisted pair with 1B |
| **5** | `2A` | Stepper Coil Phase B+ | Twisted pair with 2B |
| **6** | `2B` | Stepper Coil Phase B- | Twisted pair with 2A |
| **7** | `GND` | Common Ground (Bridge to Pin 1) | Heavy gauge ground |
| **8** | `VMOT` | **+24V DC** Power Supply | Equip with 100 µF 35V/50V cap to GND |
| **9** | `DIR` | STM32 **PC5** (CN10 Pin 6) | Signal Jumper |
| **10** | `STEP` | STM32 **PC4** (CN10 Pin 34) | Signal Jumper |
| **11** | `NC` | *Not Connected* | — |
| **12** | `SDO` (`MISO`)| STM32 **PB14** (CN10 Pin 28) | Shared SPI2 MISO |
| **13** | `CS` | STM32 **PC3** (CN7 Pin 37) | Dedicated CS1 (Motor 1) |
| **14** | `SCK` | STM32 **PB13** (CN10 Pin 30) | Shared SPI2 SCK |
| **15** | `SDI` (`MOSI`)| STM32 **PB15** (CN10 Pin 26) | Shared SPI2 MOSI |
| **16** | `ENN` | STM32 **PC7** (Arduino D9 / CN10 Pin 19) | Active LOW enable |

> [!WARNING]
> **Power Supply Protection:**
> 1. Always install an electrolytic capacitor ($\ge 100\,\mu\text{F}$, $35\text{V}$ or $50\text{V}$, low ESR) directly across the `VMOT` and `GND` pins of each TMC2240 module.
> 2. **NEVER** plug or unplug a stepper motor while the 24V supply is powered on. Inductive kickback will permanently destroy the driver output stage.

---

## 4. AS5600 Magnetic Rotary Encoders Wiring

The AS5600 has a factory-fixed 7-bit I2C address of **`0x36`**. To avoid needing an external I2C multiplexer chip, we route each encoder to an independent hardware I2C bus:

### Encoder 1: X-Axis (Bus `I2C1`)
| AS5600 Pin | Nucleo Header & Pin | Function & Notes |
| :---: | :--- | :--- |
| **VCC** | Nucleo **+3V3** (Arduino 3V3 / CN6 Pin 4) | 3.3V Logic Supply |
| **GND** | Nucleo **GND** (Arduino GND / CN6 Pin 6) | Common Ground |
| **SCL** | Arduino **D15** / CN10 Pin 3 (**PB8**) | `I2C1_SCL` (~4.7k pull-up to 3.3V)* |
| **SDA** | Arduino **D14** / CN10 Pin 5 (**PB9**) | `I2C1_SDA` (~4.7k pull-up to 3.3V)* |
| **DIR** | Tie to **GND** | Clockwise rotation = Increasing counts |
| **GPO / OUT** | *Leave Unconnected* | Unused in digital I2C mode |

### Encoder 2: Z-Axis (Bus `I2C3`)
| AS5600 Pin | Nucleo Header & Pin | Function & Notes |
| :---: | :--- | :--- |
| **VCC** | Nucleo **+3V3** (Arduino 3V3 / CN6 Pin 4) | 3.3V Logic Supply |
| **GND** | Nucleo **GND** (Arduino GND / CN6 Pin 7) | Common Ground |
| **SCL** | Arduino **D7** / CN10 Pin 23 (**PA8**) | `I2C3_SCL` (~4.7k pull-up to 3.3V)* |
| **SDA** | Morpho **CN10 Pin 1** (**PC9**) | `I2C3_SDA` (~4.7k pull-up to 3.3V)* |
| **DIR** | Tie to **GND** | Clockwise rotation = Increasing counts |
| **GPO / OUT** | *Leave Unconnected* | Unused in digital I2C mode |

*\*Note: Most commercial AS5600 breakout boards (e.g. CJMCU-5600) already have onboard $4.7\,\text{k}\Omega$ pull-up resistors on both SCL and SDA lines to VCC. If your breakout does not have pull-ups, add external $4.7\,\text{k}\Omega$ resistors between SCL➔3.3V and SDA➔3.3V.*

---

## 5. Stepper Motor Phase Wiring (4-Lead Bipolar)

Standard bipolar stepper motors have two independent isolated coils: **Coil A** and **Coil B**.

| TMC2240 Pin | Driver Phase | Typical Wire Color (Standard) | Alternate Color Code |
| :---: | :---: | :---: | :---: |
| **Pin 4** | `1A` | **Black** | Red |
| **Pin 3** | `1B` | **Green** | Blue |
| **Pin 5** | `2A` | **Red** | Green |
| **Pin 6** | `2B` | **Blue** | Black |

> [!TIP]
> **Identifying Stepper Coils with Multimeter:**
> - Set multimeter to resistance mode ($\Omega$).
> - Probe pairs of wires. The two wires showing $\approx 1.5 - 4\,\Omega$ are part of the **same coil**.
> - There must be infinite resistance (open circuit) between Coil A and Coil B wires.
> - If the motor turns backwards during operation, invert the motor direction pin in software or swap the two wires of **one coil only** (e.g. swap `1A` and `1B`).

---

## 6. Power & Ground Distribution Rules

```text
     +-----------------------------------+
     |        24V DC Power Supply        |
     +-----------------------------------+
        (+) 24V                    (-) GND
           |                          |
           +----------+---------------+
           |          |               |
           v          v               |
        +------+   +------+           |
        | VMOT |   | VMOT |           |
        |      |   |      |           |
        | TMC0 |   | TMC1 |           |
        |      |   |      |           |
        | GND  |   | GND  |           |
        +------+   +------+           |
           |          |               |
           +-----+----+               |
                 |                    |
                 v                    v
              [Star Ground Splice Point] <------- Nucleo GND
```

1. **Common Star Ground:** The negative terminal (`-`) of the 24V power supply and the `GND` pins of the Nucleo board **MUST** be solidly connected together at a single common star-ground terminal.
2. **Decoupling Capacitors:** Keep lead lengths between the 24V electrolytic capacitors and the TMC2240 `VMOT`/`GND` pins as short as possible ($\le 3\,\text{cm}$).
3. **Strict Logic Isolation:** Logic power (`VDD_IO` / `VCC`) is strictly **3.3V** supplied by the STM32 Nucleo board. Do **not** connect 5V or 24V to logic pins.
4. **Motor Cable Routing:** Keep high-current motor lead wires (1A, 1B, 2A, 2B) physically separated from SPI and I2C sensor wires to prevent inductive noise coupling.
