# Legacy Dual Motor & Dual Encoder Configuration — NUCLEO-F446RE

This pinout is an older configuration for two BIGTREETECH (BTT) / MKS TMC2240 v1.0 modules, two AS5600 encoders, and a proximity sensor on a NUCLEO-F446RE (`MB1136`). It is **not** the current SKR wiring.

> [!IMPORTANT]
> The current physical controller is the BIGTREETECH SKR Octopus Pro V1.1 (STM32H723ZET6). Use the [current system wiring](../../../../docs/hardware/wiring.md) and the active H723 firmware in [MotorControls/MotionFirmware](../../../MotionFirmware/README.md); the Nucleo pin assignments below do not apply.

## TMC2240 (Stepper Drivers)

Both TMC2240 modules share the same SPI bus (`SPI2`) and use independent Chip Select (`CS`), `STEP`, `DIR`, and `ENN` pins on Port C.

### Shared SPI Bus (`SPI2`) — Contiguous on Port B
| Signal | STM32 Pin | Morpho Header | Connects to |
| :--- | :--- | :--- | :--- |
| **SCK** | `PB13` | `CN10 Pin 30` | Pin 14 (`CLK` / `SCK`) on **both** modules |
| **MISO** | `PB14` | `CN10 Pin 28` | Pin 12 (`MISO` / `SDO`) on **both** modules |
| **MOSI** | `PB15` | `CN10 Pin 26` | Pin 15 (`MOSI` / `SDI`) on **both** modules |

### Driver 0 (Z-Axis / Motor 0) — Contiguous on Port C
| Signal | STM32 Pin | Header Pin | Connects to |
| :--- | :--- | :--- | :--- |
| **CS 0** | `PC0` | Arduino `A5` / `CN7 Pin 38` | Pin 13 (`CS`) |
| **STEP 0** | `PC1` | Arduino `A4` / `CN7 Pin 36` | Pin 10 (`STEP`) |
| **DIR 0** | `PC2` | Morpho `CN7 Pin 35` *(odd row)* | Pin 9 (`DIR`) |
| **ENN 0** | `PC6` | Morpho `CN10 Pin 4` | Pin 16 (`EN` / `ENN`, Active LOW) |

### Driver 1 (X-Axis / Motor 1) — Contiguous on Port C
| Signal | STM32 Pin | Header Pin | Connects to |
| :--- | :--- | :--- | :--- |
| **CS 1** | `PC3` | Morpho `CN7 Pin 37` *(odd row)* | Pin 13 (`CS`) |
| **STEP 1** | `PC4` | Morpho `CN10 Pin 34` | Pin 10 (`STEP`) |
| **DIR 1** | `PC5` | Morpho `CN10 Pin 6` | Pin 9 (`DIR`) |
| **ENN 1** | `PC7` | Arduino `D9` / `CN10 Pin 19` | Pin 16 (`EN` / `ENN`, Active LOW) |

### Common Connections (Both Modules)
| Signal | Connects to |
| :--- | :--- |
| **VM / VMOT** | 24V DC Motor Power Supply (equip with $\ge 100\,\mu\text{F}$ low-ESR electrolytic cap per module) |
| **VIO / VDD_IO** | 3.3V from Nucleo (`CN7 Pin 16` / `CN6 Pin 4`) |
| **GND** | Common System Star Ground (Pins 1 & 7) |

---

## AS5600 (Magnetic Encoder Breakout Modules)

The AS5600 has a **fixed I2C address (`0x36`)**. To read from two sensors simultaneously without an external I2C multiplexer chip, we map them to two independent hardware I2C peripherals: **`I2C1`** (`PB8`/`PB9`) and **`I2C3`** (`PA8`/`PC9`).

> **3.3V Operation Note (`AS5600 Datasheet` Fig. 13):** Ensure the $0\,\Omega$ resistor **`R1`** on each AS5600 breakout module is populated (or tie the module's `VCC` and `3V3` pins together to Nucleo `+3V3`).

### Encoder 1 (`I2C1` — X-Axis)
| Signal | STM32 Pin | Header Pin | Notes |
| :--- | :--- | :--- | :--- |
| **SCL** | `PB8` | Arduino `D15` / `CN10 Pin 3` | Onboard $4.7\,\text{k}\Omega$ pull-up to 3.3V |
| **SDA** | `PB9` | Arduino `D14` / `CN10 Pin 5` | Onboard $4.7\,\text{k}\Omega$ pull-up to 3.3V |
| **VCC & 3V3** | `+3V3` | `CN7 Pin 16` / `CN6 Pin 4` | Tied together via onboard `R1` ($0\,\Omega$) in 3.3V mode |
| **GND** | `GND` | `CN6 Pin 6` / `CN7 Pin 19` | From Nucleo |
| **DIR** | `GND` | Tie to `GND` | `GND` = CW increasing; `3.3V` = CCW increasing |

### Encoder 2 (`I2C3` — Z-Axis)
| Signal | STM32 Pin | Header Pin | Notes |
| :--- | :--- | :--- | :--- |
| **SCL** | `PA8` | Arduino `D7` / `CN10 Pin 23` | Onboard $4.7\,\text{k}\Omega$ pull-up to 3.3V |
| **SDA** | `PC9` | Morpho `CN10 Pin 1` | Onboard $4.7\,\text{k}\Omega$ pull-up to 3.3V |
| **VCC & 3V3** | `+3V3` | `CN7 Pin 16` / `CN6 Pin 4` | Tied together via onboard `R1` ($0\,\Omega$) in 3.3V mode |
| **GND** | `GND` | `CN6 Pin 7` / `CN7 Pin 20` | From Nucleo |
| **DIR** | `GND` | Tie to `GND` | `GND` = CW increasing; `3.3V` = CCW increasing |

---

## 5V 3-Wire Inductive Proximity Sensor (Morpho Header)

| Wire Color | Signal | Nucleo Morpho Pin | Notes |
| :--- | :--- | :--- | :--- |
| **Brown (`BN`)** | `+5V` | `CN7 Pin 18` (`+5V`) | 5V supply rail |
| **Blue (`BU`)** | `GND` | `CN7 Pin 19` (`GND`) | Common Ground |
| **Black (`BK`)** | `OUT` | `CN10 Pin 2` (`PC8`) | `GPIO_Input` / `GPIO_EXTI8` (5V-tolerant `FT` pin) |

---

## Example STM32 HAL Initialization

```c
// Define the 2 SPI configurations using the CS pins on Port C
static const TMC2240_SPIConfig_t tmc_spi_cfgs[] = {
    // Motor 0 (Z-Axis) - SPI2, CS on PC0
    { &hspi2, GPIOC, GPIO_PIN_0, 100U },

    // Motor 1 (X-Axis) - SPI2, CS on PC3
    { &hspi2, GPIOC, GPIO_PIN_3, 100U }
};

TMC2240Status status = tmc2240_hal_init(tmc_spi_cfgs, 2U);
if (status != TMC2240_OK) {
    Error_Handler();
}
```
