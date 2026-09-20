# Dual Motor Configuration — STM32F446RE

This document provides an optimized, **physically contiguous** pinout configuration for running **two MKS TMC2240** stepper driver modules and **two AS5600** magnetic encoders on a single STM32F446RE Nucleo-64 board. 

The pins have been carefully selected using matching STM32 Alternate Functions so that related signals are grouped on the same ports (Port B and Port C), making your breadboard or PCB wiring extremely clean.

## TMC2240 (Stepper Drivers)

Both TMC2240 modules share the same SPI bus (SPI2) but use independent Chip Select (CS), STEP, and DIR pins. 

### Shared SPI Bus (SPI2) — Contiguous on Port B
| Signal | STM32 Pin | Connects to |
| :--- | :--- | :--- |
| **SCK** | PB13 | MKS Pin 14 (SCK) on **both** modules |
| **MISO** | PB14 | MKS Pin 12 (SDO) on **both** modules |
| **MOSI** | PB15 | MKS Pin 15 (SDI) on **both** modules |

### Module 1 (Motor 0) — Contiguous on Port C
| Signal | STM32 Pin | Connects to |
| :--- | :--- | :--- |
| **CS 0** | PC0 | MKS Pin 13 (CS) |
| **STEP 0** | PC1 | MKS Pin 10 (STEP) |
| **DIR 0** | PC2 | MKS Pin 9 (DIR) |

### Module 2 (Motor 1) — Contiguous on Port C
| Signal | STM32 Pin | Connects to |
| :--- | :--- | :--- |
| **CS 1** | PC3 | MKS Pin 13 (CS) |
| **STEP 1** | PC4 | MKS Pin 10 (STEP) |
| **DIR 1** | PC5 | MKS Pin 9 (DIR) |

### Common Connections (Both Modules)
| Signal | Connects to |
| :--- | :--- |
| **ENN** | Application-controlled GPIO for each driver; hold inactive-high during setup and on faults |
| **VMOT** | 12V – 24V Motor Power Supply |
| **VDD** | 3.3V from Nucleo |
| **GND** | System Ground |

This is a proposed integration pinout, not a bench-qualified configuration.
Use an external inactive-high ENN bias during MCU reset and verify the actual
module pinout. Serialize all access to the shared SPI bus, including complete
two-frame reads and read-modify-write helpers. The AS5600 portion is outside
the TMC driver software audit.

---

## AS5600 (Magnetic Encoders)

The AS5600 has a **fixed I2C address (`0x36`)**. To read from two sensors simultaneously without an external I2C multiplexer chip, we map them to two hardware I2C peripherals. We are using `I2C1` and `I2C2`, placing all 4 pins continuously on Port B.

### Encoder 1 (I2C1) — Contiguous on Port B
| Signal | STM32 Pin | Notes |
| :--- | :--- | :--- |
| **SCL** | PB8 | Requires external pull-up resistor (~4.7kΩ to 3.3V)* |
| **SDA** | PB9 | Requires external pull-up resistor (~4.7kΩ to 3.3V)* |
| **VDD** | 3.3V | From Nucleo |
| **GND** | GND | From Nucleo |
| **DIR** | GND or 3.3V | Tie to GND for CW = increasing, 3.3V for CCW |

### Encoder 2 (I2C3) 
| Signal | STM32 Pin | Notes |
| :--- | :--- | :--- |
| **SCL** | PA8 | Requires external pull-up resistor (~4.7kΩ to 3.3V)* |
| **SDA** | PC9 | Requires external pull-up resistor (~4.7kΩ to 3.3V)* |
| **VDD** | 3.3V | From Nucleo |
| **GND** | GND | From Nucleo |
| **DIR** | GND or 3.3V | Tie to GND for CW = increasing, 3.3V for CCW |

*\*Note: Most AS5600 breakout boards already include pull-up resistors on the I2C lines. Check your specific module before adding external resistors.*

---

## Example STM32 HAL Initialization

When configuring this setup in your code, your initialization block for the TMC2240 array would look like this:

```c
// Define the 2 SPI configurations using the new CS pins
static const TMC2240_SPIConfig_t tmc_spi_cfgs[] = {
    // Motor 0 (IC 0) - SPI2, CS on PC0
    { &hspi2, GPIOC, GPIO_PIN_0, 100 }, 
    
    // Motor 1 (IC 1) - SPI2, CS on PC3
    { &hspi2, GPIOC, GPIO_PIN_3, 100 } 
};

// Transport initialization only; no motor defaults or activation.
TMC2240Status status = tmc2240_hal_init(tmc_spi_cfgs, 2U);
if (status != TMC2240_OK) {
    Error_Handler();
}
```

Keep both ENN inputs inactive and use the explicit checked configuration and
activation sequence in `driver\README.md`. The snippet assumes the application's
error handler inhibits both motors; it does not make the shared bus thread-safe.
