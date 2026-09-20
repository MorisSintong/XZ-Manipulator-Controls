# TMC2240 Hardware & Wiring Guide

This guide details the hardware wiring between the STM32F446RE Nucleo-64 board, the MKS TMC2240 module, and bipolar stepper motors.

## 1. Single Motor Pinout (Demo Setup)

| STM32 Nucleo-64 Pin | MKS TMC2240 Pin | Signal | Description |
|:-------------------:|:---------------:|:------:|:------------|
| **PB10** | 14 | **SCK** | SPI2 Serial Clock |
| **PC1** | 15 | **MOSI** (DI) | SPI2 Master Out Slave In |
| **PC2** | 12 | **MISO** (DO) | SPI2 Master In Slave Out |
| **PA4** | 13 | **CS** | Chip Select (Active Low) |
| **PA0** | 10 | **STEP** | Step pulse input |
| **PA1** | 9 | **DIR** | Direction input |
| Application-selected GPIO | 16 | **ENN** | Active low; hold high during setup and on a fault |
| — | 8 | **VMOT** | Motor power supply (12V – 36V DC) |
| — | 2 | **VDD** | Logic power supply (3.3V from Nucleo) |
| — | 1, 7 | **GND** | Ground (common with STM32 GND) |

**Hardware qualification is pending.** Confirm the pin numbering and ratings
against the actual module. Use an external inactive-high bias on ENN during
reset and a suitable independent motor-disable mechanism. Do not tie ENN to
GND when relying on the example's GPIO fault shutdown. Transport initialization
does not itself guarantee that physical outputs are disabled.

### Stepper Motor Wiring (4-Lead Bipolar)

| MKS Module Pin | Stepper Motor Wire | Function |
|:--------------:|:------------------:|:---------|
| **1A** (Pin 4) | Phase A+ | Coil 1 Positive |
| **1B** (Pin 3) | Phase A- | Coil 1 Negative |
| **2A** (Pin 5) | Phase B+ | Coil 2 Positive |
| **2B** (Pin 6) | Phase B- | Coil 2 Negative |

> **Note**: Pin 1A/1B belong to coil A, and 2A/2B belong to coil B.
> If the motor vibrates or hums without rotating smoothly, swap the two wires of **one** coil (e.g., swap 1A and 1B).

---

## 2. Power Supply Recommendations
- **Motor Voltage (VMOT)**: 12V to 36V DC. Use a minimum 100 µF low-ESR electrolytic capacitor near the driver power pins to absorb inductive voltage spikes.
- **Logic Voltage (VDD)**: Connect directly to 3.3V from the STM32 Nucleo board.
- Never connect or disconnect the stepper motor while VMOT is energized. Doing so can generate inductive back-EMF spikes that destroy the driver MOSFETs.

---

## 3. Dual Motor & Dual Encoder Wiring
For running two motors and two magnetic encoders (AS5600) on a single STM32F446RE, see [Dual_Motor_and_Encoder_Config.md](Dual_Motor_and_Encoder_Config.md).
