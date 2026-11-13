# MotionFirmware — Z-Bounce & Vision UART Emulation Profile (`z_bounce_emulation`)

Single-axis Z StallGuard4 bounce demonstration and hybrid binary/ASCII `VisionUart` protocol responder (`MotionFirmware_ZBounceEmulation.*`) for **STM32H723ZET6 (`BIGTREETECH SKR Octopus Pro V1.1`)**.

> [!NOTE]
> For production dual-axis motion diagnostics and binary-only telemetry with AS5600 encoder verification, use the default [`motion_diagnostics`](../motion_diagnostics/README.md) profile. For interactive two-end axis calibration, use [`stallguard_cli`](../stallguard_cli/README.md).

---

## 1. Purpose & Key Differences

| Aspect | `z_bounce_emulation` | `motion_diagnostics` (default) |
| :--- | :--- | :--- |
| **Axes active** | Single Z axis (`MOTOR1`, Driver 1) | Dual XZ (`MOTOR0` X + `MOTOR1` Z) |
| **Step generation** | Foreground DWT cycle-counter busy-wait (`PF`/`PG` GPIO) | 1 MHz timer-compare ISRs (`TIM2` Z / `TIM5` X) |
| **Encoders** | Not used | Dual AS5600 (`I2C1` + `I2C4`) |
| **USART2 RX / TX** | Interrupt `RXNE` byte ring buffer + text ACK replies | Circular `DMA1_Stream0` RX + binary `DMA1_Stream1` TX |
| **Button (`B1`, `PG15`)** | Polled active-low input (disables `EXTI15_10_IRQn` on init) | Interrupt-driven software abort |
| **Protocol** | 18-byte binary `0x01`/`0x02` (`VisionUart`) + ASCII `PING`/`STATUS`/`K...,S...,R...` | Binary-only `0x01..0x04` commands and `356/124/116`-byte `0xD3 0x7E` replies |

---

## 2. Hardware Mapping (SKR Octopus Pro V1.1 / STM32H723)

- **Z-Axis TMC2240 (`MOTOR1` / Driver 1):** `CS0=PD11`, `STEP0=PG0`, `DIR0=PG1`, `ENN0=PF15`, `SPI1` (`PA5` SCK, `PA6` MISO, `PA7` MOSI, Mode 3, 3.125 MHz)
- **Telemetry:** Dual output on **SEGGER RTT Terminal 0** and **`USART2` (`PD5` TX / `PD6` RX on `J26`, `115200 8N1`)**
- **User Button `B1`:** `PG15` (`STOP7` `J34`, polled active-low with pull-up) to start/stop Z bouncing
- **Status LED `LD2`:** `PD15` (`FAN5` indicator LED `D27`, ON while bouncing)

---

## 3. Supported `VisionUart` Commands (`vision_uart_protocol.c`)

- **18-byte binary frames** (`0xAA 0x55`, CRC16-CCITT across bytes `0..13`, `\r\n` tail):
  - `0x01` Detection/Pick packet → replies `ACK:OBJ=<id>,CLS=<cls>,X=<x>,Y=<y>,ANG=<ang>,SRV=<srv>,CRC=OK\r\n` (stores latest `VisionTarget_t`)
  - `0x02` Heartbeat packet → replies `ACK:HEARTBEAT,SEQ=<seq>\r\n`
  - Corrupted CRC → replies `NACK:CRC_ERR\r\n`; bad framing/tail → `NACK:FRAME_ERR\r\n`
- **Legacy ASCII commands** (newline-terminated):
  - `PING` → `ACK:PONG\r\n`
  - `STATUS` → `ACK:STATUS,READY\r\n`
  - `K<cls>,S<angle>,R<corr>` → `ACK:LEGACY,CLS=<cls>,CORR=<corr>\r\n`

---

## 4. Build & Flash

Run from `MotorControls\MotionFirmware\`:

```powershell
# Build (also emitted side-by-side under build\Debug\):
cmake --preset Debug-ZBounce
cmake --build --preset Debug-ZBounce

# Flash via SEGGER J-Link:
.\flash.cmd zbounce
```

Offline UART protocol tests targeting this protocol contract live in [`MotorControls/tests/emulation/`](../../../tests/emulation/run_tests.py) (use `--mode mock`, since Renode's bundled `stm32f4.repl` targets STM32F4 rather than STM32H723).
