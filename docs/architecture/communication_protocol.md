# Host PC ➔ STM32 Communication Protocol Specification

- **Subsystem:** Vision System (Host PC) to Motion Controller (existing implementation: STM32F446RE; physical target: SKR Octopus Pro V1.1 / STM32H723)
- **Physical Layer:** Full-Duplex UART via USB-CDC / Hardware Serial
- **Baud Rate:** 115200 bps (8-N-1: 8 data bits, no parity, 1 stop bit)

> [!IMPORTANT]
> This protocol is implemented by the existing NUCLEO-F446RE firmware. The SKR Octopus Pro V1.1 H723 firmware port has not been completed; its UART/USB interface and pins must be configured before connecting the host. Do not assume the Nucleo USART2 or ST-LINK VCOM wiring carries over.

---

## 1. Frame Structure Overview

The deployed command uses a fixed-length 18-byte binary frame with synchronization preambles and CRC16-CCITT validation. The diagnostic runtime is a cooperative foreground state machine with UART DMA and has no RTOS implementation. An RTOS and its queues may be evaluated in a future systems stage if more complex kinematics and real-time coordination warrant task scheduling; neither is a requirement of this build or protocol. USART2 is binary-only; human firmware logs use RTT.

```text
 +-------+-------+------+-----------+---------+-----------+-----------+-------------+------------------+---------------+-------------+
 | Byte0 | Byte1 | Byte2| Byte3-4   | Byte5   | Byte6-7   | Byte8-9   | Byte10-11   | Byte12-13        | Byte14-15     | Byte16-17   |
 | PRE_0 | PRE_1 | TYPE | OBJ_ID    | CLASS   | POS_X_MM  | POS_Y_MM  | ANGLE_DEG   | SERVO_CORR_DEG   | CRC16         | DELIM_CRLF  |
 | 0xAA  | 0x55  | uint8| uint16_LE | uint8   | int16_LE  | int16_LE  | int16_LE    | int16_LE         | uint16_LE     | 0x0D  0x0A  |
 +-------+-------+------+-----------+---------+-----------+-----------+-------------+------------------+---------------+-------------+
 Total: 18 Bytes
```

---

## 2. Field Definitions

| Byte Offset | Field Name | Type | Unit / Encoding | Description |
| :---: | :--- | :---: | :--- | :--- |
| **0** | `PREAMBLE_0` | `uint8_t` | Constant `0xAA` | Frame start identifier byte 1 |
| **1** | `PREAMBLE_1` | `uint8_t` | Constant `0x55` | Frame start identifier byte 2 |
| **2** | `MSG_TYPE` | `uint8_t` | Enum | `0x01`: Pick Command<br>`0x02`: Status / Heartbeat Request<br>`0x03`: Software Abort (not safety-rated)<br>`0x04`: Re-homing Command |
| **3 – 4** | `OBJECT_ID` | `uint16_t` | Little-Endian | Monotonically increasing tracked object identifier |
| **5** | `CLASS_ID` | `uint8_t` | Enum | `0x00`: Non-Capacitor / Unclassified<br>`0x01`: Elco Benar (Correct Polarity)<br>`0x02`: Elco Terbalik (Reversed / Defective Polarity) |
| **6 – 7** | `POS_X_MM` | `int16_t` | $0.1\ \text{mm}$ | Target $X$ position on conveyor ($1000 = 100.0\ \text{mm}$) |
| **8 – 9** | `POS_Y_MM` | `int16_t` | $0.1\ \text{mm}$ | Target $Y$ position along conveyor ($2500 = 250.0\ \text{mm}$) |
| **10 – 11** | `ANGLE_DEG` | `int16_t`| $0.1^\circ$ | Valid integers 0…3599; normalize after quantization. Signed Python packing has identical bits to unsigned values in this valid range. |
| **12 – 13** | `CORRECTION_DEG`| `int16_t`| $0.1^\circ$ | Target servo angular adjustment $[-180.0^\circ, +180.0^\circ]$ |
| **14 – 15** | `CRC16` | `uint16_t` | CCITT-FALSE | CRC across bytes **0…13 inclusive**, including AA55 |
| **16 – 17** | `DELIMITER` | `uint8_t[2]`| `0x0D, 0x0A` | Standard ASCII `\r\n` line termination |

---

## 3. CRC-16 Calculation & Command Semantics

- **Algorithm:** CRC-16/CCITT-FALSE
- **Polynomial:** $0x1021$ ($x^{16} + x^{12} + x^5 + 1$)
- **Initial Value:** $0xFFFF$
- **RefIn / RefOut:** `false` / `false` (MSB first)
- **XorOut:** $0x0000$
- **Command Coverage:** Bytes **`0…13` inclusive** (preamble `0xAA 0x55` included; stored little-endian in bytes `14…15`).
  - C call: `crc16_ccitt(frame, 14)`
  - Python call: `calculate_crc16(frame[:14])`

### Verification Test Vectors
- **Standard ASCII check:** `"123456789"` $\to$ `0x29B1`
- **Golden deployed command** (`OBJECT_ID=1`, `CLASS_ID=1`, `X=150.0 mm`, `Y=100.0 mm`, `ANGLE=0.0°`, `CORR=0.0°`):
  - Hex bytes: `AA5501010001DC05E803000000009A740D0A`
  - Computed CRC: `0x749A` (encoded little-endian as `9A 74`).
  - Legacy payload-only CRC (`0x82BB`, omitting `AA 55`) is **rejected**.

### Operational & Coordinate Rules
- **Supported message types:** Firmware accepts `0x01`, `0x02`, `0x03`, and `0x04`. Live `VisionSystem` (`uart_protocol.py`) emits `0x01` (Pick) and `0x02` (Heartbeat); the `motion-diag` bench CLI additionally emits `0x03` (Software Abort) and `0x04` (Re-homing).
- **Coordinate meaning:** `POS_Y_MM` is **conveyor longitudinal $Y$**, **not manipulator vertical $Z$**. Safe $Z$ height and pick depth are fixed, qualified firmware configuration parameters (`motion_config.c`).
- **Diagnostic profile behavior:** In `motion_diagnostics`, `POS_Y_MM`, `CLASS_ID`, `ANGLE_DEG`, and `CORRECTION_DEG` are validated and echoed in completion records, not used to actuate vacuum or servo hardware.

### Reference C Implementation (STM32 Firmware)
```c
uint16_t crc16_ccitt(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ 0x1021);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}
```

### Reference Python Implementation (Vision System)
```python
def calculate_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc
```

---

## 4. STM32 Reception & Ring-Buffer State Machine

To prevent CPU overhead, the STM32 USART2 peripheral runs in **DMA Circular Mode**. An incoming stream parser processes bytes via an interrupt-free state machine:

```text
 [WAIT_PREAMBLE_1: 0xAA] ➔ [WAIT_PREAMBLE_2: 0x55] ➔ [RECEIVE_PAYLOAD: 12 Bytes]
                                                                  |
 [DELIMITER_CHECK: \r\n]  [VALIDATE_CRC16]  [RECEIVE_CRC: 2 Bytes]
           |
           +-- VALID: Admit to bounded foreground motion queue / priority control handler
           +-- INVALID: Increment CRC error counter, discard frame.
```

---

## 5. Human-Readable ASCII Debug Formatter (Not a Runtime Fallback)

Python's optional `format_ascii_debug` creates the following display string:
```text
OBJ:102 CLS:1 X:120.5 Y:350.2 ANG:88.4 SRV:-1.6\r\n
```
It is for local display only. The diagnostic firmware has no ASCII parser/fallback.
The vision runtime sends the original 18-byte `bytes` object without text conversion;
do not send display strings or mix binary replies with console text.

## 6. STM32 ➔ PC Binary Diagnostic Replies (`v1`)

See [`motion_diagnostics_design.md` §3](motion_diagnostics_design.md#3-stm32--pc-wire-protocol-version-1) for the authoritative `v1` reply specification:

| Record Type | ID | Payload | Total Length | Summary |
| :--- | :---: | :---: | :---: | :--- |
| `COMMAND_RESULT` | `0x81` | 312 B | **356 B** | Common 40 B header + three 104 B phase blocks (`X_MOVE`, `Z_DOWN`, `Z_UP`) + CRC16 + CRLF |
| `HOME_RESULT` | `0x82` | 80 B | **124 B** | Common 40 B header + two 40 B axis homing blocks (`X`, `Z`) + CRC16 + CRLF |
| `STATUS` | `0x83` | 72 B | **116 B** | Common 40 B header + 72 B status/counter snapshot + CRC16 + CRLF |

- **Distinct Framing & CRC Coverage:** Replies start with preamble `0xD3 0x7E` (distinct from command `0xAA 0x55`) and a 38-byte little-endian header (`bytes 2…39`). Unlike commands, reply CRC16-CCITT covers **bytes `2…(39 + payload_len)` inclusive** (preamble excluded).
- **Stream Decoding:** Always decode incoming bytes with `motion_diagnostics.frames.StreamDecoder` over arbitrary read chunks—**never** `readline()`, because `0x0D 0x0A` and preamble byte pairs can legally appear inside binary payloads.
- **Host Correlation & Safety:** Host `session_id` and `host_send_seq` are correlated locally using exact command echo fields and firmware `command_seq` ordinals (they are not embedded in the 18-byte command). Software abort (`0x03`) does not replace a physical emergency stop or mechanical Z anti-drop support.
