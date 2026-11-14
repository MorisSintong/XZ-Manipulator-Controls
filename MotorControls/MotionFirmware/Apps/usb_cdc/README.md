# Portable TinyUSB CDC Module (`Apps/usb_cdc/`)

Portable, non-blocking **USB-C Virtual COM Port (`CDC ACM`)** transport module for **BIGTREETECH SKR Octopus Pro V1.1 (`STM32H723ZET6`)**, backed by vendored **TinyUSB v0.18.0** ([`MotorControls/Drivers/tinyusb/`](../../../Drivers/tinyusb/UPSTREAM_VERSION.txt)).

This module is self-contained and designed so any `MotionFirmware` profile (`motion_diagnostics`, `stallguard_cli`, `z_bounce_emulation`, or future motion controllers) can drop in USB-C RX/TX with **zero changes to `MotionFirmware.ioc`**, **zero disruption to `PLL1` / `TIM2` / `TIM5`**, and **no external USB-UART adapter on `J26` (`USART2`)**.

---

## 1. Key STM32H723 & Octopus Pro V1.1 Design Decisions

1. **Isolated 48 MHz USB Clock (`HSI48` + `CRS`)**
   - `usb_cdc_port_init()` enables the internal **`HSI48`** oscillator (`48 MHz`), selects `RCC_USBCLKSOURCE_HSI48`, and enables the **Clock Recovery System (`CRS`)** synced to USB 1 kHz Start-of-Frame (`RCC_CRS_SYNC_SOURCE_USB1`).
   - It explicitly leaves `HSE` (`25 MHz`) and `PLL1` (`400 MHz SYSCLK`, `200 MHz HCLK`, `100 MHz APB1/APB2`) untouched so **`TIM2`/`TIM5` 1 MHz step timing never changes**.
2. **Hardware VBUS Sensing Bypass (`PA9` Conflict)**
   - On the SKR Octopus Pro V1.1 schematic, `PA9` is routed to `USART1_TX` on `J70` (`TFT`), **not** to USB-C 5 V VBUS.
   - `tusb_config.h` sets `CFG_TUD_VBUS_SENSING = 0`, and `usb_cdc_port_init()` clears `USB_OTG_GCCFG_VBDEN` and forces `USB_OTG_GOTGCTL_BVALOEN | USB_OTG_GOTGCTL_BVALOVAL` (both before and after `tusb_rhport_init()`).
   - Result: USB enumerates reliably whether powered by USB-C (`J68` fitted) or 24 V `MAIN POWER` (`J68` removed for data-only USB-C).
3. **`USB1_OTG_HS` in Internal Full-Speed PHY Mode (`RHPort 0`)**
   - Pins: `PA11` (`USB_OTG_HS_DM`) and `PA12` (`USB_OTG_HS_DP`) in `GPIO_AF10_OTG1_HS`.
   - `STM32H723xx` provides a single USB controller (`USB1_OTG_HS @ 0x40040000`) and omits `USB2_OTG_FS`. In TinyUSB `0.18.0` (`dcd_dwc2` / `dwc2_stm32.h`), `RHPort 0` automatically aliases to `USB1_OTG_HS` and `OTG_HS_IRQn`.
   - Uses DWC2 slave FIFO mode (`CFG_TUD_DWC2_SLAVE_ENABLE = 1`, `CFG_TUD_DWC2_DMA_ENABLE = 0`) to avoid STM32H7 D-cache / DTCM DMA coherency hazards.
4. **Stepper ISR Priority Protection**
   - `OTG_HS_IRQn` is configured at **NVIC priority `5`** (`USB_CDC_IRQ_PRIORITY`), so it can **never preempt `TIM2`/`TIM5` step ISRs at priority `0`**.
   - Calls to `usb_cdc_port_poll()`, `usb_cdc_port_read*()`, and `usb_cdc_port_write()` never block waiting for USB enumeration or `DTR`.
5. **Windows Host `DTR` Compatibility**
   - Many Windows clients (including `.NET System.IO.Ports.SerialPort` / PowerShell) open COM ports with `DtrEnable = false`.
   - `usb_cdc_port_write()` checks `usb_cdc_port_is_mounted()` (enumerated by host) rather than `usb_cdc_port_is_connected()` (DTR high). Use `usb_cdc_port_is_connected()` only if an application explicitly wants to detect when an interactive terminal asserts DTR to trigger a welcome banner.

---

## 2. File Layout

| File | Role | Included by `add_usb_cdc_module()`? |
| :--- | :--- | :---: |
| [`usb_cdc_module.cmake`](usb_cdc_module.cmake) | CMake helper exporting `add_usb_cdc_module(<target>)` | Included in root `CMakeLists.txt` |
| [`Inc/usb_cdc_port.h`](Inc/usb_cdc_port.h) | Portable C API (`init`, `poll`, `read`, `write`, `write_all`, `printf`, `stats`, `irq_handler`) | Yes |
| [`Inc/tusb_config.h`](Inc/tusb_config.h) | TinyUSB 0.18.0 STM32H723 configuration (`RX=512 B`, `TX=1024 B`, `EP=64 B`) | Yes |
| [`Src/usb_cdc_port.c`](Src/usb_cdc_port.c) | Clock (`HSI48+CRS`), `PA11`/`PA12` GPIO, VBUS override, `OTG_HS_IRQHandler`, non-blocking CDC API | Yes |
| [`Src/usb_descriptors.c`](Src/usb_descriptors.c) | CDC ACM USB descriptors (`VID=0xCafe`, `PID=0x4001`, STM32 96-bit UID serial string) | Yes |
| [`Src/usb_cdc_app.c`](Src/usb_cdc_app.c) | Standalone `fw_app.h` verification profile (LED heartbeat, RTT log, CLI + `BINTEST`) | **No** (standalone test target only) |

---

## 3. Portable C API Reference ([`Inc/usb_cdc_port.h`](Inc/usb_cdc_port.h))

| Function | Description | Blocking? |
| :--- | :--- | :---: |
| `bool usb_cdc_port_init(void)` | Enables `HSI48` + `CRS`, USB voltage detector, `PA11`/`PA12`, VBUS override, `OTG_HS_IRQn` (prio 5), and TinyUSB `rhport 0`. Call once inside `fw_app_init()`. | Startup only |
| `void usb_cdc_port_poll(void)` | Pumps `tud_task_ext(0, false)`. Call on every `fw_app_poll()` iteration. Safe when USB is unplugged. | No |
| `bool usb_cdc_port_is_mounted(void)` | Returns `true` once USB enumeration completes. | No |
| `bool usb_cdc_port_is_connected(void)` | Returns `true` when mounted **and** host asserts `DTR`. | No |
| `uint32_t usb_cdc_port_available(void)` | Returns unread bytes waiting in the 512 B TinyUSB RX FIFO. | No |
| `uint32_t usb_cdc_port_read(uint8_t *buf, uint32_t max_len)` | Reads up to `max_len` bytes from RX FIFO; updates `rx_bytes`. | No |
| `int usb_cdc_port_read_byte(void)` | Reads 1 byte `[0..255]`, or returns `-1` if empty (drop-in for `console_getc()`). | No |
| `uint32_t usb_cdc_port_write(const uint8_t *data, uint32_t len)` | Non-blocking enqueue into 1024 B TX FIFO + flush. Excess bytes are dropped & counted in `tx_dropped_bytes`. | No |
| `bool usb_cdc_port_write_all(const uint8_t *data, uint32_t len, uint32_t timeout_ms)` | Pumps USB until all `len` bytes fit or `timeout_ms` expires (useful for multi-packet binary frames in a foreground poll). | Bounded |
| `int usb_cdc_port_printf(const char *fmt, ...)` | Formats up to 255 chars and calls `usb_cdc_port_write()`. | No |
| `void usb_cdc_port_get_stats(usb_cdc_stats_t *out)` | Snapshots RX/TX/dropped counters, mount/umount counts, and host line coding. | No |
| `void usb_cdc_port_irq_handler(void)` | Forwards to `tusb_int_handler(0, true)`. Automatically wired to `OTG_HS_IRQHandler` unless `USB_CDC_PORT_EXTERNAL_IRQ_HANDLER` is defined. | ISR |

---

## 4. How to Drop Into Any Motion Firmware Target

### Step 1: Add the module in [`CMakeLists.txt`](../../CMakeLists.txt)

[`CMakeLists.txt`](../../CMakeLists.txt) already includes [`usb_cdc_module.cmake`](usb_cdc_module.cmake). To add USB-C CDC to another target or `app_profile`, call:

```cmake
add_usb_cdc_module(${target_name})
```

This adds **only** the portable TinyUSB + `usb_cdc_port` + `usb_descriptors` sources and headers (`usb_cdc_app.c` is excluded, so your profile keeps its own `fw_app.h` implementation).

> [!NOTE]
> `OTG_HS_IRQHandler` is already defined inside [`Src/usb_cdc_port.c`](Src/usb_cdc_port.c). Keep `Core/Src/stm32h7xx_it.c` untouched unless you specifically want to manage `OTG_HS_IRQHandler` yourself (in which case define `USB_CDC_PORT_EXTERNAL_IRQ_HANDLER` and call `usb_cdc_port_irq_handler()`).

### Step 2: Wire it into your profile

#### Pattern A — Drop-in for `motion_diagnostics` (356/124/116-byte binary frames)

In [`Apps/motion_diagnostics/Src/motion_board.c`](../motion_diagnostics/Src/motion_board.c):

1. **Init (`motion_board_init`)**: call `(void)usb_cdc_port_init();`.
2. **TX (`uart_transport_io_t.start_tx`)**: check FIFO room without blocking the motion loop, enqueue the whole binary frame atomically, and complete the slot immediately so `uart_transport` keeps its queue consistent:
   ```c
   static bool usb_tx_start(void *context, const uint8_t *bytes, uint16_t size) {
     (void)context;
     if (!usb_cdc_port_is_mounted() || tud_cdc_write_available() < size) {
       return false; /* uart_transport leaves slot queued and retries cleanly */
     }
     const uint32_t queued = usb_cdc_port_write(bytes, size);
     if (queued != size) {
       return false;
     }
     uart_transport_tx_complete(&uart);
     return true;
   }
   ```
   *(Because `CFG_TUD_CDC_TX_BUFSIZE = 1024` > `356` bytes, up to two full 356-byte evidence records fit in the TinyUSB TX FIFO simultaneously without waiting.)*
3. **RX + Poll (`motion_board_poll`)**: call `usb_cdc_port_poll()` and feed incoming bytes straight into `uart.rx[]` using `uart_transport_rx_publish()`:
   ```c
   static void usb_publish_rx(void *context) {
     (void)context;
     usb_cdc_port_poll();
     uint8_t tmp[64];
     const uint32_t free_space = UART_RX_SIZE - (uart.producer - uart.consumer);
     const uint32_t budget = free_space < sizeof(tmp) ? free_space : sizeof(tmp);
     const uint32_t n = usb_cdc_port_read(tmp, budget);
     for (uint32_t i = 0U; i < n; ++i) {
       uart.rx[(uart.producer + i) % UART_RX_SIZE] = tmp[i];
     }
     if (n > 0U) {
       uart_transport_rx_publish(&uart, uart.producer + n);
     }
   }
   ```
   With `usb_tx_start` + `usb_publish_rx`, the entire `vision_parser` (`0x01..0x04`), `uart_transport` backpressure, and host Python [`tools/diag`](../../tools/diag/README.md) / `VisionSystem` stack work unchanged over USB-C!

#### Pattern B — Drop-in for `stallguard_cli` (`console.c`)

In [`Apps/stallguard_cli/Src/console.c`](../stallguard_cli/Src/console.c):
- `console_init()`: call `(void)usb_cdc_port_init();` alongside `SEGGER_RTT_Init()`.
- `console_write(s, n)`: call `(void)usb_cdc_port_write((const uint8_t *)s, (uint32_t)n);` alongside `SEGGER_RTT_Write(0U, s, (unsigned)n)`.
- `console_getc()`: check `int usb_ch = usb_cdc_port_read_byte(); if (usb_ch >= 0) return usb_ch;` alongside RTT/UART.
- `console_poll()`: call `usb_cdc_port_poll();` on every main-loop iteration.

#### Pattern C — Drop-in for `z_bounce_emulation` (`z_bounce_app.c`)

In [`Apps/z_bounce_emulation/Src/z_bounce_app.c`](../z_bounce_emulation/Src/z_bounce_app.c):
- `fw_app_init()`: call `(void)usb_cdc_port_init();`.
- `fw_app_poll()`: call `usb_cdc_port_poll();`, and feed `usb_cdc_port_read_byte()` into `VisionUart_RxByteCallback((uint8_t)ch)`.
- `log_printf()`: call `(void)usb_cdc_port_write((const uint8_t *)buf, (uint32_t)len);` alongside `SEGGER_RTT_Write(...)`.

---

## 5. Standalone Verification Profile (`usb_cdc_app.c`)

The standalone test profile builds alongside all other profiles as **`MotionFirmware_UsbCdc.*`** (`-DFIRMWARE_APP=usb_cdc` via the `Debug-UsbCdc` preset).

### Build & Flash

From `MotorControls\MotionFirmware\`:

```powershell
# Build all targets (includes MotionFirmware_UsbCdc.hex) or select Debug-UsbCdc as default:
cmake --preset Debug
cmake --build --preset Debug

# Flash MotionFirmware_UsbCdc.hex via SEGGER J-Link SWD:
.\flash.cmd usb_cdc
```

### Status LED (`LD2` on `PD15` / `FAN5` LED `D27`)

| Blink Period | Meaning |
| :--- | :--- |
| **800 ms** (slow) | Not mounted / USB host not enumerated |
| **350 ms** (medium) | USB mounted (`usb_cdc_port_is_mounted() == true`), DTR low |
| **150 ms** (fast) | USB mounted and terminal DTR high (`usb_cdc_port_is_connected() == true`) |

### Serial Test Commands (Any Baud Rate, e.g., `115200 8N1`)

Plug a USB-C cable into the SKR Octopus Pro V1.1 (`USB_VID=0xCAFE`, `USB_PID=0x4001`, Windows `usbser.sys` auto-loads):

| Command | Response | Purpose |
| :--- | :--- | :--- |
| `HELP` | Command list | Smoke check |
| `PING` | `PONG <uptime_ms>` | Latency / RX-TX roundtrip |
| `INFO` | `MCU=STM32H723ZET6 SYSCLK=400000000Hz HCLK=200000000Hz UID=... USB_MOUNTED=1 DTR=...` | Verifies clocks (`400 MHz`) & 96-bit UID serial |
| `STAT` | `RX=... TX=... DROPPED=... MOUNTS=... UMOUNTS=... BAUD=...` | Verifies zero dropped bytes |
| `ECHO <text>` | `ECHO <text>` | Payload echo |
| `LED <on\|off\|blink>` | `OK LED=<...>` | Physical `PD15` (`D27`) LED test |
| `BINTEST [len]` | Sends `<len>` raw bytes (`1..512`, default `356`): `AA 55 <len_le16> 04 05 ...` | Verifies multi-packet (`> 64 B`) binary framing |

---

## 6. Windows Troubleshooting Notes

1. **Duplicate `COMx` Port Name Collision with Bluetooth (`BthModem`)**
   - On some Windows machines, an idle `Standard Serial over Bluetooth link (COM5)` may occupy `COM5` in `HKLM\HARDWARE\DEVICEMAP\SERIALCOMM` while PnP also assigns `COM5` to `USB Serial Device (COM5)` (`VID_CAFE&PID_4001`). Opening `COM5` in a terminal then hangs on the Bluetooth modem instead of talking to TinyUSB.
   - **Fix:** Open **Device Manager → Ports (COM & LPT) → `USB Serial Device (COM5)` → Properties → Port Settings → Advanced…** and reassign the **COM Port Number** to an unused port (e.g., `COM11`), then replug USB-C.
2. **No Physical `PA9` VBUS Wire Required**
   - When switching from USB-only bench power (`J68` fitted) to 24 V `MAIN POWER` (`J68` removed per BTT's manual), Windows still enumerates the board over `PA11`/`PA12` because firmware bypasses `PA9` VBUS sensing.
