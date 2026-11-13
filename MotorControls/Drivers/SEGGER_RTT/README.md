# SEGGER RTT debug driver

This directory contains a small, fixed-buffer SEGGER Real-Time Transfer (RTT)
implementation used for firmware diagnostics. RTT carries debug text and
optional host keystrokes through the active J-Link debug connection; it is
separate from the firmware's application UART protocol.

The implementation exposes one target-to-host **up** channel and one
host-to-target **down** channel, both named `Terminal` at buffer index `0`.
It uses statically allocated RAM buffers and does not allocate from the heap.

## Files and integration

| File | Purpose |
| --- | --- |
| `SEGGER_RTT.h` | Buffer configuration, mode constants, control-block types, and public API |
| `SEGGER_RTT.c` | Control block, ring buffers, output, and input polling |
| `SEGGER_RTT_printf.c` | `SEGGER_RTT_printf`, backed by `vsnprintf` |

Add `SEGGER_RTT.c` to the firmware build. Add `SEGGER_RTT_printf.c` only if
the application calls `SEGGER_RTT_printf`. Add this directory to the compiler
include path. The driver has no STM32 HAL dependency. The repository's
[`MotionFirmware` CMake target](../../MotionFirmware/CMakeLists.txt) builds both
source files, and the [`motion_diagnostics` console](../../MotionFirmware/Apps/motion_diagnostics/Src/console.c)
uses RTT channel 0 for human-readable logs.

Default storage is 4096 bytes for the up buffer and 64 bytes for the down
buffer (`BUFFER_SIZE_UP` and `BUFFER_SIZE_DOWN`). These macros can be overridden
at compile time. Ring buffers reserve one byte to distinguish full from empty,
so usable capacities are 4095 and 63 bytes respectively at the defaults.

## API

Call `SEGGER_RTT_Init()` before using the interface. The control block is
statically initialized; `Init` fills it only if its ID is empty and does not
reset an already initialized buffer.

| Function | Behavior |
| --- | --- |
| `SEGGER_RTT_Write(0, data, length)` | Enqueues bytes for the host and returns the number accepted. Only buffer index 0 is supported. A zero length or unsupported index returns 0. |
| `SEGGER_RTT_WriteString(0, text)` | Writes the NUL-terminated string without its NUL byte; returns the number accepted. |
| `SEGGER_RTT_printf(0, format, ...)` | Formats to a local 256-byte array, attempts an RTT write, and returns the `vsnprintf` result—not the number actually accepted by RTT. See the formatting limitation below. |
| `SEGGER_RTT_HasKey()` | Returns nonzero when at least one host-to-target byte is queued; does not consume it. |
| `SEGGER_RTT_GetKey()` | Consumes and returns one byte as an `unsigned char`-range integer, or `-1` if empty. |

Example:

```c
#include "SEGGER_RTT.h"

void debug_console_init(void)
{
    static const char message[] = "firmware ready\r\n";

    SEGGER_RTT_Init();
    const unsigned accepted = SEGGER_RTT_Write(
        0U, message, (unsigned)(sizeof(message) - 1U));
    if (accepted != sizeof(message) - 1U) {
        /* Record or handle dropped diagnostic output. */
    }
}

void poll_debug_input(void)
{
    if (SEGGER_RTT_HasKey()) {
        const int key = SEGGER_RTT_GetKey();
        if (key >= 0) {
            /* Handle one received byte; this API does not parse lines. */
        }
    }
}
```

Use a valid, non-null source pointer whenever `length` is nonzero. Serialize
writers to a given ring: the implementation provides no lock or multi-producer
coordination. Poll input from the application context; the input API is not an
interrupt-driven command parser.

## Buffer-full behavior

Both buffers default to `SEGGER_RTT_MODE_NO_BLOCK_SKIP`.

| Mode | Current implementation behavior |
| --- | --- |
| `SEGGER_RTT_MODE_NO_BLOCK_SKIP` | If the complete write will not fit, accepts none of it and returns 0. |
| `SEGGER_RTT_MODE_NO_BLOCK_TRIM` | Accepts as many bytes as fit and returns that count. |
| `SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL` | Also accepts only the currently available bytes; it does **not** wait or block. |

Check the returned count from `SEGGER_RTT_Write` and `SEGGER_RTT_WriteString`.
The default skip mode can drop an entire message when the ring lacks room;
applications should count or otherwise handle rejected bytes rather than
assuming that a call was delivered.

`SEGGER_RTT_printf` has an additional limitation: keep the formatted text to at
most 255 characters so it fits with the terminator in its 256-byte local
buffer. The implementation passes the `vsnprintf` required length to
`SEGGER_RTT_Write` without capping it to the buffer capacity. A 256-character
result is truncated to 255 characters plus a NUL byte, which is then sent; a
result longer than 256 characters can read beyond the local buffer. The
function also ignores the write count, so its return value does not indicate
delivery. Use bounded formatting and check `SEGGER_RTT_Write` directly where
output size or delivery matters.

## Connect with J-Link

1. Build and flash the firmware containing the RTT sources. Keep the target
   running and connected to the J-Link probe.
2. Start SEGGER J-Link RTT Viewer for the same target/core and attach to its
   RTT control block (`SEGGER RTT`).
3. Select up channel `0` (`Terminal`) to view target logs. If the application
   polls input, send bytes through down channel `0` (`Terminal`).

RTT uses the debugger connection, not a UART COM port. It is intended for
instrumentation and interactive diagnostics; it does not replace the
application's UART transport or provide a safety-rated control path.
