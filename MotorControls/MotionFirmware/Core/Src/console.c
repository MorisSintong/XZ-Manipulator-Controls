#include "console.h"
#include "SEGGER_RTT.h"
#include <stdarg.h>
#include <stdio.h>

static uint32_t dropped;
void console_init(void)
{
    SEGGER_RTT_Init();
    _SEGGER_RTT.aUp[0].Flags = SEGGER_RTT_MODE_NO_BLOCK_SKIP;
    dropped = 0U;
}
void console_write(const char *s, size_t n)
{
    const unsigned written = SEGGER_RTT_Write(0U, s, (unsigned)n);
    dropped += (uint32_t)n - written;
}
void console_printf(const char *format, ...)
{
    char line[256];
    va_list args;
    va_start(args, format);
    const int size = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (size > 0) {
        console_write(line, (size_t)size < sizeof(line) ? (size_t)size : sizeof(line) - 1U);
    }
}
uint32_t console_dropped_bytes(void) { return dropped; }
int __io_putchar(int ch)
{
    const char byte = (char)ch;
    console_write(&byte, 1U);
    return ch;
}
