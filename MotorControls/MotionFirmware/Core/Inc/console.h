#ifndef MOTION_CONSOLE_H
#define MOTION_CONSOLE_H
#include <stddef.h>
#include <stdint.h>
void console_init(void);
void console_write(const char *, size_t);
void console_printf(const char *, ...) __attribute__((format(printf, 1, 2)));
uint32_t console_dropped_bytes(void);
#endif
