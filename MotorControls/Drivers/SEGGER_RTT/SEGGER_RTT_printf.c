#include "SEGGER_RTT.h"
#include <stdio.h>
#include <stdarg.h>

int SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...) {
  char buf[256];
  va_list args;
  va_start(args, sFormat);
  int r = vsnprintf(buf, sizeof(buf), sFormat, args);
  va_end(args);
  if (r > 0) {
    SEGGER_RTT_Write(BufferIndex, buf, (unsigned)r);
  }
  return r;
}
