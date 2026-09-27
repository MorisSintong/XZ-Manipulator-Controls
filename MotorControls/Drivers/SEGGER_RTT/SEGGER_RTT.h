#ifndef SEGGER_RTT_H
#define SEGGER_RTT_H

#include <stdint.h>
#include <stddef.h>
#include <stdarg.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SEGGER_RTT_MODE_NO_BLOCK_SKIP         (0)
#define SEGGER_RTT_MODE_NO_BLOCK_TRIM         (1)
#define SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL    (2)
#define SEGGER_RTT_MODE_MASK                  (3)

#define RTT_CTRL_RESET                "\033[0m"
#define RTT_CTRL_TEXT_BLACK           "\033[2;30m"
#define RTT_CTRL_TEXT_RED             "\033[2;31m"
#define RTT_CTRL_TEXT_GREEN           "\033[2;32m"
#define RTT_CTRL_TEXT_YELLOW          "\033[2;33m"
#define RTT_CTRL_TEXT_BLUE            "\033[2;34m"
#define RTT_CTRL_TEXT_MAGENTA         "\033[2;35m"
#define RTT_CTRL_TEXT_CYAN            "\033[2;36m"
#define RTT_CTRL_TEXT_WHITE           "\033[2;37m"

#define RTT_CTRL_TEXT_BRIGHT_BLACK    "\033[1;30m"
#define RTT_CTRL_TEXT_BRIGHT_RED      "\033[1;31m"
#define RTT_CTRL_TEXT_BRIGHT_GREEN    "\033[1;32m"
#define RTT_CTRL_TEXT_BRIGHT_YELLOW   "\033[1;33m"
#define RTT_CTRL_TEXT_BRIGHT_BLUE     "\033[1;34m"
#define RTT_CTRL_TEXT_BRIGHT_MAGENTA  "\033[1;35m"
#define RTT_CTRL_TEXT_BRIGHT_CYAN     "\033[1;36m"
#define RTT_CTRL_TEXT_BRIGHT_WHITE    "\033[1;37m"

#define BUFFER_SIZE_UP    (1024)
#define BUFFER_SIZE_DOWN  (16)

typedef struct {
  const char*       sName;
  char*             pBuffer;
  unsigned          SizeOfBuffer;
  volatile unsigned WrOff;
  volatile unsigned RdOff;
  unsigned          Flags;
} SEGGER_RTT_BUFFER_UP;

typedef struct {
  const char*       sName;
  char*             pBuffer;
  unsigned          SizeOfBuffer;
  volatile unsigned WrOff;
  volatile unsigned RdOff;
  unsigned          Flags;
} SEGGER_RTT_BUFFER_DOWN;

typedef struct {
  char                    acID[16];
  int                     MaxNumUpBuffers;
  int                     MaxNumDownBuffers;
  SEGGER_RTT_BUFFER_UP    aUp[1];
  SEGGER_RTT_BUFFER_DOWN  aDown[1];
} SEGGER_RTT_CB;

extern SEGGER_RTT_CB _SEGGER_RTT;

void     SEGGER_RTT_Init(void);
unsigned SEGGER_RTT_Write(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes);
unsigned SEGGER_RTT_WriteString(unsigned BufferIndex, const char* s);
int      SEGGER_RTT_printf(unsigned BufferIndex, const char * sFormat, ...);
int      SEGGER_RTT_GetKey(void);
int      SEGGER_RTT_HasKey(void);

#ifdef __cplusplus
}
#endif

#endif /* SEGGER_RTT_H */
