#include "SEGGER_RTT.h"
#include <string.h>

static char _acUpBuffer[BUFFER_SIZE_UP];
static char _acDownBuffer[BUFFER_SIZE_DOWN];

// Initialized control block matching Segger J-Link discovery
SEGGER_RTT_CB _SEGGER_RTT = {
  "SEGGER RTT",
  1,
  1,
  {
    {
      "Terminal",
      &_acUpBuffer[0],
      sizeof(_acUpBuffer),
      0,
      0,
      SEGGER_RTT_MODE_NO_BLOCK_SKIP
    }
  },
  {
    {
      "Terminal",
      &_acDownBuffer[0],
      sizeof(_acDownBuffer),
      0,
      0,
      SEGGER_RTT_MODE_NO_BLOCK_SKIP
    }
  }
};

void SEGGER_RTT_Init(void) {
  // Ensure the control block string is present
  if (_SEGGER_RTT.acID[0] == '\0') {
    strcpy(_SEGGER_RTT.acID, "SEGGER RTT");
    _SEGGER_RTT.MaxNumUpBuffers = 1;
    _SEGGER_RTT.MaxNumDownBuffers = 1;
    _SEGGER_RTT.aUp[0].sName = "Terminal";
    _SEGGER_RTT.aUp[0].pBuffer = _acUpBuffer;
    _SEGGER_RTT.aUp[0].SizeOfBuffer = sizeof(_acUpBuffer);
    _SEGGER_RTT.aUp[0].WrOff = 0;
    _SEGGER_RTT.aUp[0].RdOff = 0;
    _SEGGER_RTT.aUp[0].Flags = SEGGER_RTT_MODE_NO_BLOCK_SKIP;
    
    _SEGGER_RTT.aDown[0].sName = "Terminal";
    _SEGGER_RTT.aDown[0].pBuffer = _acDownBuffer;
    _SEGGER_RTT.aDown[0].SizeOfBuffer = sizeof(_acDownBuffer);
    _SEGGER_RTT.aDown[0].WrOff = 0;
    _SEGGER_RTT.aDown[0].RdOff = 0;
    _SEGGER_RTT.aDown[0].Flags = SEGGER_RTT_MODE_NO_BLOCK_SKIP;
  }
}

unsigned SEGGER_RTT_Write(unsigned BufferIndex, const void* pBuffer, unsigned NumBytes) {
  unsigned NumBytesToWrite = NumBytes;
  unsigned WrOff;
  unsigned RdOff;
  const char* pSrc = (const char*)pBuffer;
  SEGGER_RTT_BUFFER_UP* pRing;

  if (BufferIndex >= 1 || NumBytes == 0) {
    return 0;
  }
  pRing = &_SEGGER_RTT.aUp[BufferIndex];
  WrOff = pRing->WrOff;
  RdOff = pRing->RdOff;

  // Calculate available space
  unsigned Avail;
  if (RdOff <= WrOff) {
    Avail = pRing->SizeOfBuffer - 1 - WrOff + RdOff;
  } else {
    Avail = RdOff - WrOff - 1;
  }

  if (NumBytesToWrite > Avail) {
    if (pRing->Flags == SEGGER_RTT_MODE_NO_BLOCK_SKIP) {
      return 0; // Skip if buffer full
    }
    NumBytesToWrite = Avail;
  }

  // Copy bytes
  while (NumBytesToWrite > 0) {
    unsigned NumContig;
    if (WrOff >= RdOff) {
      NumContig = pRing->SizeOfBuffer - WrOff;
      if (RdOff == 0) {
        NumContig--; // Leave 1 byte empty
      }
    } else {
      NumContig = RdOff - WrOff - 1;
    }
    if (NumContig > NumBytesToWrite) {
      NumContig = NumBytesToWrite;
    }
    if (NumContig > 0) {
      memcpy(pRing->pBuffer + WrOff, pSrc, NumContig);
      pSrc += NumContig;
      NumBytesToWrite -= NumContig;
      WrOff += NumContig;
      if (WrOff >= pRing->SizeOfBuffer) {
        WrOff = 0;
      }
    } else {
      break;
    }
  }
  pRing->WrOff = WrOff;
  return NumBytes - NumBytesToWrite;
}

unsigned SEGGER_RTT_WriteString(unsigned BufferIndex, const char* s) {
  return SEGGER_RTT_Write(BufferIndex, s, (unsigned)strlen(s));
}

int SEGGER_RTT_HasKey(void) {
  SEGGER_RTT_BUFFER_DOWN* pRing = &_SEGGER_RTT.aDown[0];
  return (pRing->RdOff != pRing->WrOff);
}

int SEGGER_RTT_GetKey(void) {
  SEGGER_RTT_BUFFER_DOWN* pRing = &_SEGGER_RTT.aDown[0];
  unsigned RdOff = pRing->RdOff;
  if (RdOff != pRing->WrOff) {
    int c = (unsigned char)pRing->pBuffer[RdOff];
    RdOff++;
    if (RdOff >= pRing->SizeOfBuffer) {
      RdOff = 0;
    }
    pRing->RdOff = RdOff;
    return c;
  }
  return -1;
}
