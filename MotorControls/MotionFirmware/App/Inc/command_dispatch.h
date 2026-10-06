#ifndef COMMAND_DISPATCH_H
#define COMMAND_DISPATCH_H
#include "motion_executor.h"
/* Called only by the foreground parser, never by DMA interrupts. */
void command_dispatch(motion_executor_t *, const vision_cmd_t *, uint32_t now_us);
#endif
