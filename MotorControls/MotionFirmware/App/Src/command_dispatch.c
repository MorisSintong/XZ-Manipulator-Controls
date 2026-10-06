#include "command_dispatch.h"

void command_dispatch(motion_executor_t *e, const vision_cmd_t *c, uint32_t now)
{
    motion_executor_submit(e, c, now);
}
