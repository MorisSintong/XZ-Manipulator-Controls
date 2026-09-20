#ifndef TMC2240_MODEL_CACHE_H
#define TMC2240_MODEL_CACHE_H
#include "tmc2240_core.h"
extern int custom_cache_fail_op;
extern unsigned custom_cache_fail_at;
extern unsigned custom_cache_calls;
void custom_cache_reset_faults(void);
#endif
