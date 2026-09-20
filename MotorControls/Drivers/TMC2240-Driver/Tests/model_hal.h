#ifndef TMC2240_MODEL_HAL_H
#define TMC2240_MODEL_HAL_H
#include "tmc2240_hal.h"
#include "model_device.h"
typedef struct {
    char kind;
    unsigned id;
    uint64_t time;
    GPIO_PinState level;
    uint8_t tx[5];
} HalTrace;
extern HalTrace hal_trace[1024];
extern unsigned hal_trace_count;
extern unsigned hal_calls;
extern unsigned hal_fail_at;
extern unsigned hal_fail_also;
extern bool hal_fail_after_accept;
extern unsigned hal_drop_write_at;
extern unsigned hal_corrupt_at;
extern unsigned hal_status_at;
extern uint8_t hal_status_bits;
extern HAL_StatusTypeDef hal_failure;
extern SPI_HandleTypeDef hal_handles[4];
extern TMC2240_SPIConfig_t hal_configs[4];
extern uint32_t hal_pclk1, hal_pclk2;
extern uint64_t hal_nops;
extern bool hal_selected[4];
void hal_model_reset(void);
void hal_trace_reset(void);
void hal_check_trace(void);
#endif
