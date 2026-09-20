#ifndef FAKE_CORE_H
#define FAKE_CORE_H
#include "tmc2209.h"
#include <stddef.h>

typedef struct {
    uint32_t regs[4][128];
    uint8_t last_frame[8];
    uint16_t last_len;
    uint8_t request_addr, request_reg;
    unsigned tx_calls, rx_calls, write_calls, lock_calls, unlock_calls;
    unsigned init_calls, deinit_calls, create_calls, delete_calls;
    unsigned fail_tx_call, fail_rx_call, drop_write_call, bad_readback_reg;
    unsigned fail_lock_call, fail_unlock_call;
    tmc2209_status_t init_result, deinit_result, create_result;
    tmc2209_status_t tx_result, rx_result, lock_result, unlock_result;
    bool locked, allocated, corrupt_crc, corrupt_identity;
    uint8_t reply_reg_xor, extra_ifcnt;
    unsigned active_writes, gstat_writes;
    unsigned event_before_tx, event_before_rx, event_after_rx, event_count;
    bool event_reset;
    uint32_t event_gstat, event_drv_status;
} fake_core_t;
extern fake_core_t core_fake;
extern UART_HandleTypeDef *const core_uart;
void core_reset(void);
void core_chip_reset(uint8_t address);
tmc2209_status_t core_bus_init(tmc2209_bus_t *bus);
tmc2209_motor_config_t core_config(void);
#endif
