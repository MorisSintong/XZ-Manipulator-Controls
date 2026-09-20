#ifndef FAKE_HAL_H
#define FAKE_HAL_H
#include "stm32f4xx_hal.h"
#include "tmc2209_port.h"
#include <stdbool.h>
#include <limits.h>
typedef struct {
    USART_TypeDef uart_regs;
    UART_HandleTypeDef uart;
    uint32_t registers[4][128];
    uint32_t cycle_step, tick, delayed_ms;
    unsigned tx_mode_calls, rx_mode_calls, clears, reads, writes, status_calls;
    unsigned frames, accepted_writes, active_writes, gstat_writes;
    unsigned fail_tx_mode_call, fail_rx_mode_call;
    HAL_StatusTypeDef tx_mode_result, rx_mode_result;
    uint8_t tx[8], reply[8], fifo[64];
    unsigned tx_size, expected_size, fifo_size, fifo_pos, rx_read_count;
    unsigned reply_bytes, txe_delay, tc_delay, rx_delay;
    unsigned block_txe_at, rx_error_at, tx_error_at, drop_write;
    uint32_t inject_error;
    bool pending_reply, echo, stuck_rx, stuck_rx_after_tx, stuck_tc, corrupt_crc, corrupt_identity;
} fake_hal_t;
extern fake_hal_t hal_fake;
void hal_reset(void);
void hal_chip_reset(uint8_t address);
tmc2209_port_t hal_port(void);
#endif
