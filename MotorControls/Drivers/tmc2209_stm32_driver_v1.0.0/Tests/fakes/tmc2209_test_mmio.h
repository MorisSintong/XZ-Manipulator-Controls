#ifndef TMC2209_TEST_MMIO_H
#define TMC2209_TEST_MMIO_H
#include "stm32f4xx_hal.h"
uint32_t fake_hal_cycles(void);
uint32_t fake_hal_status(UART_HandleTypeDef *h);
uint8_t fake_hal_read(UART_HandleTypeDef *h);
void fake_hal_write(UART_HandleTypeDef *h, uint8_t value);
#define TMC2209_CYCLES() fake_hal_cycles()
#define TMC2209_STATUS(h) fake_hal_status(h)
#define TMC2209_READ_BYTE(h) fake_hal_read(h)
#define TMC2209_WRITE_BYTE(h, value) fake_hal_write(h, value)
#endif
