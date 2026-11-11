/**
 ******************************************************************************
 * @file    fw_app.h
 * @brief   Common application entry-point contract shared by all firmware
 *          profiles under MotorControls/MotionFirmware/Apps/.
 *
 *          Core/Src/main.c and Core/Src/stm32f4xx_it.c call these hooks
 *          after CubeMX peripheral initialization so a single shared Core/
 *          can build every firmware application target cleanly.
 ******************************************************************************
 */
#ifndef FW_APP_H
#define FW_APP_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Called once in main() after all MX_*_Init() peripheral initializations. */
void fw_app_init(void);

/** Called repeatedly inside main()'s infinite while(1) loop. */
void fw_app_poll(void);

/** Called from TIM2_IRQHandler (motor=0) and TIM5_IRQHandler (motor=1). */
void fw_app_tim_irq(uint8_t motor);

/**
 * Called at the top of USART2_IRQHandler.
 * @return true if the app handled the interrupt directly and
 * HAL_UART_IRQHandler should be skipped; false to continue into
 * HAL_UART_IRQHandler(&huart2).
 */
bool fw_app_usart2_irq(void);

/** Called from Error_Handler() before entering the fault blink loop. */
void fw_app_error_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* FW_APP_H */
