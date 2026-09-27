/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : TMC2240 Z-Axis StallGuard4 Sensorless Limit Bouncing
  *                   Hardware: STM32F446RE Nucleo-64 + MKS TMC2240 (SPI Mode)
  *                   Z-Axis: Lead Screw (~400 steps/mm @ 1/16 microstep)
  *                   Dual Telemetry: SEGGER RTT Terminal 0 + USART2 (COM9 @ 115200)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "SEGGER_RTT.h"
#include "tmc2240_hal.h"
#include "tmc2240_core.h"
#include <stdio.h>
#include <stdarg.h>
#include <stdbool.h>

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;

/* TMC2240 SPI Configuration: CS on PC0 */
static const TMC2240_SPIConfig_t tmc_spi_cfgs[] = {
    {
        .hspi = &hspi2,
        .cs_port = TMC_CS0_GPIO_Port,
        .cs_pin = TMC_CS0_Pin,
        .spi_timeout_ms = 100U
    }
};

/* Z-Axis Direction Definitions */
typedef enum {
    DIR_UP = 0,
    DIR_DOWN = 1
} ZDirection_t;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void DWT_Init(void);
static void DWT_DelayUs(uint32_t us);
static void log_printf(const char *fmt, ...);
static TMC2240Status configure_tmc2240_motor(uint16_t icID, uint16_t run_current_mA);
static void dump_driver_diagnostics(uint16_t icID);
static bool probe_until_stall(ZDirection_t dir, uint32_t *steps_moved, uint16_t *stall_sg, uint16_t *stall_base);

/* ---------------------------------------------------------------------------*/
/* High-Precision Cycle Counter (DWT) Timing Functions                        */
/* ---------------------------------------------------------------------------*/
static void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void DWT_DelayUs(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000UL);
    while ((DWT->CYCCNT - start) < ticks);
}

/* ---------------------------------------------------------------------------*/
/* Dual Logger: Sends to SEGGER RTT and USART2 (115200 baud)                  */
/* ---------------------------------------------------------------------------*/
static void log_printf(const char *fmt, ...) {
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    if (len > 0) {
        SEGGER_RTT_Write(0, buf, (unsigned)len);
        HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100U);
    }
}

/* ---------------------------------------------------------------------------*/
/* Main Program                                                               */
/* ---------------------------------------------------------------------------*/
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  DWT_Init();

  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  SEGGER_RTT_Init();

  /* Log Welcome Banner to SEGGER RTT & USART2 COM9 */
  log_printf("\r\n" RTT_CTRL_TEXT_BRIGHT_CYAN);
  log_printf("=================================================================\r\n");
  log_printf("   TMC2240 Z-Axis StallGuard4 Sensorless Limit Bouncing Suite    \r\n");
  log_printf("   Target: STM32F446RE Nucleo-64 @ 84 MHz                        \r\n");
  log_printf("   SPI2: PB13(SCK), PB14(MISO), PB15(MOSI)                      \r\n");
  log_printf("   Pins: CS0=PC0, STEP0=PC1, DIR0=PC2, ENN0=PC6                 \r\n");
  log_printf("   Telemetry: SEGGER RTT Terminal 0 + USART2 @ 115200 (COM9)    \r\n");
  log_printf("=================================================================\r\n" RTT_CTRL_RESET);

  /* Step 1: Initialize TMC2240 HAL */
  log_printf("[INFO] Initializing TMC2240 SPI HAL...\r\n");
  TMC2240Status status = tmc2240_hal_init(tmc_spi_cfgs, 1U);
  if (status != TMC2240_OK) {
      log_printf(RTT_CTRL_TEXT_BRIGHT_RED "[FAIL] tmc2240_hal_init failed! Code: %d\r\n" RTT_CTRL_RESET, (int)status);
      Error_Handler();
  }

  /* Step 2: Test SPI Connection (Verify IOIN.VERSION == 0x40) */
  log_printf("[INFO] Testing SPI connection to TMC2240 (verifying silicon ID)...\r\n");
  HAL_Delay(10);
  status = tmc2240_hal_testConnection(0U);
  if (status != TMC2240_OK) {
      log_printf(RTT_CTRL_TEXT_BRIGHT_RED "[FAIL] SPI Communication Check Failed (status: %d)!\r\n" RTT_CTRL_RESET, (int)status);
      log_printf(RTT_CTRL_TEXT_YELLOW "  Troubleshooting Tips:\r\n");
      log_printf("  1. Verify 24V (or VMOT) is connected and powered ON.\r\n");
      log_printf("  2. Check SPI pins: PB13(SCK), PB14(MISO), PB15(MOSI), PC0(CS).\r\n");
      log_printf("  3. Ensure common GND between STM32 and motor power supply.\r\n" RTT_CTRL_RESET);
  } else {
      log_printf(RTT_CTRL_TEXT_BRIGHT_GREEN "[PASS] TMC2240 Silicon Detected! (Version 0x40 matched)\r\n" RTT_CTRL_RESET);
  }

  /* Step 3: Dump Diagnostic Registers */
  dump_driver_diagnostics(0U);

  /* Step 4: Configure Motor Parameters (24V, 800mA RMS, StealthChop2, StallGuard4) */
  log_printf("[INFO] Configuring TMC2240 motor registers (800mA RMS, StallGuard4)...\r\n");
  status = configure_tmc2240_motor(0U, 800U);
  if (status == TMC2240_OK) {
      log_printf(RTT_CTRL_TEXT_BRIGHT_GREEN "[PASS] TMC2240 StallGuard4 Configured & Ready!\r\n" RTT_CTRL_RESET);
  } else {
      log_printf(RTT_CTRL_TEXT_BRIGHT_RED "[WARN] Motor configuration returned code: %d\r\n" RTT_CTRL_RESET, (int)status);
  }

  log_printf("\r\n" RTT_CTRL_TEXT_BRIGHT_YELLOW);
  log_printf("=================================================================\r\n");
  log_printf(">>> PRESS BLUE BUTTON (B1 / PC13) TO START Z-AXIS BOUNCING <<<\r\n");
  log_printf(">>> (Press Button again anytime during motion to STOP)        <<<\r\n");
  log_printf("=================================================================\r\n" RTT_CTRL_RESET);

  /* Keep ENN disabled (HIGH) until user presses button */
  HAL_GPIO_WritePin(TMC_ENN0_GPIO_Port, TMC_ENN0_Pin, GPIO_PIN_SET);

  uint32_t bounce_cycles = 0;

  /* Infinite loop */
  while (1)
  {
    /* Wait for Blue User Button (PC13) press to START */
    if (HAL_GPIO_ReadPin(B1_USER_BUTTON_Port, B1_USER_BUTTON_Pin) == GPIO_PIN_RESET)
    {
      HAL_Delay(50); /* Debounce */
      if (HAL_GPIO_ReadPin(B1_USER_BUTTON_Port, B1_USER_BUTTON_Pin) == GPIO_PIN_RESET)
      {
        /* Wait for button release before starting */
        while (HAL_GPIO_ReadPin(B1_USER_BUTTON_Port, B1_USER_BUTTON_Pin) == GPIO_PIN_RESET) {
            HAL_Delay(10);
        }
        HAL_Delay(100);

        log_printf("\r\n" RTT_CTRL_TEXT_BRIGHT_GREEN "[START] Z-Axis Continuous StallGuard Bouncing Started!\r\n" RTT_CTRL_RESET);

        /* Turn on Green LED LD2 */
        HAL_GPIO_WritePin(LD2_LED_GREEN_Port, LD2_LED_GREEN_Pin, GPIO_PIN_SET);

        /* Enable motor driver (ENN active LOW) */
        HAL_GPIO_WritePin(TMC_ENN0_GPIO_Port, TMC_ENN0_Pin, GPIO_PIN_RESET);
        HAL_Delay(50); /* Power stage wake-up time */

        bool user_aborted = false;
        uint32_t measured_rail_steps = 0;

        while (!user_aborted)
        {
            bounce_cycles++;
            uint32_t steps_up = 0;
            uint16_t stall_sg = 0, stall_base = 0;

            /* ------------------------------------------------------------- */
            /* 1. Probe UP towards TOP hardstop                              */
            /* ------------------------------------------------------------- */
            log_printf(RTT_CTRL_TEXT_BRIGHT_CYAN "\r\n--- [Cycle #%lu] Probing UP towards TOP Hardstop ---\r\n" RTT_CTRL_RESET, bounce_cycles);
            bool stalled = probe_until_stall(DIR_UP, &steps_up, &stall_sg, &stall_base);

            /* Check if user stopped the cycle */
            if (HAL_GPIO_ReadPin(B1_USER_BUTTON_Port, B1_USER_BUTTON_Pin) == GPIO_PIN_RESET) {
                user_aborted = true;
                break;
            }

            if (stalled) {
                log_printf(RTT_CTRL_TEXT_BRIGHT_GREEN "[STALL] Top Hardstop Reached! Steps: %lu (SG=%u, Base=%u)\r\n" RTT_CTRL_RESET,
                           steps_up, stall_sg, stall_base);
            } else {
                log_printf(RTT_CTRL_TEXT_BRIGHT_RED "[WARN] Max travel limit reached on UP stroke without stall!\r\n" RTT_CTRL_RESET);
            }

            /* Settle pause to relax mechanical stress */
            log_printf("[HOLD] Pausing 300 ms for stress relaxation...\r\n");
            HAL_Delay(300);

            /* Check button again */
            if (HAL_GPIO_ReadPin(B1_USER_BUTTON_Port, B1_USER_BUTTON_Pin) == GPIO_PIN_RESET) {
                user_aborted = true;
                break;
            }

            /* ------------------------------------------------------------- */
            /* 2. Probe DOWN towards BOTTOM hardstop                         */
            /* ------------------------------------------------------------- */
            uint32_t steps_down = 0;
            log_printf(RTT_CTRL_TEXT_BRIGHT_CYAN "--- [Cycle #%lu] Probing DOWN towards BOTTOM Hardstop ---\r\n" RTT_CTRL_RESET, bounce_cycles);
            stalled = probe_until_stall(DIR_DOWN, &steps_down, &stall_sg, &stall_base);

            if (HAL_GPIO_ReadPin(B1_USER_BUTTON_Port, B1_USER_BUTTON_Pin) == GPIO_PIN_RESET) {
                user_aborted = true;
                break;
            }

            if (stalled) {
                measured_rail_steps = steps_down;
                float travel_mm = (float)measured_rail_steps / 400.0f; /* ~400 steps/mm on T8 screw @ 1/16 */
                log_printf(RTT_CTRL_TEXT_BRIGHT_GREEN "[STALL] Bottom Hardstop Reached! Rail Travel: %lu steps (~%.2f mm) (SG=%u, Base=%u)\r\n" RTT_CTRL_RESET,
                           measured_rail_steps, travel_mm, stall_sg, stall_base);
            } else {
                log_printf(RTT_CTRL_TEXT_BRIGHT_RED "[WARN] Max travel limit reached on DOWN stroke without stall!\r\n" RTT_CTRL_RESET);
            }

            /* Settle pause before next reversal */
            log_printf("[HOLD] Pausing 300 ms for stress relaxation...\r\n");
            HAL_Delay(300);

            if (HAL_GPIO_ReadPin(B1_USER_BUTTON_Port, B1_USER_BUTTON_Pin) == GPIO_PIN_RESET) {
                user_aborted = true;
                break;
            }
        }

        /* Stop Motion & Disable Driver */
        HAL_GPIO_WritePin(TMC_ENN0_GPIO_Port, TMC_ENN0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LD2_LED_GREEN_Port, LD2_LED_GREEN_Pin, GPIO_PIN_RESET);

        log_printf("\r\n" RTT_CTRL_TEXT_BRIGHT_YELLOW);
        log_printf("=================================================================\r\n");
        log_printf("[STOP] Motor Safely Disabled (ENN=HIGH). Total Cycles: %lu\r\n", bounce_cycles);
        if (measured_rail_steps > 0) {
            log_printf("[INFO] Calibrated Z Rail Length: %lu steps (~%.2f mm)\r\n",
                       measured_rail_steps, (float)measured_rail_steps / 400.0f);
        }
        log_printf(">>> Press Blue Button to start a new bouncing session <<<\r\n");
        log_printf("=================================================================\r\n" RTT_CTRL_RESET);

        /* Wait for button release */
        while (HAL_GPIO_ReadPin(B1_USER_BUTTON_Port, B1_USER_BUTTON_Pin) == GPIO_PIN_RESET) {
            HAL_Delay(20);
        }
      }
    }
    HAL_Delay(10);
  }
}

/* ---------------------------------------------------------------------------*/
/* Probe In Specified Direction Until StallGuard4 Endstop Triggered           */
/* ---------------------------------------------------------------------------*/
static bool probe_until_stall(ZDirection_t dir, uint32_t *steps_moved, uint16_t *stall_sg, uint16_t *stall_base)
{
    /* Set DIR Pin: DIR_UP = HIGH, DIR_DOWN = LOW */
    HAL_GPIO_WritePin(TMC_DIR0_GPIO_Port, TMC_DIR0_Pin, (dir == DIR_UP) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    DWT_DelayUs(20);

    /* Velocity Parameters:
     * Gentle probe speed: 2000 steps/s (period = 500 us, ~5 mm/s on T8)
     * Ramp-up start speed: 500 steps/s (period = 2000 us)
     * Ramp duration: 500 steps (~0.25s)
     */
    const uint32_t target_period_us = 500U;
    const uint32_t start_period_us  = 2000U;
    const uint32_t ramp_steps       = 500U;
    const uint32_t holdoff_steps    = 400U;
    const uint32_t max_leg_steps    = 100000U; /* 250 mm travel safety limit */

    int32_t sg_base = 0;
    bool baseline_initialized = false;
    uint8_t low_count = 0;
    uint32_t step_count = 0;
    uint32_t last_telemetry_tick = HAL_GetTick();

    while (step_count < max_leg_steps)
    {
        /* Check if user pressed button to abort */
        if (HAL_GPIO_ReadPin(B1_USER_BUTTON_Port, B1_USER_BUTTON_Pin) == GPIO_PIN_RESET) {
            *steps_moved = step_count;
            return false;
        }

        /* Calculate current step period (smooth acceleration ramp) */
        uint32_t current_period_us;
        if (step_count < ramp_steps) {
            current_period_us = start_period_us - ((start_period_us - target_period_us) * step_count / ramp_steps);
        } else {
            current_period_us = target_period_us;
        }

        /* Pulse STEP Pin (5 us high pulse) */
        HAL_GPIO_WritePin(TMC_STEP0_GPIO_Port, TMC_STEP0_Pin, GPIO_PIN_SET);
        DWT_DelayUs(5);
        HAL_GPIO_WritePin(TMC_STEP0_GPIO_Port, TMC_STEP0_Pin, GPIO_PIN_RESET);

        /* Wait step period */
        if (current_period_us > 5) {
            DWT_DelayUs(current_period_us - 5);
        }
        step_count++;

        /* Sample StallGuard4 every 32 steps, after acceleration holdoff */
        if ((step_count > holdoff_steps) && ((step_count % 32U) == 0U))
        {
            uint16_t sg_val = 0;
            if (tmc2240_stallguard_read(0U, &sg_val) == TMC2240_OK)
            {
                if (!baseline_initialized) {
                    sg_base = (int32_t)sg_val;
                    baseline_initialized = true;
                } else {
                    /* Adaptive EMA filter of baseline load: EMA weight 1/16 */
                    if ((int32_t)sg_val >= (sg_base - 60)) {
                        sg_base += ((int32_t)sg_val - sg_base) >> 4;
                    }
                }

                /* Stall Condition:
                 * Load drops sharply 60 below baseline, or absolute load drops near 0 (< 30) */
                if (((int32_t)sg_val < (sg_base - 60)) || (sg_val < 30U)) {
                    low_count++;
                    if (low_count >= 3U) {
                        /* Confirmed Stall! */
                        *steps_moved = step_count;
                        *stall_sg = sg_val;
                        *stall_base = (uint16_t)sg_base;
                        return true;
                    }
                } else {
                    low_count = 0;
                }

                /* Periodic live telemetry log (every 500 ms) */
                if ((HAL_GetTick() - last_telemetry_tick) >= 500U) {
                    last_telemetry_tick = HAL_GetTick();
                    log_printf("  [Z-RUN] Dir:%s | Steps:%6lu | SG4:%4u | Base:%4ld\r\n",
                               (dir == DIR_UP) ? "UP  " : "DOWN", step_count, sg_val, (long)sg_base);
                }
            }
        }
    }

    /* Reached max limit without stall */
    *steps_moved = step_count;
    return false;
}

/* ---------------------------------------------------------------------------*/
/* TMC2240 Motor & StallGuard4 Configuration Helper                           */
/* ---------------------------------------------------------------------------*/
static TMC2240Status configure_tmc2240_motor(uint16_t icID, uint16_t run_current_mA)
{
    /* Clear any initial startup fault flags */
    uint8_t faults = 0;
    tmc2240_check_faults(icID, &faults);
    if (faults != 0) {
        log_printf("[INFO] Clearing startup faults: 0x%02X\r\n", faults);
        tmc2240_clear_faults(icID, faults);
    }

    TMC2240_MotorConfig_t motor_cfg = {
        .gconf = TMC2240_EN_PWM_MODE_MASK,            /* StealthChop2 enabled */
        .drv_conf = 2U << TMC2240_SLOPE_CONTROL_SHIFT, /* Moderate slew rate */
        .global_scaler = 0U,                          /* Full scale (256) */
        .ihold_irun = 0U,                             /* Calculated below */
        .chopconf = 0x14410150U,                      /* 1/16 microstep, 256 interpolation */
        .pwmconf = 0xC40C001DU                        /* StealthChop autoscale enabled */
    };

    /* RREF on standard TMC2240 breakout modules is 12,000 ohms */
    const uint16_t rref_ohms = 12000U;
    TMC2240Status status = tmc2240_calculateCurrent(run_current_mA, rref_ohms, 0U, 0U,
                                                   50U, 5U, 5U, &motor_cfg.ihold_irun);
    if (status != TMC2240_OK) {
        log_printf("[WARN] calculateCurrent returned: %d, applying standard current register\r\n", (int)status);
        /* Fallback: IRUN=20 (approx 800mA RMS), IHOLD=10, IHOLDDELAY=6 */
        motor_cfg.ihold_irun = (6U << 16) | (20U << 8) | (10U);
    }

    /* Configure driver registers */
    status = tmc2240_hal_configureMotor(icID, &motor_cfg);
    if (status != TMC2240_OK) {
        return status;
    }

    /* Set stealthchop threshold to high speed so it operates in quiet mode */
    tmc2240_set_tpwmthrs(icID, 0U);
    tmc2240_set_tpowerdown(icID, 20U);

    /* Configure velocity threshold so StallGuard4 is active at 2000 steps/s */
    tmc2240_set_tcoolthrs(icID, 0x000FFFFFUL);

    /* StallGuard4 threshold: 100 (0..255, higher = more sensitive) */
    tmc2240_stallguard_set_threshold(icID, 100U);

    /* Activate chopper with TOFF=3 */
    return tmc2240_hal_activateMotor(icID, 3U);
}

/* ---------------------------------------------------------------------------*/
/* Diagnostics Dump via Dual Logger                                           */
/* ---------------------------------------------------------------------------*/
static void dump_driver_diagnostics(uint16_t icID)
{
    uint32_t val = 0;
    log_printf("\r\n---------------- [TMC2240 Register Diagnostics] ----------------\r\n");

    /* GCONF (0x00) */
    if (tmc2240_readRegister(icID, 0x00, &val) == TMC2240_OK) {
        bool stealthchop = (val & TMC2240_EN_PWM_MODE_MASK) != 0;
        log_printf("  GCONF       (0x00): 0x%08lX  [StealthChop2: %s]\r\n",
                   val, stealthchop ? "ENABLED" : "DISABLED");
    }

    /* GSTAT (0x01) */
    if (tmc2240_readRegister(icID, 0x01, &val) == TMC2240_OK) {
        log_printf("  GSTAT       (0x01): 0x%08lX  [Reset: %d, DrvErr: %d, UV_CP: %d]\r\n",
                   val, (int)(val & 1), (int)((val >> 1) & 1), (int)((val >> 2) & 1));
    }

    /* IOIN (0x04) */
    if (tmc2240_readRegister(icID, 0x04, &val) == TMC2240_OK) {
        uint8_t version = (uint8_t)((val >> 24) & 0xFF);
        bool step_pin = (val & (1 << 0)) != 0;
        bool dir_pin  = (val & (1 << 1)) != 0;
        log_printf("  IOIN        (0x04): 0x%08lX  [Version: 0x%02X, STEP: %d, DIR: %d]\r\n",
                   val, version, (int)step_pin, (int)dir_pin);
    }

    /* DRV_STATUS (0x6F) */
    if (tmc2240_readRegister(icID, 0x6F, &val) == TMC2240_OK) {
        bool ot     = (val & (1 << 25)) != 0;
        bool otpw   = (val & (1 << 26)) != 0;
        bool s2ga   = (val & (1 << 27)) != 0;
        bool s2gb   = (val & (1 << 28)) != 0;
        bool stst   = (val & (1 << 31)) != 0;
        uint16_t sg = (uint16_t)(val & 0x3FF);
        log_printf("  DRV_STATUS  (0x6F): 0x%08lX  [Standstill: %d, SG_LOAD: %u, OT: %d, OTPW: %d, Short: %d]\r\n",
                   val, (int)stst, sg, (int)ot, (int)otpw, (int)(s2ga || s2gb));
    }

    /* CHOPCONF (0x6C) */
    if (tmc2240_readRegister(icID, 0x6C, &val) == TMC2240_OK) {
        uint8_t toff = val & 0x0F;
        uint8_t mres = (val >> 24) & 0x0F;
        bool intpol  = (val & (1 << 28)) != 0;
        log_printf("  CHOPCONF    (0x6C): 0x%08lX  [TOFF: %d, MRES: %d (1/%d step), INTPOL: %d]\r\n",
                          val, toff, mres, 1 << mres, (int)intpol);
    }

    /* SG4_IND (0x74) */
    uint16_t sg_result = 0;
    if (tmc2240_stallguard_read(icID, &sg_result) == TMC2240_OK) {
        log_printf("  SG4_RESULT  (0x74): %u (StallGuard4 live load indicator)\r\n", sg_result);
    }

    log_printf("----------------------------------------------------------------\r\n\r\n");
}

/* ---------------------------------------------------------------------------*/
/* Hardware Peripheral Initializations                                        */
/* ---------------------------------------------------------------------------*/
static void MX_SPI2_Init(void)
{
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  /* TMC2240 uses SPI Mode 3: CPOL=1, CPHA=1 */
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  /* APB1 = 42MHz. Prescaler 16 -> 2.625 MHz SPI Clock */
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable GPIO Clocks */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Configure CS0 (PC0): Output PP, Initial SET (High / Deselected) */
  HAL_GPIO_WritePin(TMC_CS0_GPIO_Port, TMC_CS0_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = TMC_CS0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(TMC_CS0_GPIO_Port, &GPIO_InitStruct);

  /* Configure STEP0 (PC1) & DIR0 (PC2): Output PP, Initial RESET */
  HAL_GPIO_WritePin(GPIOC, TMC_STEP0_Pin | TMC_DIR0_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = TMC_STEP0_Pin | TMC_DIR0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* Configure ENN0 (PC6): Output PP, Initial SET (High / Disabled for safety) */
  HAL_GPIO_WritePin(TMC_ENN0_GPIO_Port, TMC_ENN0_Pin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = TMC_ENN0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(TMC_ENN0_GPIO_Port, &GPIO_InitStruct);

  /* Configure Green LED LD2 (PA5): Output PP, Initial RESET (Off) */
  HAL_GPIO_WritePin(LD2_LED_GREEN_Port, LD2_LED_GREEN_Pin, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = LD2_LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_LED_GREEN_Port, &GPIO_InitStruct);

  /* Configure Blue User Button B1 (PC13): Input with Pull-Up */
  GPIO_InitStruct.Pin = B1_USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B1_USER_BUTTON_Port, &GPIO_InitStruct);
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    /* Fast toggle LED to signal fault state */
    HAL_GPIO_TogglePin(LD2_LED_GREEN_Port, LD2_LED_GREEN_Pin);
    for (volatile uint32_t i = 0; i < 500000; i++) __NOP();
  }
}
