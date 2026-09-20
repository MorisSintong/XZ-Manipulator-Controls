/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : TMC2240 Stepper Motor Control
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tmc2240_hal.h"
#include "tmc2240_core.h"
#include "tmc2240_demo_guard.h"
#include "tmc2240_demo_startup.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#ifndef TMC2240_DEMO_ENABLE_MOTION
#define TMC2240_DEMO_ENABLE_MOTION 0
#endif
#if TMC2240_DEMO_ENABLE_MOTION != 0 && TMC2240_DEMO_ENABLE_MOTION != 1
#error "TMC2240_DEMO_ENABLE_MOTION must be 0 or 1"
#endif
#if TMC2240_DEMO_ENABLE_MOTION
#if !defined(TMC2240_DEMO_RUN_CURRENT_MA) || !defined(TMC2240_DEMO_RREF_OHM) || \
    !defined(TMC2240_DEMO_MAX_STEPS) || !defined(TMC2240_DEMO_TIMEOUT_MS) || \
    !defined(TMC2240_DEMO_EN_PORT) || !defined(TMC2240_DEMO_EN_PIN) || \
    !defined(TMC2240_DEMO_EN_CLOCK_ENABLE) || !defined(TMC2240_DEMO_ACK_STARTUP_FLAGS)
#error "Motion requires explicit current, RREF, limits, ENN GPIO, and startup acknowledgement policy"
#endif
_Static_assert(TMC2240_DEMO_RUN_CURRENT_MA > 0U &&
               TMC2240_DEMO_RUN_CURRENT_MA <= UINT16_MAX, "Invalid motor current");
_Static_assert(TMC2240_DEMO_RREF_OHM > 0U &&
               TMC2240_DEMO_RREF_OHM <= UINT16_MAX, "Invalid RREF");
_Static_assert(TMC2240_DEMO_MAX_STEPS > 0U &&
               TMC2240_DEMO_MAX_STEPS <= INT32_MAX, "Invalid travel limit");
_Static_assert(TMC2240_DEMO_TIMEOUT_MS > 0U &&
               TMC2240_DEMO_TIMEOUT_MS <= INT32_MAX, "Invalid motion timeout");
_Static_assert((TMC2240_DEMO_ACK_STARTUP_FLAGS & ~0x1FU) == 0U,
               "Invalid startup GSTAT acknowledgement mask");
_Static_assert(TMC2240_DEMO_EN_PIN > 0U &&
               TMC2240_DEMO_EN_PIN <= UINT16_MAX &&
               (TMC2240_DEMO_EN_PIN & (TMC2240_DEMO_EN_PIN - 1U)) == 0U,
               "ENN must use one GPIO pin");
#endif
/* Nominal speeds only: pulse timing must be measured on the target hardware. */
#define TMC_HOME_SPEED_HZ        8000U    /* slow speed: SG-valid, used for homing and end detection */
#define TMC_TRAVEL_SPEED_HZ      24000U   /* fast traverse between the two ends */
#define TMC_RAMPUP_STEPS         2000U    /* half-speed steps before homing starts */
#define TMC_SLOW_ZONE_STEPS      2000UL   /* slow down this many steps before each end (step count) */
#define TMC_MAX_RAIL_STEPS       TMC2240_DEMO_MAX_STEPS
#define TMC_SG_DROP_MARGIN       80       /* slow-speed stall: SG drops this far below baseline (higher = less sensitive) */
#define TMC_SG_BASELINE_SHIFT    4U       /* baseline EMA weight: 1/16 per SG read */
#define TMC_SG_CONFIRM_COUNT     2U       /* consecutive low reads required to confirm stall */
#define TMC_SG_CHECK_INTERVAL    64U      /* steps between SG4 reads (each read ~0.5 ms SPI) */
#define TMC_SG_HOLDOFF_STEPS     500U     /* iterations after state change before SG checks resume */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static const TMC2240_SPIConfig_t tmc_spi_cfgs[] = {
    { .hspi = &hspi2, .cs_port = TMC_CS_GPIO_Port,
      .cs_pin = TMC_CS_Pin, .spi_timeout_ms = 100U },
};
static volatile bool demo_gpio_ready;
static volatile TMC2240Status demo_last_error = TMC2240_OK;
#if TMC2240_DEMO_ENABLE_MOTION
static tmc_demo_guard_t demo_guard;
static volatile bool demo_enable_pin_ready;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static void tmc2240_init(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void demo_require(TMC2240Status status)
{
    if (status != TMC2240_OK) {
        if (demo_last_error == TMC2240_OK) {
            demo_last_error = status;
        }
        Error_Handler();
    }
}

/* ===== TMC2240 Initialization ===== */
static void tmc2240_init(void) {
    demo_require(tmc2240_hal_init(tmc_spi_cfgs, 1U));
    demo_require(tmc2240_hal_testConnection(0U));
}

static void demo_send(char *message, int length, size_t capacity)
{
    uint16_t count;
    if (!tmc_demo_message_length(length, capacity, &count)) {
        demo_require(TMC2240_ERROR_RANGE);
    }
    if (HAL_UART_Transmit(&huart2, (uint8_t *)message, count, 100U) != HAL_OK) {
        demo_require(TMC2240_ERROR_IO);
    }
}

#if TMC2240_DEMO_ENABLE_MOTION
static void demo_init_enable_pin(void)
{
    GPIO_InitTypeDef gpio = {0};
    GPIO_TypeDef *const port = TMC2240_DEMO_EN_PORT;
    const uint16_t pin = TMC2240_DEMO_EN_PIN;
    if (port == NULL ||
        (port == TMC_CS_GPIO_Port && (pin & TMC_CS_Pin) != 0U) ||
        (port == GPIOA && (pin & (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 |
                                GPIO_PIN_3 | GPIO_PIN_5 | GPIO_PIN_13 |
                                GPIO_PIN_14)) != 0U) ||
        (port == GPIOB && (pin & (GPIO_PIN_3 | GPIO_PIN_10)) != 0U) ||
        (port == GPIOC && (pin & (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_13)) != 0U)) {
        tmc_demo_guard_stop(&demo_guard, TMC_DEMO_INVALID_CONFIGURATION);
        Error_Handler();
    }
    TMC2240_DEMO_EN_CLOCK_ENABLE();
    HAL_GPIO_WritePin(TMC2240_DEMO_EN_PORT, TMC2240_DEMO_EN_PIN, GPIO_PIN_SET);
    gpio.Pin = TMC2240_DEMO_EN_PIN;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(TMC2240_DEMO_EN_PORT, &gpio);
    demo_enable_pin_ready = true;
}

/* ===== LED status signaling (LD2 = PA5) =====
 * Solid ON: motor stepping normally
 * Blinking: an endpoint was confirmed.
 * Each blink phase is ~5 ms (400k NOPs @ 84 MHz). */
static void blink_led(uint32_t pulses)
{
    uint32_t p;
    for (p = 0U; p < pulses; p++) {
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
        { volatile uint32_t j; for (j = 0U; j < 400000UL; j++) { __NOP(); } }
        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
        { volatile uint32_t j; for (j = 0U; j < 400000UL; j++) { __NOP(); } }
    }
}
#endif

/* USER CODE END 0 */

/**
  * @brief  Main program - TMC2240 stepper motor control
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  HAL_Init();
  /* USER CODE BEGIN Init */
#if TMC2240_DEMO_ENABLE_MOTION
  demo_init_enable_pin();
#endif
  /* USER CODE END Init */
  SystemClock_Config();
  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */

  /* ---- Send startup message ---- */
  {
      char startup[] = "TMC2240 diagnostics; motion requires explicit build configuration\r\n";
      demo_send(startup, (int)(sizeof(startup) - 1U), sizeof(startup));
  }

  tmc2240_init();

#if TMC2240_DEMO_ENABLE_MOTION
  /* ---- Configure STEP (PA0) and DIR (PA1) ---- */
  {
      GPIO_InitTypeDef g = {0};
      g.Pin = GPIO_PIN_0 | GPIO_PIN_1;
      g.Mode = GPIO_MODE_OUTPUT_PP;
      g.Pull = GPIO_NOPULL;
      g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
      HAL_GPIO_Init(GPIOA, &g);
  }

  demo_require(tmc_demo_prepare_motor(0U, TMC2240_DEMO_RUN_CURRENT_MA,
                                     TMC2240_DEMO_RREF_OHM,
                                     TMC2240_DEMO_ACK_STARTUP_FLAGS));
  demo_require(tmc2240_hal_activateMotor(0U, 3U));
  if (!tmc_demo_guard_init(&demo_guard, HAL_GetTick(), TMC_MAX_RAIL_STEPS,
                           TMC2240_DEMO_TIMEOUT_MS)) {
      Error_Handler();
  }
  HAL_GPIO_WritePin(TMC2240_DEMO_EN_PORT, TMC2240_DEMO_EN_PIN, GPIO_PIN_RESET);

  /* ---- Start forward (CW) ---- */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

  /* ---- Ramp up at half homing speed before homing begins ---- */
  {
      volatile uint32_t dly = 7000000UL / (TMC_HOME_SPEED_HZ / 2U);
      uint32_t ramp;
      for (ramp = 0UL; ramp < TMC_RAMPUP_STEPS; ramp++) {
          if (!tmc_demo_guard_step(&demo_guard, HAL_GetTick(), (int32_t)ramp, 1)) {
              Error_Handler();
          }
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
          { volatile uint32_t j; for (j = 0; j < dly; j++) { __NOP(); } }
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
          { volatile uint32_t j; for (j = 0; j < dly; j++) { __NOP(); } }
      }
  }
  if (!tmc_demo_guard_new_leg(&demo_guard, HAL_GetTick())) {
      Error_Handler();
  }
#endif

  /* USER CODE END 2 */

  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */

#if TMC2240_DEMO_ENABLE_MOTION
    {
        /* States: slow homing -> slow measure of the rail -> fast bounce with
         * SG-confirmed ends. Slowdown is step-count based; SG runs only at slow
         * speed where it is reliable. */
        enum { ST_HOMING = 0, ST_MEASURE, ST_BOUNCE };
        static const char *const state_names[] = { "HOMING", "MEASURE", "BOUNCE" };

        static uint8_t  state       = ST_HOMING;
        static int8_t   dir         = -1;    /* -1 = toward home, +1 = away */
        static int32_t  pos         = 0L;    /* software position; 0 = home end */
        static int32_t  rail        = 0L;    /* rail length in steps, measured in ST_MEASURE */
        static uint8_t  slow        = 1U;    /* 1 = slow (SG-valid), 0 = fast traverse */
        static uint32_t state_age   = 0UL;   /* iterations since last state change */
        static uint32_t settle      = 0UL;   /* iterations to pause after a confirm */
        static uint32_t sg_low_cnt  = 0UL;
        static int32_t  sg_base     = 0L;    /* EMA of free-running SG (slow speed only) */
        static uint32_t stall_cnt   = 0UL;
        static uint32_t home_cnt    = 0UL;
        static uint32_t step_cnt    = 0UL;
        static uint32_t fault_timer = 0UL;
        static uint32_t last_print  = 0UL;
        uint32_t cur_hz;
        volatile uint32_t dly;

        cur_hz = ((state == ST_BOUNCE) && (slow == 0U)) ? TMC_TRAVEL_SPEED_HZ
                                                        : TMC_HOME_SPEED_HZ;
        dly = 7000000UL / cur_hz;

        HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);  /* solid = running */

        fault_timer++;
        if ((fault_timer & 1023UL) == 0UL) {
            uint8_t faults = 0U;
            demo_require(tmc2240_check_faults(0U, &faults));
            if (faults != 0U) {
                char evt[64];
                int len = snprintf(evt, sizeof(evt),
                                   "FAULT 0x%02X; stopping\r\n", (unsigned)faults);
                tmc_demo_guard_stop(&demo_guard, TMC_DEMO_DRIVER_FAULT);
                demo_last_error = TMC2240_ERROR_FAULT;
                HAL_GPIO_WritePin(TMC2240_DEMO_EN_PORT, TMC2240_DEMO_EN_PIN, GPIO_PIN_SET);
                demo_send(evt, len, sizeof(evt));
                Error_Handler();
            }
        }

        step_cnt++;
        state_age++;

        if (settle > 0UL) {
            /* brief pause after a confirm: let coil current settle before stepping the other way */
            settle--;
            { volatile uint32_t j; for (j = 0; j < dly; j++) { __NOP(); } }
            { volatile uint32_t j; for (j = 0; j < dly; j++) { __NOP(); } }
        } else {
            /* ---- SG sampling (slow speed only, where it is reliable) ---- */
            if ((slow != 0U) && ((step_cnt % TMC_SG_CHECK_INTERVAL) == 0UL) &&
                (state_age > TMC_SG_HOLDOFF_STEPS)) {
                uint16_t sg_value = 0U;
                demo_require(tmc2240_stallguard_read(0U, &sg_value));
                const int32_t sg = (int32_t)sg_value;

                {
                    /* track the free-running baseline; confirm stall on persistent drop */
                    if (sg >= (sg_base - (int32_t)TMC_SG_DROP_MARGIN)) {
                        sg_base += (sg - sg_base) >> TMC_SG_BASELINE_SHIFT;
                    }
                    if (sg < (sg_base - (int32_t)TMC_SG_DROP_MARGIN)) {
                        sg_low_cnt++;
                    } else {
                        sg_low_cnt = 0UL;
                    }

                    if (sg_low_cnt >= TMC_SG_CONFIRM_COUNT) {
                        if (!tmc_demo_guard_new_leg(&demo_guard, HAL_GetTick())) {
                            Error_Handler();
                        }
                        sg_low_cnt = 0UL;
                        settle = 200UL;
                        state_age = 0UL;

                        if (state == ST_HOMING) {
                            /* first end found: that is home; measure the rail away from it */
                            home_cnt++;
                            pos = 0L;
                            dir = 1;
                            state = ST_MEASURE;
                            blink_led(2U);
                            char evt[128];
                            int len = snprintf(evt, sizeof(evt),
                                "HOME #%lu found @step %lu SG=%ld base=%ld\r\n",
                                (unsigned long)home_cnt, (unsigned long)step_cnt,
                                (long)sg, (long)sg_base);
                            demo_send(evt, len, sizeof(evt));
                        } else if (state == ST_MEASURE) {
                            if (pos <= 0L) {
                                tmc_demo_guard_stop(&demo_guard, TMC_DEMO_INVALID_CONFIGURATION);
                                Error_Handler();
                            }
                            /* far end found: rail length known, start the fast bounce */
                            rail = pos;
                            stall_cnt++;
                            dir = -1;
                            slow = 0U;
                            state = ST_BOUNCE;
                            blink_led(3U);
                            char evt[96];
                            int len = snprintf(evt, sizeof(evt),
                                "RAIL = %ld steps @step %lu SG=%ld base=%ld\r\n",
                                (long)rail, (unsigned long)step_cnt,
                                (long)sg, (long)sg_base);
                            demo_send(evt, len, sizeof(evt));
                        } else {
                            /* bounce end confirm: reverse and go fast away from the end */
                            if (dir > 0) {
                                rail = pos;   /* refine the far end each cycle */
                                stall_cnt++;
                                blink_led(3U);
                                char evt[96];
                                int len = snprintf(evt, sizeof(evt),
                                    "STALL far @pos %ld SG=%ld base=%ld\r\n",
                                    (long)pos, (long)sg, (long)sg_base);
                                demo_send(evt, len, sizeof(evt));
                            } else {
                                home_cnt++;
                                pos = 0L;
                                stall_cnt++;
                                blink_led(2U);
                                char evt[96];
                                int len = snprintf(evt, sizeof(evt),
                                    "STALL home @pos %ld SG=%ld base=%ld\r\n",
                                    (long)pos, (long)sg, (long)sg_base);
                                demo_send(evt, len, sizeof(evt));
                            }
                            dir = (int8_t)-dir;
                            slow = 0U;
                        }
                    }
                }
            }

            /* ---- bounce: slow down by step count near each end ---- */
            if (state == ST_BOUNCE) {
                if ((dir > 0) && (pos >= (rail - (int32_t)TMC_SLOW_ZONE_STEPS))) {
                    slow = 1U;
                } else if ((dir < 0) && (pos <= (int32_t)TMC_SLOW_ZONE_STEPS)) {
                    slow = 1U;
                }
            }

            if (!tmc_demo_guard_step(&demo_guard, HAL_GetTick(), pos, dir)) {
                Error_Handler();
            }

            /* ---- drive DIR and one STEP pulse at the current rate ---- */
            pos += (int32_t)dir;
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1,
                              (dir > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
            { volatile uint32_t j; for (j = 0; j < dly; j++) { __NOP(); } }
            HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
            { volatile uint32_t j; for (j = 0; j < dly; j++) { __NOP(); } }
        }

        /* ---- Periodic UART status (1 Hz) ---- */
        if ((HAL_GetTick() - last_print) >= 1000UL) {
            last_print = HAL_GetTick();

            uint16_t sg = 0U;
            uint32_t tstep = 0U;
            uint32_t vmv = 0U;
            int16_t tc10 = 0;
            demo_require(tmc2240_stallguard_read(0U, &sg));
            demo_require(tmc2240_readRegister(0U, TMC2240_TSTEP, &tstep));
            demo_require(tmc2240_get_vsupply_mV(0U, &vmv));
            demo_require(tmc2240_get_temperature_c10(0U, &tc10));
            char temperature[16];
            if (!tmc_demo_format_temperature(tc10, temperature, sizeof(temperature))) {
                demo_require(TMC2240_ERROR_RANGE);
            }

            char buf[256];
            int len = snprintf(buf, sizeof(buf),
                "ST:%s POS:%6ld RNG:%6ld SG:%4ld base=%4ld TSTEP:%5lu V:%lumV T:%sC STALLS:%lu HOME:%lu\r\n",
                state_names[state], (long)pos, (long)rail,
                (long)sg, (long)sg_base, (unsigned long)tstep, (unsigned long)vmv,
                temperature,
                (unsigned long)stall_cnt, (unsigned long)home_cnt);
            demo_send(buf, len, sizeof(buf));
        }
    }
#else
    HAL_Delay(1000U);
#endif

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
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

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TMC_CS_GPIO_Port, TMC_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TMC_CS_Pin LD2_Pin */
  GPIO_InitStruct.Pin = TMC_CS_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  demo_gpio_ready = true;
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
#if TMC2240_DEMO_ENABLE_MOTION
  tmc_demo_guard_stop(&demo_guard, TMC_DEMO_COMMUNICATION_FAILURE);
  if (demo_enable_pin_ready) {
      HAL_GPIO_WritePin(TMC2240_DEMO_EN_PORT, TMC2240_DEMO_EN_PIN, GPIO_PIN_SET);
  }
#endif
  if (demo_gpio_ready) {
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
  }
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
