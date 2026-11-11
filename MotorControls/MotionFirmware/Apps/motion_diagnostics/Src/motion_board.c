#include "as5600_stm32_hal.h"
#include "command_dispatch.h"
#include "console.h"
#include "main.h"
#include "motion_board.h"
#include "tmc2240_core.h"
#include "tmc2240_hal.h"

extern I2C_HandleTypeDef hi2c1, hi2c3;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim2, htim5;
extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;

static step_engine_t steps;
static encoder_sampler_t encoders;
static driver_evidence_t drivers;
static uart_transport_t uart;
static motion_executor_t executor;
static as5600_t sensors[2];
static as5600_stm32_hal_t ports[2];
static volatile uint32_t rx_laps;
static uint32_t rx_base, driver_poll_us;
static bool initialized;
static bool sensor_configured[2];
static uint32_t timer_offset[2];
static bool prev_home_pending;
static uint8_t prev_home_axis;
static int64_t home_start_counts[2][2];
static uint32_t prev_rejected;
static uint32_t prev_command_seq;
static void rx_dma_error(DMA_HandleTypeDef *);
static void tx_dma_error(DMA_HandleTypeDef *);

static uint32_t lock(void *context) {
  (void)context;
  const uint32_t p = __get_PRIMASK();
  __disable_irq();
  return p;
}
static void unlock(void *context, uint32_t p) {
  (void)context;
  __set_PRIMASK(p);
}
static uint32_t clock_us(void *context) {
  (void)context;
  return TIM2->CNT;
}
static uint32_t clock_ms(void *context) {
  (void)context;
  return HAL_GetTick();
}
static bool safe_inputs(void *context) {
  (void)context;
  return HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) != GPIO_PIN_RESET;
}
static TIM_TypeDef *timer(uint8_t axis) { return axis == AXIS_Z ? TIM2 : TIM5; }
static void step_out(void *context, uint8_t axis, bool high) {
  (void)context;
  const uint32_t pin = axis == AXIS_Z ? TMC_STEP0_Pin : TMC_STEP1_Pin;
  GPIOC->BSRR = high ? pin : pin << 16U;
}
static void dir_out(void *context, uint8_t axis, int8_t direction) {
  (void)context;
  const uint32_t pin = axis == AXIS_Z ? TMC_DIR0_Pin : TMC_DIR1_Pin;
  GPIOC->BSRR = direction > 0 ? pin : pin << 16U;
}
static void schedule(void *context, uint8_t axis, uint32_t deadline,
                     bool enable) {
  (void)context;
  TIM_TypeDef *t = timer(axis);
  t->DIER &= ~TIM_DIER_CC1IE;
  if (enable) {
    /* Translate the common TIM2 timebase into TIM5's free-running domain. */
    t->CCR1 = deadline + timer_offset[axis];
    t->SR = ~TIM_SR_CC1IF;
    t->DIER |= TIM_DIER_CC1IE;
    /* Never wait a full timer wrap if programming the compare took too long.
     * The engine drops late rises and always safely lowers an existing pulse.
     */
    if ((int32_t)(deadline + timer_offset[axis] - t->CNT) <= 0) {
      t->EGR = TIM_EGR_CC1G;
    }
  }
}
void motion_board_timer_irq(uint8_t motor) {
  TIM_TypeDef *t = motor == 0U ? TIM2 : TIM5;
  if ((t->SR & TIM_SR_CC1IF) != 0U && (t->DIER & TIM_DIER_CC1IE) != 0U) {
    t->SR = ~TIM_SR_CC1IF;
    const uint8_t axis = (uint8_t)(motor == 0U ? AXIS_Z : AXIS_X);
    step_engine_irq(&steps, axis, t->CNT - timer_offset[axis]);
  }
}
static bool raw_read(void *context, uint8_t axis, uint16_t *raw) {
  (void)context;
  return sensor_configured[axis] &&
         as5600_read_raw_angle(&sensors[axis], raw, 2U) == AS5600_OK;
}
static bool health_read(void *context, uint8_t axis, uint8_t *status,
                        uint8_t *agc) {
  (void)context;
  if (!sensor_configured[axis]) {
    return false;
  }
  as5600_diagnostics_t sample;
  if (as5600_read_diagnostics(&sensors[axis], &sample, 2U) != AS5600_OK) {
    return false;
  }
  *status = sample.status;
  *agc = sample.agc;
  return true;
}
static bool conf_read(void *context, uint8_t axis, uint16_t *conf) {
  (void)context;
  uint8_t bytes[2];
  I2C_HandleTypeDef *bus = axis == AXIS_X ? &hi2c1 : &hi2c3;
  /* Foreground-only, serialized with the AS5600 driver on each physical bus.
   * Retain the complete high-byte-first word, including factory/reserved bits.
   */
  if (HAL_I2C_Mem_Read(bus, 0x6CU, 0x07U, I2C_MEMADD_SIZE_8BIT, bytes, 2U,
                       2U) != HAL_OK) {
    return false;
  }
  *conf = (uint16_t)((uint16_t)((uint16_t)bytes[0] << 8U) | bytes[1]);
  return true;
}
static bool driver_read(void *context, uint8_t axis, driver_sample_t *s) {
  (void)context;
  const uint16_t motor = motion_default_config.axis[axis].motor;
  uint32_t chop, gconf;
  if (tmc2240_get_microstep_counter(motor, &s->mscnt) != TMC2240_OK ||
      tmc2240_stallguard_read(motor, &s->sg) != TMC2240_OK ||
      tmc2240_driver_status(motor, &s->status) != TMC2240_OK ||
      tmc2240_check_faults(motor, &s->gstat) != TMC2240_OK ||
      tmc2240_readRegister(motor, TMC2240_SG4_IND, &s->sg_ind) != TMC2240_OK ||
      tmc2240_readRegister(motor, TMC2240_TSTEP, &s->tstep) != TMC2240_OK ||
      tmc2240_readRegister(motor, TMC2240_CHOPCONF, &chop) != TMC2240_OK ||
      tmc2240_readRegister(motor, TMC2240_GCONF, &gconf) != TMC2240_OK ||
      tmc2240_hal_getSPIStatus(motor, &s->spi_status) != TMC2240_OK) {
    return false;
  }
  s->mode = (uint8_t)(((chop >> 24U) & 15U) | (((chop >> 28U) & 3U) << 4U) |
                      (((gconf >> 4U) & 1U) << 6U));
  s->fault = (chop & 15U) == 0U || (gconf & 0x1001CU) != 4U;
  return true;
}
static bool tx_start(void *context, const uint8_t *bytes, uint16_t size) {
  (void)context;
  /* The vendor shared DMA-error callback ends both UART directions. Bind
   * source-specific callbacks before permitting this stream's interrupt. */
  HAL_NVIC_DisableIRQ(DMA1_Stream6_IRQn);
  const bool started = HAL_UART_Transmit_DMA(&huart2, bytes, size) == HAL_OK;
  if (started) {
    huart2.hdmatx->XferErrorCallback = tx_dma_error;
  }
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* Error/completion can preempt startup. A completed slot is already freed
   * by its callback; an aborted start must leave the frame queued for retry. */
  return started && huart2.gState == HAL_UART_STATE_BUSY_TX;
}
static void received(void *context, const vision_cmd_t *command) {
  (void)context;
  command_dispatch(&executor, command, clock_us(NULL));
}

static void publish_rx(void *context) {
  (void)context;
  if (!uart.rx_running) {
    return;
  }
  const uint32_t p = lock(NULL);
  uint32_t laps = rx_laps;
  const uint32_t flag = __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart2_rx);
  /* Include a wrap pending in DMA before its IRQ can update rx_laps. */
  const bool before = __HAL_DMA_GET_FLAG(&hdma_usart2_rx, flag) != RESET;
  uint32_t offset = UART_RX_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
  const bool after = __HAL_DMA_GET_FLAG(&hdma_usart2_rx, flag) != RESET;
  if (before != after) {
    offset = UART_RX_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
  }
  if (after) {
    ++laps;
  }
  if (offset == UART_RX_SIZE) {
    offset = 0U;
  }
  uint32_t producer = rx_base + laps * UART_RX_SIZE + offset;
  /* An abort can clear an unserviced wrap flag; frozen NDTR still exposes
   * the new modulo position. Never regress into the preceding DMA lap. */
  if ((int32_t)(producer - uart.producer) < 0) {
    ++rx_laps;
    producer += UART_RX_SIZE;
  }
  uart_transport_rx_publish(&uart, producer);
  unlock(NULL, p);
}
static bool start_rx(void *context) {
  (void)context;
  rx_base = uart.producer;
  rx_laps = 0U;
  HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
  const bool started =
      HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart.rx, UART_RX_SIZE) == HAL_OK;
  if (started) {
    huart2.hdmarx->XferErrorCallback = rx_dma_error;
  }
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  return started;
}
static bool abort_rx(void *context) {
  (void)context;
  HAL_NVIC_DisableIRQ(DMA1_Stream5_IRQn);
  const bool wrap =
      __HAL_DMA_GET_FLAG(&hdma_usart2_rx,
                         __HAL_DMA_GET_TC_FLAG_INDEX(&hdma_usart2_rx)) != RESET;
  const bool stopped = HAL_UART_AbortReceive(&huart2) == HAL_OK;
  if (stopped && wrap) {
    ++rx_laps;
  }
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  return stopped;
}
static bool abort_tx(void *context) {
  (void)context;
  return HAL_UART_AbortTransmit(&huart2) == HAL_OK;
}
static void drain_rx(void *context) {
  (void)context;
  uart_transport_poll(&uart, clock_us(NULL));
}
static const uart_recovery_io_t recovery_io = {NULL,     abort_rx, publish_rx,
                                               drain_rx, start_rx, abort_tx};
static void rx_dma_error(DMA_HandleTypeDef *dma) {
  (void)dma;
  huart2.ErrorCode |= HAL_UART_ERROR_DMA;
  uart_transport_error(&uart, true, false);
}
static void tx_dma_error(DMA_HandleTypeDef *dma) {
  (void)dma;
  huart2.ErrorCode |= HAL_UART_ERROR_DMA;
  uart_transport_error(&uart, false, true);
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *h, uint16_t size) {
  (void)size;
  if (h == &huart2 && uart.rx_running) {
    if (HAL_UARTEx_GetRxEventType(h) == HAL_UART_RXEVENT_TC) {
      ++rx_laps;
    }
    publish_rx(NULL);
  }
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *h) {
  if (h == &huart2) {
    uart_transport_tx_complete(&uart);
  }
}
void HAL_UART_ErrorCallback(UART_HandleTypeDef *h) {
  if (h == &huart2) {
    const uint32_t rx_line_errors = HAL_UART_ERROR_ORE | HAL_UART_ERROR_FE |
                                    HAL_UART_ERROR_NE | HAL_UART_ERROR_PE;
    const uint32_t dma_errors =
        HAL_DMA_ERROR_TE | HAL_DMA_ERROR_FE | HAL_DMA_ERROR_DME;
    const bool dma_error = (h->ErrorCode & HAL_UART_ERROR_DMA) != 0U;
    const bool rx_error =
        (h->ErrorCode & rx_line_errors) != 0U ||
        (dma_error && (HAL_DMA_GetError(h->hdmarx) & dma_errors) != 0U);
    const bool tx_error =
        dma_error && (HAL_DMA_GetError(h->hdmatx) & dma_errors) != 0U;
    uart_transport_error(&uart, rx_error, tx_error);
  }
}
void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  if (initialized && pin == B1_Pin) {
    step_engine_abort(&steps);
    console_printf("[ABORT] B1 (PC13) pressed -> step engine aborted\n");
  }
}

static bool configure_motor(uint8_t axis) {
  const uint16_t motor = motion_default_config.axis[axis].motor;
  uint8_t faults;
  uint32_t current;
  if (tmc2240_hal_testConnection(motor) != TMC2240_OK ||
      tmc2240_check_faults(motor, &faults) != TMC2240_OK) {
    return false;
  }
  console_printf("[TMC%u] startup GSTAT=%02X\n", (unsigned)motor, faults);
  /* Startup acknowledgement only. Runtime faults are captured, never
   * auto-cleared. */
  if (faults != 0U && tmc2240_clear_faults(motor, faults) != TMC2240_OK) {
    return false;
  }
  if (tmc2240_calculateCurrent(axis == AXIS_Z ? 600U : 500U, 12000U, 0U, 0U,
                               100U, 6U, 4U, &current) != TMC2240_OK) {
    return false;
  }
  const TMC2240_MotorConfig_t cfg = {.gconf = 4U,
                                     .drv_conf = 0x20U,
                                     .global_scaler = 0U,
                                     .ihold_irun = current,
                                     .chopconf = 0x04410150U,
                                     .pwmconf = 0xC40C001DU};
  if (tmc2240_hal_configureMotor(motor, &cfg) != TMC2240_OK ||
      tmc2240_writeRegisterVerified(motor, TMC2240_TPOWERDOWN, 10U) !=
          TMC2240_OK ||
      tmc2240_writeRegisterVerified(motor, TMC2240_TPWMTHRS, 0U) !=
          TMC2240_OK ||
      tmc2240_writeRegisterVerified(motor, TMC2240_TCOOLTHRS, 1500U) !=
          TMC2240_OK ||
      tmc2240_writeRegisterVerified(motor, TMC2240_THIGH, 0U) != TMC2240_OK ||
      tmc2240_writeRegisterVerified(motor, TMC2240_COOLCONF, 0U) !=
          TMC2240_OK ||
      tmc2240_writeRegisterVerified(
          motor, TMC2240_SG4_THRS,
          0x300U | motion_default_config.axis[axis].sg_threshold) !=
          TMC2240_OK ||
      tmc2240_hal_activateMotor(motor, 3U) != TMC2240_OK) {
    return false;
  }
  HAL_GPIO_WritePin(GPIOC, axis == AXIS_Z ? TMC_ENN0_Pin : TMC_ENN1_Pin,
                    GPIO_PIN_RESET);
  return true;
}

static void recover_bus(I2C_HandleTypeDef *bus, uint8_t axis) {
  GPIO_TypeDef *scl = axis == AXIS_X ? GPIOB : GPIOA;
  GPIO_TypeDef *sda = axis == AXIS_X ? GPIOB : GPIOC;
  const uint16_t scl_pin = GPIO_PIN_8, sda_pin = GPIO_PIN_9;
  (void)HAL_I2C_DeInit(bus);
  HAL_GPIO_WritePin(scl, scl_pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(sda, sda_pin, GPIO_PIN_SET);
  GPIO_InitTypeDef g = {.Mode = GPIO_MODE_OUTPUT_OD,
                        .Pull = GPIO_PULLUP,
                        .Speed = GPIO_SPEED_FREQ_LOW,
                        .Pin = scl_pin};
  HAL_GPIO_Init(scl, &g);
  g.Pin = sda_pin;
  HAL_GPIO_Init(sda, &g);
  /* Startup-only bus clear with conservative millisecond clocks; no STEP
   * active. */
  for (unsigned i = 0U;
       i < 9U && HAL_GPIO_ReadPin(sda, sda_pin) == GPIO_PIN_RESET; ++i) {
    HAL_GPIO_WritePin(scl, scl_pin, GPIO_PIN_RESET);
    HAL_Delay(1U);
    HAL_GPIO_WritePin(scl, scl_pin, GPIO_PIN_SET);
    HAL_Delay(1U);
  }
  HAL_GPIO_WritePin(scl, scl_pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(sda, sda_pin, GPIO_PIN_RESET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(scl, scl_pin, GPIO_PIN_SET);
  HAL_Delay(1U);
  HAL_GPIO_WritePin(sda, sda_pin, GPIO_PIN_SET);
  HAL_Delay(1U);
  (void)HAL_I2C_Init(bus);
}

static int64_t abs64(int64_t v) { return v < 0 ? -v : v; }

static void monitor_diagnostics(void) {
  if (executor.home_pending) {
    const uint8_t ax = executor.home.axis;
    if (!prev_home_pending || ax != prev_home_axis) {
      prev_home_pending = true;
      prev_home_axis = ax;
      home_start_counts[ax][AXIS_X] = encoders.axis[AXIS_X].unwrap.total_counts;
      home_start_counts[ax][AXIS_Z] = encoders.axis[AXIS_Z].unwrap.total_counts;
      console_printf(
          "[HOME] starting axis=%c (dir_sign=%d enc_sign=%d sg_thrs=%u)\n",
          ax == AXIS_X ? 'X' : 'Z',
          (int)motion_default_config.axis[ax].dir_sign,
          (int)motion_default_config.axis[ax].encoder_sign,
          (unsigned)motion_default_config.axis[ax].sg_threshold);
    }
  } else if (prev_home_pending) {
    prev_home_pending = false;
    for (uint8_t ax = 0U; ax < 2U; ++ax) {
      const diag_home_t *r = &executor.home_record.data.home[ax];
      const char name = ax == AXIS_X ? 'X' : 'Z';
      const int32_t net_steps = r->home_emitted_origin_steps;
      const int64_t enc_delta =
          r->home_offset_counts - home_start_counts[ax][ax];
      console_printf("[HOME] axis=%c result=%u seek_steps=%ld latch_steps=%ld "
                     "net_steps=%ld "
                     "sg_base=%u sg_trig=%u enc_delta=%ld\n",
                     name, (unsigned)r->result, (long)r->seek_emitted_steps,
                     (long)r->latch_emitted_steps, (long)net_steps,
                     (unsigned)r->sg_baseline, (unsigned)r->sg_trigger,
                     (long)enc_delta);
      if (abs64(net_steps) >= 1600) {
        const int64_t expected_mag = (abs64(net_steps) * 4096) / 3200;
        const int64_t signed_enc =
            enc_delta * motion_default_config.axis[ax].encoder_sign;
        if (abs64(enc_delta) < expected_mag / 4) {
          const uint8_t other = (uint8_t)(ax ^ 1U);
          const int64_t other_delta = encoders.axis[other].unwrap.total_counts -
                                      home_start_counts[ax][other];
          console_printf(
              "[WARN] axis=%c encoder moved only %ld counts for %ld steps "
              "(check magnet/I2C%s)\n",
              name, (long)enc_delta, (long)net_steps,
              abs64(other_delta) > expected_mag / 2
                  ? " -> OTHER AXIS ENCODER TURNED! Swap I2C1/I2C3 cables"
                  : "");
        } else if ((net_steps < 0 && signed_enc > 0) ||
                   (net_steps > 0 && signed_enc < 0)) {
          console_printf("[WARN] axis=%c encoder sign opposes step delta "
                         "(steps=%ld, enc_delta=%ld, encoder_sign=%d) -> flip "
                         "MOTION_%c_ENCODER_SIGN\n",
                         name, (long)net_steps, (long)enc_delta,
                         (int)motion_default_config.axis[ax].encoder_sign,
                         name);
        }
      }
    }
    console_printf("[HOME] epoch=%lu state=%u status_bits=0x%08lX\n",
                   (unsigned long)executor.home_epoch, (unsigned)executor.state,
                   (unsigned long)executor.home_record.status_bits);
  }

  if (executor.rejected != prev_rejected) {
    prev_rejected = executor.rejected;
    console_printf("[REJECT] total=%lu state=%u homed=0x%X faults=0x%08lX "
                   "bounds_confirmed=%u\n",
                   (unsigned long)executor.rejected, (unsigned)executor.state,
                   (unsigned)executor.homed_mask,
                   (unsigned long)executor.faults,
                   (unsigned)motion_default_config.bounds_confirmed);
  }

  if (!executor.active && executor.result.command_seq != 0U &&
      executor.result.command_seq != prev_command_seq) {
    prev_command_seq = executor.result.command_seq;
    console_printf(
        "[PICK] cmd_seq=%lu obj=%u x=%d(0.1mm) phases=%u status=0x%08lX\n",
        (unsigned long)executor.result.command_seq,
        (unsigned)executor.result.command.obj_id,
        (int)executor.result.command.x_01mm,
        (unsigned)executor.result.phase_count,
        (unsigned long)executor.result.status_bits);
  }
}

void motion_board_init(void) {
  console_init();
  console_printf(
      "\n==============================================================\n");
  console_printf(
      " [PROFILE] motion_diagnostics (Binary USART2 0xAA55/0xD3 @ 115200)\n");
  console_printf(" [CFG] bounds_confirmed=%u clamp_x=%u safe_z=%ld.%ldmm "
                 "pick_depth=%ld.%ldmm\n",
                 (unsigned)motion_default_config.bounds_confirmed,
                 (unsigned)motion_default_config.clamp_x,
                 (long)(motion_default_config.safe_z_01mm / 10),
                 (long)(motion_default_config.safe_z_01mm % 10),
                 (long)(motion_default_config.pick_depth_01mm / 10),
                 (long)(motion_default_config.pick_depth_01mm % 10));
  console_printf(
      " [CFG] X(dir=%+d,enc=%+d) Z(dir=%+d,enc=%+d) mscnt_qualified=%u\n",
      (int)motion_default_config.axis[AXIS_X].dir_sign,
      (int)motion_default_config.axis[AXIS_X].encoder_sign,
      (int)motion_default_config.axis[AXIS_Z].dir_sign,
      (int)motion_default_config.axis[AXIS_Z].encoder_sign,
      (unsigned)motion_default_config.mscnt_qualified);
  console_printf(
      "==============================================================\n");
  TIM2->CNT = 0U;
  TIM5->CNT = 0U;
  (void)HAL_TIM_Base_Start(&htim2);
  (void)HAL_TIM_Base_Start(&htim5);
  timer_offset[AXIS_Z] = 0U;
  timer_offset[AXIS_X] = TIM5->CNT - TIM2->CNT;
  const step_engine_io_t step_io = {NULL,    lock,     unlock,  step_out,
                                    dir_out, schedule, clock_us};
  const encoder_io_t encoder_io = {NULL,     raw_read, health_read,
                                   clock_us, clock_ms, conf_read};
  const driver_io_t driver_io = {NULL, driver_read};
  const uart_transport_io_t uart_io = {NULL, lock, unlock, tx_start, received};
  step_engine_init(&steps, &motion_default_config, step_io);
  encoder_sampler_init(&encoders, &motion_default_config, encoder_io);
  driver_evidence_init(&drivers, driver_io);
  uart_transport_init(&uart, uart_io);
  motion_executor_init(&executor, &motion_default_config, &steps, &encoders,
                       &drivers, &uart);
  executor.safe_inputs = safe_inputs;
  const TMC2240_SPIConfig_t motor_cfg[2] = {{&hspi2, GPIOC, TMC_CS0_Pin, 1U},
                                            {&hspi2, GPIOC, TMC_CS1_Pin, 1U}};
  const bool bus_ok = tmc2240_hal_init(motor_cfg, 2U) == TMC2240_OK;
  HAL_Delay(10U);
  I2C_HandleTypeDef *const buses[2] = {&hi2c1, &hi2c3};
  for (uint8_t i = 0U; i < 2U; ++i) {
    recover_bus(buses[i], i);
    as5600_config_t profile, readback;
    (void)as5600_default_config(&profile);
    profile.slow_filter = AS5600_SLOW_FILTER_2X;
    const bool encoder_ok =
        as5600_stm32_hal_init(&sensors[i], &ports[i], buses[i], NULL) ==
            AS5600_OK &&
        as5600_write_config(&sensors[i], &profile, 10U) == AS5600_OK &&
        as5600_read_config(&sensors[i], &readback, 2U) == AS5600_OK &&
        readback.slow_filter == AS5600_SLOW_FILTER_2X &&
        readback.power == AS5600_POWER_NORMAL &&
        readback.hysteresis == AS5600_HYSTERESIS_OFF &&
        readback.fast_filter == AS5600_FAST_FILTER_OFF && !readback.watchdog;
    uint16_t conf = 0U;
    const bool conf_ok = conf_read(NULL, i, &conf);
    encoder_sampler_config(&encoders.axis[i], conf_ok, conf);
    const bool profile_ok = encoder_ok && conf_ok &&
                            (conf & ENCODER_CONF_MASK) == ENCODER_CONF_EXPECTED;
    const bool motor_ok = bus_ok && configure_motor(i);
    sensor_configured[i] = profile_ok;
    console_printf("[BOOT] axis=%c sensor=%s motor=%s; 2000 steps/s cap\n",
                   i == AXIS_X ? 'X' : 'Z', profile_ok ? "ok" : "fault",
                   motor_ok ? "ok" : "fault");
    if (!profile_ok) {
      encoders.axis[i].valid = false;
    }
    (void)driver_evidence_poll(&drivers, i);
  }
  encoder_sampler_poll(&encoders, true);
  initialized = true;
  uart_transport_recover(&uart, recovery_io);
  motion_executor_boot(&executor, clock_us(NULL));
}
void motion_board_poll(void) {
  step_engine_poll(&steps);
  uart_transport_recover(&uart, recovery_io);
  uart_transport_poll(&uart, clock_us(NULL));
  encoder_sampler_poll(&encoders, false);
  uint32_t now = clock_us(NULL);
  if (now - driver_poll_us >= 20000U && executor.state != EXEC_HOMING) {
    driver_poll_us = now;
    (void)driver_evidence_poll(&drivers, 0U);
    (void)driver_evidence_poll(&drivers, 1U);
  }
  now = clock_us(NULL);
  motion_executor_poll(&executor, now);
  monitor_diagnostics();
  uart_transport_poll(&uart, clock_us(NULL));
}

/* ------------------------------------------------------------------------- */
/* Shared Core/ entry-point bindings (fw_app.h)                              */
/* ------------------------------------------------------------------------- */
#include "fw_app.h"

void fw_app_init(void) { motion_board_init(); }

void fw_app_poll(void) { motion_board_poll(); }

void fw_app_tim_irq(uint8_t motor) { motion_board_timer_irq(motor); }

bool fw_app_usart2_irq(void) { return false; }

void fw_app_error_handler(void) {
  HAL_GPIO_WritePin(GPIOC, TMC_STEP0_Pin | TMC_STEP1_Pin, GPIO_PIN_RESET);
}
