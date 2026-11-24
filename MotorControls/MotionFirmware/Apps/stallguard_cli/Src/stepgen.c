/**
 * @file    stepgen.c
 * @brief   Timer-interrupt STEP/DIR pulse generation (see stepgen.h).
 */
#include "stepgen.h"

#include "motion_profile.h"

#define STEPGEN_MIN_TICKS 1000U /* 5 us at 200 MHz: floor for bad parameters   \
                                 */
#define STEPGEN_MAX_TICKS 4000000000U

typedef struct {
  stepgen_hw_t hw;
  step_core_t core;
  float timer_hz;
  uint32_t step_level; /* STEP pin level; with dedge every edge is a step */
  bool ready;
#ifdef STEPGEN_CHARACTERIZATION
  stepgen_char_stats_t cstats;
  uint32_t expected_edge_us;
  uint32_t planned_dt_us;
  uint32_t last_toggle_us;
  int8_t   last_dir_out;
  bool     dir_known;
  bool     inhibited;
  bool     was_cruise;
#endif
} stepgen_t;

static stepgen_t g_sg[STEPGEN_MAX_AXES];

#ifdef STEPGEN_CHARACTERIZATION
static uint32_t (*g_char_now_us)(void);

static inline uint32_t char_now_us(void) {
  return (g_char_now_us != NULL) ? g_char_now_us() : 0U;
}
#endif

static inline uint32_t lock_irq(void) {
  const uint32_t primask = __get_PRIMASK();

  __disable_irq();
  return primask;
}

static inline void unlock_irq(uint32_t primask) { __set_PRIMASK(primask); }

static stepgen_t *axis(uint8_t ax) {
  return ((ax < STEPGEN_MAX_AXES) && g_sg[ax].ready) ? &g_sg[ax] : NULL;
}

static uint32_t timer_input_clock(const TIM_TypeDef *tim) {
  RCC_ClkInitTypeDef clk;
  uint32_t latency;
  bool apb2 = (tim == TIM1) || (tim == TIM8) || (tim == TIM15) ||
              (tim == TIM16) || (tim == TIM17);
  uint32_t pclk;
  uint32_t div;
  uint32_t div1;

  HAL_RCC_GetClockConfig(&clk, &latency);
  pclk = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
  div = apb2 ? clk.APB2CLKDivider : clk.APB1CLKDivider;
  div1 = apb2 ? RCC_APB2_DIV1 : RCC_APB1_DIV1;
  /* APB timers run at 2x PCLK whenever the APB prescaler is not 1. */
  return (div == div1) ? pclk : 2U * pclk;
}

static inline void dir_out(const stepgen_t *s, int8_t dir) {
  s->hw.dir_port->BSRR =
      (dir > 0) ? (uint32_t)s->hw.dir_pin : ((uint32_t)s->hw.dir_pin << 16U);
}

static inline void step_toggle(stepgen_t *s) {
  s->step_level ^= 1U;
  s->hw.step_port->BSRR = (s->step_level != 0U)
                              ? (uint32_t)s->hw.step_pin
                              : ((uint32_t)s->hw.step_pin << 16U);
}

static inline uint32_t ticks_for(const stepgen_t *s, float v) {
  float t = s->timer_hz / v;

  if (t < (float)STEPGEN_MIN_TICKS) {
    t = (float)STEPGEN_MIN_TICKS;
  }
  if (t > (float)STEPGEN_MAX_TICKS) {
    t = (float)STEPGEN_MAX_TICKS;
  }
  return (uint32_t)t;
}

static void timer_stop(stepgen_t *s) {
  TIM_TypeDef *t = s->hw.tim;

  t->CR1 &= ~TIM_CR1_CEN;
  t->DIER &= ~TIM_DIER_UIE;
  t->SR = ~TIM_SR_UIF;
  NVIC_ClearPendingIRQ(s->hw.irq);
}

/* Called with interrupts locked. The first edge comes one interval later, far
 * more than the 20 ns DIR setup time of the TMC2240. */
static void timer_start(stepgen_t *s, float v) {
  TIM_TypeDef *t = s->hw.tim;
  uint32_t ticks = ticks_for(s, v);

#ifdef STEPGEN_CHARACTERIZATION
  if (s->inhibited) {
    s->cstats.edges_inhibited++;
    step_core_cmd_halt(&s->core);
    timer_stop(s);
    return;
  }
  /* Enforce >= 20 us DIR setup/hold on stationary starts and reversals. */
  {
    const uint32_t min_start_ticks = (uint32_t)(s->timer_hz * 20e-6f);
    if ((!s->dir_known || (s->last_dir_out != s->core.step_dir)) &&
        (ticks < min_start_ticks)) {
      ticks = min_start_ticks;
    }
  }
  s->last_dir_out = s->core.step_dir;
  s->dir_known = true;
  s->planned_dt_us = (uint32_t)(((uint64_t)ticks * 1000000ULL) /
                                (uint64_t)s->timer_hz);
  if (s->planned_dt_us < 5U) {
    s->planned_dt_us = 5U;
  }
  s->expected_edge_us = char_now_us() + s->planned_dt_us;
  s->was_cruise = false;
#endif
  dir_out(s, s->core.step_dir);
  t->CR1 &= ~TIM_CR1_CEN;
  t->CNT = 0U;
  t->ARR = ticks - 1U;
  t->SR = ~TIM_SR_UIF;
  NVIC_ClearPendingIRQ(s->hw.irq);
  t->DIER |= TIM_DIER_UIE;
  t->CR1 |= TIM_CR1_CEN;
}

void stepgen_irq_handler(uint8_t ax) {
  stepgen_t *s = &g_sg[ax];
  TIM_TypeDef *t = s->hw.tim;
  bool more = false;
  bool dir_changed = false;
  float v = 0.0f;

  if ((t->SR & TIM_SR_UIF) == 0U) {
    return;
  }
  t->SR = ~TIM_SR_UIF;
#ifdef STEPGEN_CHARACTERIZATION
  if (s->inhibited) {
    s->cstats.edges_inhibited++;
    step_core_cmd_halt(&s->core);
    timer_stop(s);
    return;
  }
  {
    const uint32_t now_us = char_now_us();
    const int32_t late_signed = (int32_t)(now_us - s->expected_edge_us);
    const uint32_t lateness = (late_signed > 0) ? (uint32_t)late_signed : 0U;
    const uint32_t q_dt = s->planned_dt_us / 4U;
    const uint32_t lateness_limit = (q_dt < 20U) ? q_dt : 20U;
    if (lateness > s->cstats.max_lateness_us) {
      s->cstats.max_lateness_us = lateness;
    }
    if ((lateness_limit > 0U && lateness > lateness_limit) ||
        (lateness > 20U)) {
      s->cstats.late_edges++;
      s->cstats.lateness_fault = true;
    }
    if (step_core_expire(&s->core, &more, &v, &dir_changed)) {
      step_toggle(s);
      s->cstats.edges_emitted++;
      if (s->cstats.edges_emitted == 1U) {
        s->cstats.first_edge_us = now_us;
      } else {
        const uint32_t dt = now_us - s->last_toggle_us;
        if (s->cstats.min_dt_us == 0U || dt < s->cstats.min_dt_us) {
          s->cstats.min_dt_us = dt;
        }
        if (dt > s->cstats.max_dt_us) {
          s->cstats.max_dt_us = dt;
        }
      }
      s->last_toggle_us = now_us;
      s->cstats.last_edge_us = now_us;
      const bool in_cruise = mp_at_cruise(&s->core.mp);
      if (in_cruise && !s->was_cruise) {
        s->cstats.ramp_up_end_us = now_us;
        s->cstats.ramp_up_end_edges = s->cstats.edges_emitted;
        s->was_cruise = true;
      } else if (!in_cruise && s->was_cruise) {
        s->cstats.cruise_end_us = now_us;
        s->cstats.cruise_end_edges = s->cstats.edges_emitted;
        s->was_cruise = false;
      }
    }
    if (!more) {
      if (s->was_cruise) {
        s->cstats.cruise_end_us = now_us;
        s->cstats.cruise_end_edges = s->cstats.edges_emitted;
        s->was_cruise = false;
      }
      timer_stop(s);
      return;
    }
    uint32_t next_ticks = ticks_for(s, v);
    if (dir_changed) {
      const uint32_t min_dir_ticks = (uint32_t)(s->timer_hz * 20e-6f);
      if (next_ticks < min_dir_ticks) {
        next_ticks = min_dir_ticks;
      }
      dir_out(s, s->core.step_dir);
      s->last_dir_out = s->core.step_dir;
      s->dir_known = true;
    }
    s->planned_dt_us = (uint32_t)(((uint64_t)next_ticks * 1000000ULL) /
                                  (uint64_t)s->timer_hz);
    if (s->planned_dt_us < 5U) {
      s->planned_dt_us = 5U;
    }
    /* Never issue catch-up pulses on a missed period: reset CNT to 0 and
     * schedule full next_ticks from now. */
    t->CNT = 0U;
    t->ARR = next_ticks - 1U;
    s->expected_edge_us = now_us + s->planned_dt_us;
  }
#else
  if (step_core_expire(&s->core, &more, &v, &dir_changed)) {
    step_toggle(s);
  }
  if (!more) {
    timer_stop(s);
    return;
  }
  if (dir_changed) {
    dir_out(s, s->core.step_dir); /* after the edge: >20 ns hold time */
  }
  /* ARR is not preloaded, so the new interval applies to the running
   * period. If this ISR ran so late that the counter already passed the new
   * value, force the update now instead of waiting for a 32-bit wrap. */
  t->ARR = ticks_for(s, v) - 1U;
  if (t->CNT > t->ARR) {
    t->EGR = TIM_EGR_UG;
  }
#endif
}

void stepgen_init(uint8_t ax, const stepgen_hw_t *hw) {
  stepgen_t *s;
  TIM_TypeDef *t;

  if ((ax >= STEPGEN_MAX_AXES) || (hw == NULL)) {
    return;
  }
  s = &g_sg[ax];
  s->ready = false;
  s->hw = *hw;
  t = hw->tim;
  /* Run timer at full APB timer clock (84 MHz, PSC = 0) regardless of CubeMX
   * default. */
  t->CR1 &=
      ~(TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_OPM | TIM_CR1_URS | TIM_CR1_UDIS);
  t->PSC = 0U;
  t->EGR = TIM_EGR_UG;
  t->DIER = 0U;
  t->SR = 0U;
  step_core_init(&s->core, 1000.0f, 1000.0f, 100.0f);
  s->timer_hz = (float)timer_input_clock(t) / (float)(t->PSC + 1U);
  s->step_level = ((hw->step_port->ODR & hw->step_pin) != 0U) ? 1U : 0U;
  NVIC_ClearPendingIRQ(hw->irq);
  s->ready = true;
}

void stepgen_set_vmin(uint8_t ax, float vmin) {
  stepgen_t *s = axis(ax);
  uint32_t p;

  if (s == NULL) {
    return;
  }
  p = lock_irq();
  s->core.vmin = (vmin >= 1.0f) ? vmin : 1.0f;
  unlock_irq(p);
}

void stepgen_move_to(uint8_t ax, int32_t target, float vmax, float accel) {
  stepgen_t *s = axis(ax);
  float v = 0.0f;
  uint32_t p;

  if (s == NULL) {
    return;
  }
  p = lock_irq();
  if (step_core_cmd_move(&s->core, target, vmax, accel, &v)) {
    timer_start(s, v);
  }
  unlock_irq(p);
}

void stepgen_run(uint8_t ax, int8_t dir, float vmax, float accel) {
  stepgen_t *s = axis(ax);
  float v = 0.0f;
  uint32_t p;

  if (s == NULL) {
    return;
  }
  p = lock_irq();
  if (step_core_cmd_run(&s->core, dir, vmax, accel, &v)) {
    timer_start(s, v);
  }
  unlock_irq(p);
}

void stepgen_stop(uint8_t ax) {
  stepgen_t *s = axis(ax);
  uint32_t p;

  if (s == NULL) {
    return;
  }
  p = lock_irq();
  step_core_cmd_stop(&s->core);
  unlock_irq(p);
}

void stepgen_halt(uint8_t ax) {
  stepgen_t *s = axis(ax);
  uint32_t p;

  if (s == NULL) {
    return;
  }
  p = lock_irq();
  step_core_cmd_halt(&s->core);
  timer_stop(s);
  unlock_irq(p);
}

bool stepgen_busy(uint8_t ax) {
  const stepgen_t *s = axis(ax);

  return (s != NULL) && *(const volatile bool *)&s->core.running;
}

int32_t stepgen_position(uint8_t ax) {
  const stepgen_t *s = axis(ax);

  return (s != NULL) ? *(const volatile int32_t *)&s->core.pos : 0;
}

bool stepgen_set_position(uint8_t ax, int32_t pos) {
  stepgen_t *s = axis(ax);
  bool ok = false;
  uint32_t p;

  if (s == NULL) {
    return false;
  }
  p = lock_irq();
  if (!s->core.running) {
    s->core.pos = pos;
    ok = true;
  }
  unlock_irq(p);
  return ok;
}

float stepgen_velocity(uint8_t ax) {
  const stepgen_t *s = axis(ax);
  float v = 0.0f;
  uint32_t p;

  if (s == NULL) {
    return 0.0f;
  }
  p = lock_irq();
  if (s->core.running) {
    v = s->core.mp.v * (float)s->core.step_dir;
  }
  unlock_irq(p);
  return v;
}

bool stepgen_at_cruise(uint8_t ax) {
  const stepgen_t *s = axis(ax);
  bool cruise;
  uint32_t p;

  if (s == NULL) {
    return false;
  }
  p = lock_irq();
  cruise = s->core.running && mp_at_cruise(&s->core.mp);
  unlock_irq(p);
  return cruise;
}

void stepgen_set_limits(uint8_t ax, int32_t lo, int32_t hi) {
  stepgen_t *s = axis(ax);
  uint32_t p;

  if (s == NULL) {
    return;
  }
  p = lock_irq();
  s->core.lim_lo = lo;
  s->core.lim_hi = hi;
  unlock_irq(p);
}

bool stepgen_take_limit_hit(uint8_t ax) {
  stepgen_t *s = axis(ax);
  bool hit;
  uint32_t p;

  if (s == NULL) {
    return false;
  }
  p = lock_irq();
  hit = s->core.limit_hit;
  s->core.limit_hit = false;
  unlock_irq(p);
  return hit;
}

uint32_t stepgen_timer_hz(uint8_t ax) {
  const stepgen_t *s = axis(ax);

  return (s != NULL) ? (uint32_t)s->timer_hz : 0U;
}

#ifdef STEPGEN_CHARACTERIZATION
void stepgen_char_set_time_fn(uint32_t (*now_us_fn)(void)) {
  g_char_now_us = now_us_fn;
}

void stepgen_char_set_inhibit(uint8_t ax, bool inhibit) {
  stepgen_t *s = axis(ax);
  uint32_t p;

  if (s == NULL) {
    return;
  }
  p = lock_irq();
  s->inhibited = inhibit;
  if (inhibit) {
    step_core_cmd_halt(&s->core);
    timer_stop(s);
  }
  unlock_irq(p);
}

bool stepgen_char_is_inhibited(uint8_t ax) {
  const stepgen_t *s = axis(ax);
  return (s == NULL) || *(const volatile bool *)&s->inhibited;
}

void stepgen_char_reset_stats(uint8_t ax) {
  stepgen_t *s = axis(ax);
  uint32_t p;

  if (s == NULL) {
    return;
  }
  p = lock_irq();
  s->cstats.edges_emitted = 0U;
  s->cstats.edges_inhibited = 0U;
  s->cstats.first_edge_us = 0U;
  s->cstats.last_edge_us = 0U;
  s->cstats.min_dt_us = 0U;
  s->cstats.max_dt_us = 0U;
  s->cstats.max_lateness_us = 0U;
  s->cstats.late_edges = 0U;
  s->cstats.ramp_up_end_us = 0U;
  s->cstats.ramp_up_end_edges = 0U;
  s->cstats.cruise_end_us = 0U;
  s->cstats.cruise_end_edges = 0U;
  s->cstats.lateness_fault = false;
  s->cstats.timing_reject = false;
  s->was_cruise = false;
  unlock_irq(p);
}

void stepgen_char_get_stats(uint8_t ax, stepgen_char_stats_t *out) {
  const stepgen_t *s = axis(ax);
  uint32_t p;

  if ((s == NULL) || (out == NULL)) {
    return;
  }
  p = lock_irq();
  *out = s->cstats;
  unlock_irq(p);
}

bool stepgen_char_validate_rate(uint8_t ax, float vmax, float vmin) {
  const stepgen_t *s = axis(ax);
  if ((s == NULL) || (vmax <= 0.0f) || (vmin <= 0.0f) || (vmin > vmax)) {
    return false;
  }
  /* Enforce >= 5 us minimum dwell between toggles (200 kHz max edge rate)
   * and reject quantization ceilings where timer ticks would clamp. */
  const float min_dwell_ticks = s->timer_hz * 5e-6f;
  const float ticks_vmax = s->timer_hz / vmax;
  const float ticks_vmin = s->timer_hz / vmin;
  if ((ticks_vmax < min_dwell_ticks) || (ticks_vmax < (float)STEPGEN_MIN_TICKS) ||
      (ticks_vmin > (float)STEPGEN_MAX_TICKS)) {
    return false;
  }
  return true;
}
#endif
