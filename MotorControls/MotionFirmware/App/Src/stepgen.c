/**
 * @file    stepgen.c
 * @brief   Timer-interrupt STEP/DIR pulse generation (see stepgen.h).
 */
#include "stepgen.h"

#include "motion_profile.h"

#define STEPGEN_MIN_TICKS 420U        /* 5 us at 84 MHz: floor for bad parameters */
#define STEPGEN_MAX_TICKS 4000000000U

typedef struct {
    stepgen_hw_t hw;
    step_core_t  core;
    float        timer_hz;
    uint32_t     step_level; /* STEP pin level; with dedge every edge is a step */
    bool         ready;
} stepgen_t;

static stepgen_t g_sg[STEPGEN_MAX_AXES];

static inline uint32_t lock_irq(void)
{
    const uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

static inline void unlock_irq(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static stepgen_t *axis(uint8_t ax)
{
    return ((ax < STEPGEN_MAX_AXES) && g_sg[ax].ready) ? &g_sg[ax] : NULL;
}

static uint32_t timer_input_clock(const TIM_TypeDef *tim)
{
    RCC_ClkInitTypeDef clk;
    uint32_t latency;
    bool apb2 = (tim == TIM1) || (tim == TIM8) || (tim == TIM9) ||
                (tim == TIM10) || (tim == TIM11);
    uint32_t pclk;
    uint32_t div;

    HAL_RCC_GetClockConfig(&clk, &latency);
    pclk = apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
    div = apb2 ? clk.APB2CLKDivider : clk.APB1CLKDivider;
    /* APB timers run at 2x PCLK whenever the APB prescaler is not 1. */
    return (div == RCC_HCLK_DIV1) ? pclk : 2U * pclk;
}

static inline void dir_out(const stepgen_t *s, int8_t dir)
{
    s->hw.dir_port->BSRR = (dir > 0) ? (uint32_t)s->hw.dir_pin
                                     : ((uint32_t)s->hw.dir_pin << 16U);
}

static inline void step_toggle(stepgen_t *s)
{
    s->step_level ^= 1U;
    s->hw.step_port->BSRR = (s->step_level != 0U) ? (uint32_t)s->hw.step_pin
                                                  : ((uint32_t)s->hw.step_pin << 16U);
}

static inline uint32_t ticks_for(const stepgen_t *s, float v)
{
    float t = s->timer_hz / v;

    if (t < (float)STEPGEN_MIN_TICKS) {
        t = (float)STEPGEN_MIN_TICKS;
    }
    if (t > (float)STEPGEN_MAX_TICKS) {
        t = (float)STEPGEN_MAX_TICKS;
    }
    return (uint32_t)t;
}

static void timer_stop(stepgen_t *s)
{
    TIM_TypeDef *t = s->hw.tim;

    t->CR1 &= ~TIM_CR1_CEN;
    t->DIER &= ~TIM_DIER_UIE;
    t->SR = ~TIM_SR_UIF;
    NVIC_ClearPendingIRQ(s->hw.irq);
}

/* Called with interrupts locked. The first edge comes one interval later, far
 * more than the 20 ns DIR setup time of the TMC2240. */
static void timer_start(stepgen_t *s, float v)
{
    TIM_TypeDef *t = s->hw.tim;

    dir_out(s, s->core.step_dir);
    t->CR1 &= ~TIM_CR1_CEN;
    t->CNT = 0U;
    t->ARR = ticks_for(s, v) - 1U;
    t->SR = ~TIM_SR_UIF;
    NVIC_ClearPendingIRQ(s->hw.irq);
    t->DIER |= TIM_DIER_UIE;
    t->CR1 |= TIM_CR1_CEN;
}

void stepgen_irq_handler(uint8_t ax)
{
    stepgen_t *s = &g_sg[ax];
    TIM_TypeDef *t = s->hw.tim;
    bool more = false;
    bool dir_changed = false;
    float v = 0.0f;

    if ((t->SR & TIM_SR_UIF) == 0U) {
        return;
    }
    t->SR = ~TIM_SR_UIF;
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
}

void stepgen_init(uint8_t ax, const stepgen_hw_t *hw)
{
    stepgen_t *s;
    TIM_TypeDef *t;

    if ((ax >= STEPGEN_MAX_AXES) || (hw == NULL)) {
        return;
    }
    s = &g_sg[ax];
    s->ready = false;
    s->hw = *hw;
    t = hw->tim;
    step_core_init(&s->core, 1000.0f, 1000.0f, 100.0f);
    s->timer_hz = (float)timer_input_clock(t) / (float)(t->PSC + 1U);
    s->step_level = ((hw->step_port->ODR & hw->step_pin) != 0U) ? 1U : 0U;
    t->CR1 &= ~(TIM_CR1_CEN | TIM_CR1_ARPE | TIM_CR1_OPM | TIM_CR1_URS | TIM_CR1_UDIS);
    t->DIER = 0U;
    t->SR = 0U;
    NVIC_ClearPendingIRQ(hw->irq);
    s->ready = true;
}

void stepgen_set_vmin(uint8_t ax, float vmin)
{
    stepgen_t *s = axis(ax);
    uint32_t p;

    if (s == NULL) {
        return;
    }
    p = lock_irq();
    s->core.vmin = (vmin >= 1.0f) ? vmin : 1.0f;
    unlock_irq(p);
}

void stepgen_move_to(uint8_t ax, int32_t target, float vmax, float accel)
{
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

void stepgen_run(uint8_t ax, int8_t dir, float vmax, float accel)
{
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

void stepgen_stop(uint8_t ax)
{
    stepgen_t *s = axis(ax);
    uint32_t p;

    if (s == NULL) {
        return;
    }
    p = lock_irq();
    step_core_cmd_stop(&s->core);
    unlock_irq(p);
}

void stepgen_halt(uint8_t ax)
{
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

bool stepgen_busy(uint8_t ax)
{
    const stepgen_t *s = axis(ax);

    return (s != NULL) && *(const volatile bool *)&s->core.running;
}

int32_t stepgen_position(uint8_t ax)
{
    const stepgen_t *s = axis(ax);

    return (s != NULL) ? *(const volatile int32_t *)&s->core.pos : 0;
}

bool stepgen_set_position(uint8_t ax, int32_t pos)
{
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

float stepgen_velocity(uint8_t ax)
{
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

bool stepgen_at_cruise(uint8_t ax)
{
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

void stepgen_set_limits(uint8_t ax, int32_t lo, int32_t hi)
{
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

bool stepgen_take_limit_hit(uint8_t ax)
{
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

uint32_t stepgen_timer_hz(uint8_t ax)
{
    const stepgen_t *s = axis(ax);

    return (s != NULL) ? (uint32_t)s->timer_hz : 0U;
}
