#ifndef SIM_DIAGNOSTICS_H
#define SIM_DIAGNOSTICS_H
#include "command_dispatch.h"
#include <assert.h>
#include <string.h>
typedef struct {
    uint32_t now, deadline[2], last_rise[2], last_fall[2], rising[2], falling[2];
    uint16_t raw[2], sg[2], conf[2];
    uint16_t driver_phase[2];
    int8_t dir_level[2];
    uint32_t irq_depth, seen_ids[128], abort_received;
    bool assert_tx_unmasked, tx_complete_on_start, tx_error_before_start;
    bool timer_on[2], high[2], i2c_ok, driver_ok, tx_ok, mscnt_error, stop_input;
    uint8_t magnet[2];
    motion_config_t config;
    step_engine_t steps;
    encoder_sampler_t encoders;
    driver_evidence_t drivers;
    uart_transport_t uart;
    motion_executor_t executor;
    uint32_t received;
} sim_diagnostics_t;
static uint32_t sim_lock(void *p)
{
    sim_diagnostics_t *s=p;const uint32_t old=s->irq_depth;++s->irq_depth;return old;
}
static void sim_unlock(void *p,uint32_t irq) { ((sim_diagnostics_t *)p)->irq_depth=irq; }
static uint32_t sim_us(void *p) { return ((sim_diagnostics_t *)p)->now; }
static uint32_t sim_ms(void *p) { return sim_us(p)/1000U; }
static bool sim_safe(void *p) { return !((sim_diagnostics_t *)p)->stop_input; }
static void sim_step(void *p,uint8_t axis,bool high)
{
    sim_diagnostics_t *s=p; s->high[axis]=high;
    if (high) {
        ++s->rising[axis]; s->last_rise[axis]=s->now;
        s->driver_phase[axis]=(uint16_t)((s->driver_phase[axis]+
            (s->dir_level[axis]>0?16U:1008U))&1023U);
    }
    else { ++s->falling[axis];s->last_fall[axis]=s->now;
        assert(s->now-s->last_rise[axis]>=5U); }
}
static void sim_dir(void *p,uint8_t axis,int8_t dir) { ((sim_diagnostics_t *)p)->dir_level[axis]=dir; }
static void sim_schedule(void *p,uint8_t axis,uint32_t deadline,bool on)
{
    sim_diagnostics_t *s=p;s->deadline[axis]=deadline;s->timer_on[axis]=on;
}
static bool sim_raw(void *p,uint8_t axis,uint16_t *raw)
{
    sim_diagnostics_t *s=p;*raw=s->raw[axis];return s->i2c_ok;
}
static bool sim_health(void *p,uint8_t axis,uint8_t *status,uint8_t *agc)
{
    sim_diagnostics_t *s=p;*status=s->magnet[axis];*agc=64U;return s->i2c_ok;
}
static bool sim_conf(void *p,uint8_t axis,uint16_t *conf)
{
    sim_diagnostics_t *s=p;*conf=s->conf[axis];return s->i2c_ok;
}
static bool sim_driver(void *p,uint8_t axis,driver_sample_t *sample)
{
    sim_diagnostics_t *s=p;memset(sample,0,sizeof(*sample));sample->sg=s->sg[axis];
    const int32_t pos=step_engine_snapshot(&s->steps,axis).position;
    sample->mscnt=s->driver_phase[axis];sample->mode=4U;
    if (s->mscnt_error && pos != 0) { sample->mscnt ^= 1U; }
    return s->driver_ok;
}
static bool sim_tx(void *p,const uint8_t *bytes,uint16_t size)
{
    sim_diagnostics_t *s=p;assert(bytes[0]==0xD3U && size>=116U);
    if (s->assert_tx_unmasked) { assert(s->irq_depth==0U); }
    if (s->tx_error_before_start) { uart_transport_tx_error(&s->uart); }
    if (s->tx_ok && s->tx_complete_on_start) { uart_transport_tx_complete(&s->uart); }
    return s->tx_ok;
}
static void sim_command(void *p,const vision_cmd_t *cmd)
{
    sim_diagnostics_t *s=p;++s->received;
    if (cmd->obj_id<128U) { ++s->seen_ids[cmd->obj_id]; }
    if (cmd->type==VISION_TYPE_ABORT) { ++s->abort_received; }
    command_dispatch(&s->executor,cmd,s->now);
}
static inline void sim_init(sim_diagnostics_t *s)
{
    memset(s,0,sizeof(*s));s->config=motion_default_config;
    s->config.bounds_confirmed=true;s->i2c_ok=true;s->driver_ok=true;s->tx_ok=true;
    s->magnet[0]=32U;s->magnet[1]=32U;s->sg[0]=100U;s->sg[1]=100U;
    s->conf[0]=0x0300U;s->conf[1]=0x0300U;
    s->now=1000U;
    const step_engine_io_t si={s,sim_lock,sim_unlock,sim_step,sim_dir,sim_schedule,sim_us};
    const encoder_io_t ei={s,sim_raw,sim_health,sim_us,sim_ms,sim_conf};
    const driver_io_t di={s,sim_driver};
    const uart_transport_io_t ui={s,sim_lock,sim_unlock,sim_tx,sim_command};
    step_engine_init(&s->steps,&s->config,si);
    encoder_sampler_init(&s->encoders,&s->config,ei);
    driver_evidence_init(&s->drivers,di);uart_transport_init(&s->uart,ui);
    motion_executor_init(&s->executor,&s->config,&s->steps,&s->encoders,&s->drivers,&s->uart);
    s->executor.safe_inputs=sim_safe;s->executor.safe_context=s;
    encoder_sampler_poll(&s->encoders,true);
    assert(driver_evidence_poll(&s->drivers,0U));assert(driver_evidence_poll(&s->drivers,1U));
}
static inline void sim_tick(sim_diagnostics_t *s,uint32_t us,bool execute)
{
    const uint32_t end=s->now+us;
    while (true) {
        uint8_t axis=2U;uint32_t closest=end;
        for (uint8_t i=0U;i<2U;++i) {
            if (s->timer_on[i] && (int32_t)(s->deadline[i]-closest)<=0 &&
                (int32_t)(s->deadline[i]-s->now)>=0) {
                closest=s->deadline[i];axis=i;
            }
        }
        if (axis==2U) { break; }
        s->now=closest;step_engine_irq(&s->steps,axis,s->now);
    }
    s->now=end;
    for (uint8_t i=0U;i<2U;++i) {
        const int32_t pos=step_engine_snapshot(&s->steps,i).position;
        const int64_t counts=(int64_t)pos*4096/3200;
        s->raw[i]=(uint16_t)((uint64_t)counts&4095U);
    }
    encoder_sampler_poll(&s->encoders,false);
    if (execute) { motion_executor_poll(&s->executor,s->now); }
    else { step_engine_poll(&s->steps); }
}
#endif
