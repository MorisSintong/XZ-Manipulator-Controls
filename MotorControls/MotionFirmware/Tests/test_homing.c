#include "sim_diagnostics.h"
int main(void)
{
    sim_diagnostics_t s;sim_init(&s);
    motion_executor_boot(&s.executor,s.now);
    assert(s.executor.state==EXEC_HOMING && s.executor.home.axis==AXIS_Z);
    for (unsigned i=0U;i<10000U && s.executor.home_pending;++i) {
        const homing_t *h=&s.executor.home;
        if (h->state==HOME_SEEK || h->state==HOME_LATCH) {
            s.sg[h->axis]=s.now-h->state_us>500000U?0U:100U;
        } else { s.sg[0]=100U;s.sg[1]=100U; }
        if (s.executor.home.result[AXIS_Z].result!=1U) { assert(s.rising[AXIS_X]==0U); }
        sim_tick(&s,1000U,true);
    }
    assert(s.executor.state==EXEC_READY && s.executor.homed_mask==3U);
    assert(s.executor.home_epoch==1U && homing_success(&s.executor.home));
    assert(s.executor.home.result[0].seek_emitted_steps<0);
    assert(s.executor.home.result[1].seek_emitted_steps<0);
    assert(s.executor.home.result[0].latch_emitted_steps<0);
    assert(s.executor.home.result[1].latch_emitted_steps<0);
    assert(step_engine_snapshot(&s.steps,0U).position==0);
    assert(step_engine_snapshot(&s.steps,1U).position==0);
    assert(s.uart.reserved==0U && s.uart.tx[(s.uart.head-1U)%8U].bytes[3]==DIAG_HOME_RESULT);
    sim_init(&s);s.config.axis[AXIS_Z].home_budget_steps=8;
    motion_executor_boot(&s.executor,s.now);
    for (unsigned i=0U;i<1000U && s.executor.home_pending;++i) { sim_tick(&s,1000U,true); }
    assert(s.executor.state==EXEC_FAULT && s.executor.home.result[AXIS_Z].result==3U);
    assert(s.executor.home_epoch==0U && s.rising[AXIS_X]==0U);
    sim_init(&s);s.config.axis[AXIS_Z].home_timeout_us=10000U;
    motion_executor_boot(&s.executor,s.now);
    for (unsigned i=0U;i<30U;++i) { sim_tick(&s,1000U,true); }
    assert(s.executor.home.result[AXIS_Z].result==2U && s.executor.homed_mask==0U);
    sim_init(&s);motion_executor_boot(&s.executor,s.now);
    for (unsigned i=0U;i<500U;++i) { sim_tick(&s,1000U,true); }
    const vision_cmd_t abort={VISION_TYPE_ABORT,88U,0U,0,0,0U,0};
    command_dispatch(&s.executor,&abort,s.now);
    for (unsigned i=0U;i<20U;++i) { sim_tick(&s,1000U,true); }
    assert(s.executor.home.result[AXIS_Z].result==4U && s.executor.state==EXEC_ABORTED);
    assert(s.executor.home.result[AXIS_Z].seek_emitted_steps<0);
    sim_init(&s);motion_executor_boot(&s.executor,s.now);
    sim_tick(&s,1000U,true);s.i2c_ok=false;
    for (unsigned i=0U;i<20U;++i) { sim_tick(&s,1000U,true); }
    assert(s.executor.home.result[AXIS_Z].result==5U && s.executor.state==EXEC_FAULT);
    sim_init(&s);s.stop_input=true;motion_executor_boot(&s.executor,s.now);
    sim_tick(&s,1000U,true);
    assert(s.executor.home.result[AXIS_Z].result==4U &&
           s.rising[0]==0U && s.rising[1]==0U && s.executor.state==EXEC_ABORTED);
    return 0;
}
