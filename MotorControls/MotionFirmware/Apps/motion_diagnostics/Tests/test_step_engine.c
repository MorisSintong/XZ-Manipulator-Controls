#include "sim_diagnostics.h"
int main(void)
{
    sim_diagnostics_t s;sim_init(&s);
    assert(step_engine_move(&s.steps,AXIS_X,800,s.now));
    for (unsigned i=0U;i<1500U;++i) { sim_tick(&s,1000U,false); }
    step_snapshot_t v=step_engine_snapshot(&s.steps,AXIS_X);
    assert(!v.running && v.position==800 && v.emitted==800 && v.edges==800U);
    assert(s.rising[0]==800U && s.falling[0]==800U && !v.timing_fault);
    assert(step_engine_move(&s.steps,AXIS_X,-16,s.now));
    for (unsigned i=0U;i<1500U;++i) { sim_tick(&s,1000U,false); }
    v=step_engine_snapshot(&s.steps,AXIS_X);
    assert(v.position==-16 && v.emitted==-816 && v.edges==816U && !v.timing_fault);
    assert(step_engine_move(&s.steps,AXIS_Z,10,s.now));
    s.now=s.deadline[1];step_engine_irq(&s.steps,AXIS_Z,s.now);
    assert(s.high[1]);step_engine_abort(&s.steps);
    s.now=s.deadline[1];step_engine_irq(&s.steps,AXIS_Z,s.now);
    v=step_engine_snapshot(&s.steps,AXIS_Z);
    assert(!s.high[1] && !v.running && v.emitted==1 && v.edges==1U);
    assert(!step_engine_move(&s.steps,AXIS_Z,100,s.now));
    s.now+=100U;assert(step_engine_clear(&s.steps));
    assert(step_engine_move(&s.steps,AXIS_Z,100,s.now));
    s.now=s.deadline[1]+21U;step_engine_irq(&s.steps,AXIS_Z,s.now);
    v=step_engine_snapshot(&s.steps,AXIS_Z);
    assert(v.timing_fault && !v.running && v.edges==0U);
    sim_init(&s);assert(step_engine_move(&s.steps,AXIS_X,1000,s.now));
    while (s.timer_on[0]) { s.now=s.deadline[0];step_engine_irq(&s.steps,AXIS_X,s.now); }
    v=step_engine_snapshot(&s.steps,AXIS_X);
    assert(v.timing_fault && v.edges==STEP_LOOKAHEAD && s.rising[0]==s.falling[0]);
    sim_init(&s);s.now=UINT32_MAX-1000U;
    assert(step_engine_move(&s.steps,AXIS_X,1,s.now));
    while (s.timer_on[0]) { s.now=s.deadline[0];step_engine_irq(&s.steps,AXIS_X,s.now); }
    assert(step_engine_snapshot(&s.steps,AXIS_X).position==1);
    sim_init(&s);assert(step_engine_move(&s.steps,AXIS_X,4,s.now));
    s.now=s.deadline[0];step_engine_irq(&s.steps,AXIS_X,s.now);
    s.now=s.deadline[0]+600U;step_engine_irq(&s.steps,AXIS_X,s.now);
    assert(s.rising[0]==1U && s.falling[0]==1U && !s.timer_on[0]);
    return 0;
}
