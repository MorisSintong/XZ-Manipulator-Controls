#include "sim_diagnostics.h"
static void ready(sim_diagnostics_t *s)
{
    sim_init(s);s->executor.state=EXEC_READY;s->executor.homed_mask=3U;
}
static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0]|((uint32_t)p[1]<<8U)|((uint32_t)p[2]<<16U)|((uint32_t)p[3]<<24U);
}
int main(void)
{
    sim_diagnostics_t s;ready(&s);
    const vision_cmd_t pick={VISION_TYPE_PICK,12U,1U,10,123,3570U,-30};
    s.config.pick_depth_01mm=2;
    command_dispatch(&s.executor,&pick,s.now);
    assert(s.executor.qhead==1U && s.uart.reserved==1U);
    for (unsigned i=0U;i<1000U;++i) { sim_tick(&s,1000U,true); }
    assert(s.executor.state==EXEC_READY && !s.executor.active);
    assert(s.executor.result.phase_count==3U && s.executor.result.command.y_01mm==123);
    assert(s.executor.result.data.phases[0].emitted_delta_steps==80);
    assert(s.executor.result.data.phases[1].emitted_delta_steps==80);
    assert(s.executor.result.data.phases[2].emitted_delta_steps==-80);
    assert(s.executor.result.data.phases[1].emitted_edge_count==80U);
    assert(s.executor.result.data.phases[2].emitted_edge_count==80U);
    assert((s.executor.result.status_bits&DIAG_SUCCESS)!=0U && s.uart.reserved==0U);
    for (unsigned i=0U;i<3U;++i) {
        assert((s.executor.result.data.phases[i].axis_flags&0xFFU)==0xFFU);
        assert(s.executor.result.data.phases[i].mscnt_check==3U);
    }
    const vision_cmd_t heartbeat={VISION_TYPE_STATUS,77U,0U,0,0,0U,0};
    const uint32_t before=s.rising[0]+s.rising[1],epoch=s.executor.home_epoch;
    command_dispatch(&s.executor,&heartbeat,s.now);
    assert(s.rising[0]+s.rising[1]==before && s.executor.home_epoch==epoch);
    assert(s.uart.tx[(s.uart.head-1U)%8U].bytes[3]==DIAG_STATUS);
    assert(s.uart.tx[(s.uart.head-1U)%8U].bytes[26]==77U);
    ready(&s);vision_cmd_t invalid=pick;invalid.x_01mm=-1;
    command_dispatch(&s.executor,&invalid,s.now);
    assert(s.executor.qhead==0U && s.executor.rejected==1U && s.uart.reserved==0U);
    assert((le32(s.uart.tx[0].bytes+22)&(DIAG_REJECTED|DIAG_LIMIT_FAULT))==
           (DIAG_REJECTED|DIAG_LIMIT_FAULT));
    ready(&s);s.config.bounds_confirmed=false;
    command_dispatch(&s.executor,&pick,s.now);assert(s.executor.rejected==1U);
    ready(&s);
    for (unsigned i=0U;i<4U;++i) { command_dispatch(&s.executor,&pick,s.now); }
    assert(s.executor.qhead-s.executor.qtail==4U && s.uart.reserved==4U);
    command_dispatch(&s.executor,&pick,s.now);assert(s.executor.rejected==1U);
    sim_tick(&s,1000U,true);command_dispatch(&s.executor,&pick,s.now);
    assert(s.executor.active && s.executor.qhead-s.executor.qtail==4U && s.uart.reserved==5U);
    for (unsigned i=0U;i<20U;++i) { sim_tick(&s,1000U,true); }
    const vision_cmd_t abort={VISION_TYPE_ABORT,88U,0U,0,0,0U,0};
    command_dispatch(&s.executor,&abort,s.now);
    for (unsigned i=0U;i<20U;++i) { sim_tick(&s,1000U,true); }
    assert(s.executor.state==EXEC_ABORTED && !s.executor.active && s.executor.homed_mask==0U);
    assert(s.executor.qhead==s.executor.qtail && s.uart.reserved==0U);
    assert((s.executor.result.status_bits&DIAG_ABORTED)!=0U);
    assert(s.executor.result.data.phases[0].emitted_edge_count<80U);
    unsigned cancelled=0U,aborted=0U;
    for (uint32_t i=s.uart.tail;i<s.uart.head;++i) {
        const uint8_t *p=s.uart.tx[i%8U].bytes;
        if (p[3]==DIAG_COMMAND_RESULT) {
            if ((le32(p+22)&DIAG_CANCELLED)!=0U) { ++cancelled;assert(p[38]==0U); }
            if ((le32(p+22)&DIAG_ABORTED)!=0U && p[38]!=0U) { ++aborted; }
        }
    }
    assert(cancelled==4U && aborted==1U);
    /* Explicit re-home can clear abort only with healthy sensors and no work. */
    const vision_cmd_t home={VISION_TYPE_HOME,99U,0U,0,0,0U,0};
    while (s.uart.tail!=s.uart.head) { s.uart.active=true;uart_transport_tx_complete(&s.uart); }
    s.i2c_ok=false;command_dispatch(&s.executor,&home,s.now);
    assert(s.executor.state==EXEC_ABORTED);
    s.i2c_ok=true;command_dispatch(&s.executor,&home,s.now);
    assert(s.executor.state==EXEC_HOMING && s.executor.home_pending);
    ready(&s);s.config.clamp_x=true;invalid.x_01mm=-3;
    command_dispatch(&s.executor,&invalid,s.now);
    for (unsigned i=0U;i<500U;++i) { sim_tick(&s,1000U,true); }
    assert((s.executor.result.status_bits&DIAG_CLAMPED)!=0U);
    assert(s.executor.result.data.phases[0].requested_target_01mm==-3);
    assert(s.executor.result.data.phases[0].applied_target_01mm==0);
    assert(s.executor.result.data.phases[0].quant_residual_nm==0);
    ready(&s);s.config.mscnt_qualified=true;s.mscnt_error=true;
    command_dispatch(&s.executor,&pick,s.now);
    for (unsigned i=0U;i<500U;++i) { sim_tick(&s,1000U,true); }
    assert(s.executor.state==EXEC_FAULT && s.executor.homed_mask==0U);
    assert((s.executor.faults&DIAG_MSCNT_MISMATCH)!=0U &&
           (s.executor.faults&DIAG_ABORTED)==0U);
    ready(&s);command_dispatch(&s.executor,&pick,s.now);
    sim_tick(&s,1000U,true);s.driver_ok=false;
    assert(!driver_evidence_poll(&s.drivers,0U));
    for (unsigned i=0U;i<30U;++i) { sim_tick(&s,1000U,true); }
    assert(s.executor.state==EXEC_FAULT && (s.executor.faults&DIAG_DRIVER_FAULT)!=0U);
    assert((s.executor.faults&DIAG_ABORTED)==0U);
    ready(&s);command_dispatch(&s.executor,&pick,s.now);
    while (s.executor.phase_state!=PHASE_DWELL) { sim_tick(&s,1000U,true); }
    const uint32_t frozen=s.executor.result.data.phases[1].raw_end_timestamp_us;
    command_dispatch(&s.executor,&abort,s.now);
    sim_tick(&s,1000U,true);
    assert(s.executor.state==EXEC_ABORTED);
    assert(s.executor.result.data.phases[1].raw_end_timestamp_us==frozen);
    assert((s.executor.result.data.phases[1].axis_flags&PH_COMPLETE)!=0U);
    ready(&s);s.stop_input=true;command_dispatch(&s.executor,&home,s.now);
    assert(s.executor.rejected==1U && !s.executor.home_pending);
    return 0;
}
