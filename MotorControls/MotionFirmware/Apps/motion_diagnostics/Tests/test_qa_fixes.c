#include "sim_diagnostics.h"
#include <stdlib.h>
static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0]|((uint32_t)p[1]<<8U)|((uint32_t)p[2]<<16U)|((uint32_t)p[3]<<24U);
}
static void drain(sim_diagnostics_t *s)
{
    while (s->uart.tail!=s->uart.head) {
        s->uart.active=true;uart_transport_tx_complete(&s->uart);
    }
}
static void rx_restart(void)
{
    sim_diagnostics_t s;sim_init(&s);
    s.executor.state=EXEC_READY;s.executor.homed_mask=3U;
    uint32_t produced=0U;
    for (uint16_t id=1U;id<=30U;++id) {
        const vision_cmd_t pick={VISION_TYPE_PICK,id,1U,0,0,0U,0};
        uint8_t bytes[18];assert(vision_frame_encode(&pick,bytes)==18U);
        for (uint32_t i=0U;i<18U;++i) { s.uart.rx[(produced+i)%512U]=bytes[i]; }
        produced+=18U;uart_transport_rx_publish(&s.uart,produced);
        uart_transport_poll(&s.uart,s.now);
        for (unsigned tick=0U;tick<150U;++tick) { sim_tick(&s,1000U,true); }
        assert(!s.executor.active && s.executor.result.command.obj_id==id &&
               (s.executor.result.status_bits&DIAG_SUCCESS)!=0U);
        drain(&s);
    }
    assert(s.received==30U && s.uart.producer==540U);
    s.uart.parser.used=4U;
    const uint32_t base=uart_transport_rx_restart(&s.uart);
    const vision_cmd_t abort={VISION_TYPE_ABORT,90U,0U,0,0,0U,0};
    assert(vision_frame_encode(&abort,s.uart.rx)==18U);
    uart_transport_rx_publish(&s.uart,base+18U);uart_transport_poll(&s.uart,s.now);
    assert(s.abort_received==1U && s.received==31U);
    for (uint16_t id=1U;id<=30U;++id) { assert(s.seen_ids[id]==1U); }
    assert(s.uart.parser.used==0U && base%512U==0U);
    for (uint32_t i=0U;i<500U;++i) { s.uart.rx[(18U+i)%512U]=0xEFU; }
    uart_transport_rx_publish(&s.uart,base+518U);uart_transport_poll(&s.uart,s.now);
    assert(s.received==31U && s.abort_received==1U);
    for (uint16_t id=1U;id<=30U;++id) { assert(s.seen_ids[id]==1U); }
    /* Full-lap loss remains observable after changing the logical origin. */
    s.uart.parser.used=3U;uart_transport_rx_publish(&s.uart,base+518U+512U);
    uart_transport_poll(&s.uart,s.now);
    assert(s.uart.rx_overflows==2U && s.uart.parser.used==0U && s.received==31U);
    for (uint16_t id=1U;id<=30U;++id) { assert(s.seen_ids[id]==1U); }
    s.uart.producer=UINT32_MAX-11U;s.uart.consumer=s.uart.producer;
    const uint32_t wrapped=uart_transport_rx_restart(&s.uart);
    assert(wrapped==0U && s.uart.consumer==0U);
    /* Recovery drains complete unconsumed frames before dropping partial residue. */
    sim_init(&s);
    const vision_cmd_t pick={VISION_TYPE_PICK,1U,1U,0,0,0U,0};
    assert(vision_frame_encode(&pick,s.uart.rx)==18U);
    s.uart.rx[18]=0xAAU;s.uart.rx[19]=0x55U;s.uart.rx[20]=1U;
    uart_transport_rx_publish(&s.uart,21U);uart_transport_poll(&s.uart,s.now);
    assert(s.seen_ids[1]==1U && s.uart.parser.used==3U);
    (void)uart_transport_rx_restart(&s.uart);
    assert(s.uart.parser.used==0U && s.uart.parser.frames_ok==1U);
}
static void rx_event(void)
{
    sim_diagnostics_t s;sim_init(&s);s.executor.state=EXEC_READY;s.executor.homed_mask=3U;
    s.uart.rx_overflows=1U;
    motion_executor_status(&s.executor,NULL,0U,true,s.now);
    const uint8_t *first=s.uart.tx[(s.uart.head-1U)%8U].bytes;
    assert((le32(first+22)&DIAG_RX_OVERFLOW)!=0U && le32(first+52)==1U);
    motion_executor_status(&s.executor,NULL,0U,true,s.now);
    const uint8_t *second=s.uart.tx[(s.uart.head-1U)%8U].bytes;
    assert((le32(second+22)&DIAG_RX_OVERFLOW)==0U && le32(second+52)==1U);
    assert(s.uart.head-s.uart.tail==2U); /* pending event must not be coalesced away */
    s.uart.parser.crc_errors=1U;
    motion_executor_status(&s.executor,NULL,0U,false,s.now);
    assert((le32(s.uart.tx[(s.uart.head-1U)%8U].bytes+22)&DIAG_RX_OVERFLOW)!=0U);
    motion_executor_status(&s.executor,NULL,0U,false,s.now);
    assert((le32(s.uart.tx[(s.uart.head-1U)%8U].bytes+22)&DIAG_RX_OVERFLOW)==0U);
    drain(&s);
    diag_record_t status={.type=DIAG_STATUS,.config_id=1U};
    for (unsigned i=0U;i<8U;++i) { assert(uart_transport_record(&s.uart,&status,false,false)); }
    s.uart.parser.format_errors=1U;
    motion_executor_status(&s.executor,NULL,0U,false,s.now);
    assert(s.executor.status_rx_format_errors==0U);
    drain(&s);motion_executor_status(&s.executor,NULL,0U,false,s.now);
    assert((le32(s.uart.tx[(s.uart.head-1U)%8U].bytes+22)&DIAG_RX_OVERFLOW)!=0U);
    assert(s.executor.status_rx_format_errors==1U);
    drain(&s);
    const vision_cmd_t home={VISION_TYPE_HOME,12U,0U,0,0,0U,0};
    command_dispatch(&s.executor,&home,s.now);
    assert(s.executor.home_pending && s.executor.faults==0U);
}
static void phase_sign(void)
{
    sim_diagnostics_t s;sim_init(&s);s.executor.state=EXEC_READY;s.executor.homed_mask=3U;
    s.config.mscnt_qualified=true;s.config.pick_depth_01mm=2;
    const vision_cmd_t pick={VISION_TYPE_PICK,1U,1U,10,0,0U,0};
    command_dispatch(&s.executor,&pick,s.now);
    for (unsigned i=0U;i<1000U;++i) { sim_tick(&s,1000U,true); }
    assert(s.executor.state==EXEC_READY && (s.executor.result.status_bits&DIAG_SUCCESS)!=0U);
    const diag_phase_t *down=&s.executor.result.data.phases[1],*up=&s.executor.result.data.phases[2];
    assert(down->mscnt_expected==768U && down->mscnt_observed==768U && down->mscnt_check==1U);
    assert(up->mscnt_expected==256U && up->mscnt_observed==256U && up->mscnt_check==1U);
    assert((down->axis_flags&PH_MSCNT_NEG)!=0U && (up->axis_flags&PH_MSCNT_NEG)!=0U);
}
static void tx_unmasked(void)
{
    sim_diagnostics_t s;sim_init(&s);s.assert_tx_unmasked=true;s.tx_complete_on_start=true;
    diag_record_t r={.type=DIAG_STATUS,.config_id=1U};
    assert(uart_transport_record(&s.uart,&r,false,false));
    uart_transport_poll(&s.uart,s.now);
    assert(s.uart.tail==1U && !s.uart.active && !s.uart.tx_failed);
    s.tx_complete_on_start=false;s.tx_ok=false;
    assert(uart_transport_record(&s.uart,&r,false,false));
    uart_transport_poll(&s.uart,s.now);
    assert(s.uart.tail==1U && !s.uart.active && s.uart.tx_failed);
    s.tx_ok=true;uart_transport_poll(&s.uart,s.now);
    assert(s.uart.tail==1U && s.uart.active);
    uart_transport_tx_complete(&s.uart);assert(s.uart.tail==2U && !s.uart.active);
    s.tx_error_before_start=true;
    assert(uart_transport_record(&s.uart,&r,false,false));
    uart_transport_poll(&s.uart,s.now);
    assert(s.uart.active && !s.uart.tx_starting && s.uart.tail==2U);
    uart_transport_tx_complete(&s.uart);
    assert(s.uart.tail==3U && !s.uart.active);
}
static void conf_readback(void)
{
    sim_diagnostics_t s;sim_init(&s);
    s.conf[AXIS_X]=0xC300U;s.now+=2000U;
    encoder_sampler_poll(&s.encoders,true);
    assert(s.encoders.axis[AXIS_X].conf==0xC300U);
    assert(encoder_sampler_reseed(&s.encoders,AXIS_X));
    assert(s.encoders.axis[AXIS_X].conf==0xC300U && s.encoders.axis[AXIS_X].valid);
    s.executor.state=EXEC_READY;s.executor.homed_mask=3U;
    const vision_cmd_t pick={VISION_TYPE_PICK,1U,1U,100,0,0U,0};
    command_dispatch(&s.executor,&pick,s.now);sim_tick(&s,1000U,true);
    assert(s.executor.result.data.phases[0].as_conf==0xC300U);
    s.conf[AXIS_X]=0xC000U;s.now+=2000U;
    encoder_sampler_poll(&s.encoders,true);
    assert(s.encoders.axis[AXIS_X].conf==0xC000U && !s.encoders.axis[AXIS_X].valid);
    for (unsigned i=0U;i<20U;++i) { sim_tick(&s,1000U,true); }
    assert(s.executor.state==EXEC_FAULT &&
           (s.executor.faults&(DIAG_ENCODER_INVALID|DIAG_CONFIG_INVALID))==
           (DIAG_ENCODER_INVALID|DIAG_CONFIG_INVALID));
    assert(s.executor.result.data.phases[0].as_conf==0xC000U);
    assert(!encoder_sampler_reseed(&s.encoders,AXIS_X));
    s.conf[AXIS_X]=0xC300U;assert(encoder_sampler_reseed(&s.encoders,AXIS_X));
    assert(s.encoders.axis[AXIS_X].conf==0xC300U && s.encoders.axis[AXIS_X].valid);
}
int main(int argc,char **argv)
{
    assert(argc==2);
    switch (atoi(argv[1])) {
    case 1: rx_restart();break;
    case 3: rx_event();break;
    case 6: phase_sign();break;
    case 7: conf_readback();break;
    case 9: tx_unmasked();break;
    default: return 1;
    }
    return 0;
}
