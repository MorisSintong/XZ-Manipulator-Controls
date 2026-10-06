#include "sim_diagnostics.h"
#include <stdlib.h>
typedef struct {
    sim_diagnostics_t sim;
    uint32_t counter, publications, abort_rx_calls, abort_tx_calls, start_calls;
    bool start_ok, stop_ok, dma_live, final_byte_on_abort;
} recovery_sim_t;
static bool abort_rx(void *p)
{
    recovery_sim_t *s=p;++s->abort_rx_calls;
    if (!s->stop_ok) { return false; }
    if (s->final_byte_on_abort) {
        s->sim.uart.rx[17]=10U;s->counter=18U;
    }
    s->dma_live=false;return true;
}
static void publish(void *p)
{
    recovery_sim_t *s=p;assert(s->sim.uart.rx_running);++s->publications;
    uart_transport_rx_publish(&s->sim.uart,s->counter);
}
static void drain_rx(void *p)
{
    recovery_sim_t *s=p;uart_transport_poll(&s->sim.uart,s->sim.now);
}
static bool start_rx(void *p)
{
    recovery_sim_t *s=p;++s->start_calls;
    if (s->start_ok) { s->dma_live=true;s->counter=s->sim.uart.producer; }
    return s->start_ok;
}
static bool abort_tx(void *p)
{
    ++((recovery_sim_t *)p)->abort_tx_calls;return true;
}
static uart_recovery_io_t io(recovery_sim_t *s)
{
    const uart_recovery_io_t callbacks={s,abort_rx,publish,drain_rx,start_rx,abort_tx};
    return callbacks;
}
static void init(recovery_sim_t *s)
{
    memset(s,0,sizeof(*s));sim_init(&s->sim);s->start_ok=true;s->stop_ok=true;
}
static void rx_tx_isolation(void)
{
    recovery_sim_t s;init(&s);
    s.sim.uart.rx_running=true;s.dma_live=true;
    uart_transport_error(&s.sim.uart,false,true);
    uart_transport_recover(&s.sim.uart,io(&s));
    assert(s.abort_tx_calls==1U && s.abort_rx_calls==0U && s.start_calls==0U);
    assert(s.sim.uart.rx_running && s.sim.uart.rx_overflows==0U);
    /* Last byte arrives during abort: snapshot after the stream is frozen. */
    const vision_cmd_t abort={VISION_TYPE_ABORT,90U,0U,0,0,0U,0};
    assert(vision_frame_encode(&abort,s.sim.uart.rx)==18U);
    s.sim.uart.rx[17]=0U;s.counter=17U;s.final_byte_on_abort=true;
    uart_transport_error(&s.sim.uart,true,false);
    uart_transport_recover(&s.sim.uart,io(&s));
    assert(s.sim.abort_received==1U && s.sim.seen_ids[90]==1U);
    assert(s.abort_rx_calls==1U && s.abort_tx_calls==1U);
    assert(s.sim.uart.rx_running && s.sim.uart.rx_overflows==1U);
    s.counter=s.sim.uart.producer;uart_transport_recover(&s.sim.uart,io(&s));
    assert(s.sim.abort_received==1U);
    /* A failed abort neither reads an unfrozen stream nor starts another one. */
    s.stop_ok=false;uart_transport_error(&s.sim.uart,true,false);
    const uint32_t publications=s.publications,starts=s.start_calls;
    uart_transport_recover(&s.sim.uart,io(&s));
    assert(s.publications==publications && s.start_calls==starts &&
           s.sim.uart.rx_error_pending);
}
static void failed_start(void)
{
    recovery_sim_t s;init(&s);s.start_ok=false;
    const vision_cmd_t stale={VISION_TYPE_PICK,12U,1U,0,0,0U,0};
    assert(vision_frame_encode(&stale,s.sim.uart.rx)==18U);s.counter=18U;
    for (unsigned i=0U;i<3U;++i) {
        uart_transport_recover(&s.sim.uart,io(&s));
        uart_transport_poll(&s.sim.uart,s.sim.now);
    }
    assert(!s.sim.uart.rx_running && s.publications==0U);
    assert(s.sim.received==0U && s.sim.uart.producer==0U && s.sim.uart.rx_overflows==0U);
    assert(s.start_calls==3U);
    s.start_ok=true;uart_transport_recover(&s.sim.uart,io(&s));
    assert(s.sim.uart.rx_running && s.publications==0U);
    const vision_cmd_t heartbeat={VISION_TYPE_STATUS,55U,0U,0,0,0U,0};
    assert(vision_frame_encode(&heartbeat,s.sim.uart.rx)==18U);s.counter=18U;
    uart_transport_recover(&s.sim.uart,io(&s));uart_transport_poll(&s.sim.uart,s.sim.now);
    assert(s.sim.seen_ids[55]==1U && s.sim.seen_ids[12]==0U);
    /* Failed recovery start must likewise never publish old stream progress. */
    s.start_ok=false;uart_transport_error(&s.sim.uart,true,false);
    uart_transport_recover(&s.sim.uart,io(&s));assert(!s.sim.uart.rx_running);
    const uint32_t count=s.publications,producer=s.sim.uart.producer;
    s.counter=producer+18U;
    uart_transport_recover(&s.sim.uart,io(&s));
    assert(s.publications==count && s.sim.uart.producer==producer);
}
int main(int argc,char **argv)
{
    assert(argc==2);
    if (atoi(argv[1])==1) { rx_tx_isolation(); }
    else { failed_start(); }
    return 0;
}
