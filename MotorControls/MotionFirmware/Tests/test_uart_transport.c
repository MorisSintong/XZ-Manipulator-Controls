#include "sim_diagnostics.h"
#include "crc16_ccitt.h"
int main(void)
{
    sim_diagnostics_t s;sim_init(&s);
    const vision_cmd_t c={VISION_TYPE_STATUS,1U,1U,1500,1000,0U,0};
    uint8_t cmd[VISION_FRAME_SIZE];assert(vision_frame_encode(&c,cmd)==18U);
    memcpy(s.uart.rx,cmd,18U);uart_transport_rx_publish(&s.uart,18U);
    uart_transport_poll(&s.uart,s.now);assert(s.received==1U && s.uart.active);
    const uart_tx_slot_t first=s.uart.tx[0];
    diag_record_t r={.type=DIAG_STATUS,.config_id=1U};
    assert(uart_transport_record(&s.uart,&r,false,true));
    r.timestamp_ms=123U;assert(uart_transport_record(&s.uart,&r,false,true));
    assert(uart_transport_depth(&s.uart)==2U && memcmp(&first,&s.uart.tx[0],sizeof(first))==0);
    assert(uart_transport_reserve(&s.uart));assert(uart_transport_reserve(&s.uart));
    assert(uart_transport_reserve(&s.uart));assert(uart_transport_reserve(&s.uart));
    assert(uart_transport_reserve(&s.uart));assert(uart_transport_reserve(&s.uart));
    assert(!uart_transport_reserve(&s.uart));
    assert(!uart_transport_record(&s.uart,&r,false,false));
    for (unsigned i=0U;i<6U;++i) { assert(uart_transport_record(&s.uart,&r,true,false)); }
    assert(s.uart.reserved==0U && uart_transport_depth(&s.uart)==8U);
    uart_transport_tx_error(&s.uart);s.tx_ok=false;uart_transport_poll(&s.uart,s.now);
    s.now+=100001U;assert(uart_transport_backpressure(&s.uart,s.now));
    s.tx_ok=true;uart_transport_poll(&s.uart,s.now);
    assert(memcmp(&first,&s.uart.tx[0],sizeof(first))==0);
    uart_transport_tx_complete(&s.uart);assert(s.uart.tail==1U);
    s.uart.parser.used=4U;uart_transport_rx_publish(&s.uart,18U+512U);
    uart_transport_poll(&s.uart,s.now);
    assert(s.uart.rx_overflows==1U && s.uart.parser.used==0U && s.received==1U);
    /* A complete frame crossing the circular buffer boundary. */
    sim_init(&s);s.uart.consumer=510U;s.uart.producer=510U;
    for (uint32_t i=0U;i<18U;++i) { s.uart.rx[(510U+i)%512U]=cmd[i]; }
    uart_transport_rx_publish(&s.uart,528U);uart_transport_poll(&s.uart,s.now);
    assert(s.received==1U);
    return 0;
}
