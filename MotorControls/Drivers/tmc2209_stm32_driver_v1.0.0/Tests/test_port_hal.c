#include "fake_hal.h"
#include "test_support.h"
static test_counts_t counts;
static const uint8_t request[4] = {5u, 0u, 6u, 0x6Fu};
static const uint8_t write_frame[8] = {5u, 0u, 0x80u, 0u, 0u, 0u, 0xC0u, 0x40u};

static tmc2209_port_t start(void)
{
    hal_reset();
    tmc2209_port_t port = hal_port();
    CHECK(TMC2209_Port_Init(&port) == TMC2209_OK);
    return port;
}

static void test_init_parameters(void)
{
    hal_reset();
    tmc2209_port_t port = {0};
    CHECK(TMC2209_Port_Init(NULL) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Port_Init(&port) == TMC2209_ERR_PARAM);
    port = hal_port(); hal_fake.uart.Instance = NULL;
    CHECK(TMC2209_Port_Init(&port) == TMC2209_ERR_PARAM);
    for (unsigned error = 0u; error < 19u; ++error) {
        hal_reset(); port = hal_port();
        switch (error) {
        case 0: port.baud = 8999u; break;
        case 1: port.baud = 500001u; break;
        case 2: hal_fake.uart.Init.BaudRate = 9600u; break;
        case 3: hal_fake.uart.Init.WordLength = 1u; break;
        case 4: hal_fake.uart.Init.StopBits = 1u; break;
        case 5: hal_fake.uart.Init.Parity = 1u; break;
        case 6: hal_fake.uart.Init.HwFlowCtl = 1u; break;
        case 7: hal_fake.uart.Init.Mode = 0u; break;
        case 8: hal_fake.uart.Init.OverSampling = 1u; break;
        case 9: hal_fake.uart_regs.CR3 = 0u; break;
        case 10: hal_fake.uart_regs.CR1 = 0u; break;
        case 11: hal_fake.uart.gState = 0u; break;
        case 12: hal_fake.uart.RxState = 0u; break;
        case 13: port.tx_timeout_ms = 1001u; break;
        case 14: port.rx_timeout_ms = 1001u; break;
        case 15: port.tx_timeout_ms = 0u; break;
        case 16: port.rx_timeout_ms = 0u; break;
        case 17: port.rx_timeout_ms = 2u; break;
        default: port.bus_idle_us = 1000001u; break;
        }
        CHECK(TMC2209_Port_Init(&port) == TMC2209_ERR_PARAM);
        CHECK(!port.initialized && hal_fake.writes == 0u);
    }
    port = start();
    CHECK(TMC2209_Port_Init(&port) == TMC2209_ERR_STATE);
    CHECK(TMC2209_Port_Deinit(&port) == TMC2209_OK);
    CHECK(!port.initialized && port.huart == NULL);
    CHECK((hal_fake.uart_regs.CR1 & (USART_CR1_TE | USART_CR1_RE)) == 0u);
    CHECK(TMC2209_Port_Deinit(&port) == TMC2209_OK);
    CHECK(TMC2209_Port_Deinit(NULL) == TMC2209_OK);
    for (unsigned rate = 9000u; rate <= 500000u; rate += 491000u) {
        hal_reset(); port = hal_port(); port.baud = rate;
        port.tx_timeout_ms = 30u; port.rx_timeout_ms = 30u;
        hal_fake.uart.Init.BaudRate = rate;
        CHECK(TMC2209_Port_Init(&port) == TMC2209_OK);
        CHECK(TMC2209_Port_Transmit(&port, write_frame, 8u) == TMC2209_OK);
    }
}

static void test_init_failures_and_clock(void)
{
    for (unsigned result = HAL_ERROR; result <= HAL_TIMEOUT; ++result) {
        hal_reset();
        tmc2209_port_t port = hal_port();
        hal_fake.rx_mode_result = (HAL_StatusTypeDef)result;
        tmc2209_status_t expected = result == HAL_TIMEOUT ? TMC2209_ERR_TIMEOUT :
                                    (result == HAL_BUSY ? TMC2209_ERR_BUSY : TMC2209_ERR_PORT);
        CHECK(TMC2209_Port_Init(&port) == expected);
        CHECK(!port.initialized && hal_fake.writes == 0u);
    }
    hal_reset(); tmc2209_port_t port = hal_port();
    hal_fake.stuck_rx = true;
    CHECK(TMC2209_Port_Init(&port) == TMC2209_ERR_TIMEOUT);
    CHECK(hal_fake.reads == TMC2209_RX_DRAIN_LIMIT);
    hal_reset(); port = hal_port(); hal_fake.cycle_step = 0u;
    CHECK(TMC2209_Port_Init(&port) == TMC2209_ERR_TIMEOUT);
    CHECK(!port.initialized);
    hal_reset(); port = hal_port(); SystemCoreClock = 0u;
    CHECK(TMC2209_Port_Init(&port) == TMC2209_ERR_PORT);
    SystemCoreClock = 180000001u;
    CHECK(TMC2209_Port_DelayUs(1u) == TMC2209_ERR_PORT);
    SystemCoreClock = 180000000u; fake_dwt.CTRL = 0u;
    CHECK(TMC2209_Port_DelayUs(1u) == TMC2209_ERR_PORT);
    fake_dwt.CTRL = 1u; fake_debug.DEMCR = 0u;
    CHECK(TMC2209_Port_DelayUs(1u) == TMC2209_ERR_PORT);
    CHECK(TMC2209_Port_DelayUs(0u) == TMC2209_OK);
    CHECK(TMC2209_Port_DelayUs(1000001u) == TMC2209_ERR_PARAM);
    port = start();
    fake_dwt.CYCCNT = UINT32_MAX - 20000u;
    uint32_t before = fake_dwt.CYCCNT;
    CHECK(TMC2209_Port_DelayUs(1000u) == TMC2209_OK);
    CHECK((uint32_t)(fake_dwt.CYCCNT - before) >= 180000u);
    hal_fake.cycle_step = 1800000u;
    CHECK(TMC2209_Port_DelayUs(1000000u) == TMC2209_OK);
    hal_fake.cycle_step = 0u;
    CHECK(TMC2209_Port_DelayUs(1u) == TMC2209_ERR_TIMEOUT);
    hal_fake.cycle_step = 18000u;
    port = hal_port(); fake_dwt.CYCCNT = 1234567u;
    CHECK(TMC2209_Port_Init(&port) == TMC2209_OK);
    CHECK(fake_dwt.CYCCNT > 1234567u); /* Existing DWT is not reset. */
    SystemCoreClock = 168123456u;
    before = fake_dwt.CYCCNT;
    CHECK(TMC2209_Port_DelayUs(101u) == TMC2209_OK);
    CHECK((uint32_t)(fake_dwt.CYCCNT - before) >= 16981u);
}

static void test_turnaround_and_echo(void)
{
    tmc2209_port_t port = start();
    hal_fake.echo = true;
    hal_fake.fifo[0] = 0xAAu; hal_fake.fifo_size = 1u;
    hal_fake.uart_regs.SR = USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE;
    CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_OK);
    CHECK(port.awaiting_reply && hal_fake.reads == 5u);
    CHECK((hal_fake.uart_regs.CR1 & USART_CR1_RE) != 0u);
    CHECK((hal_fake.uart_regs.CR1 & USART_CR1_TE) == 0u);
    CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_ERR_STATE);
    uint8_t data[8], expected[8];
    reference_reply(6u, 0x21000000u, expected);
    unsigned clears = hal_fake.clears;
    CHECK(TMC2209_Port_Receive(&port, data, 7u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Port_Receive(&port, data, 8u) == TMC2209_OK);
    CHECK(memcmp(data, expected, 8u) == 0);
    CHECK(hal_fake.clears == clears); /* No receive-entry flush of live reply. */
    CHECK(!port.awaiting_reply);
    CHECK(TMC2209_Port_Receive(&port, data, 8u) == TMC2209_ERR_STATE);
    port.bus_idle_us = 2000u;
    uint32_t before = fake_dwt.CYCCNT;
    CHECK(TMC2209_Port_Transmit(&port, write_frame, 8u) == TMC2209_OK);
    CHECK((uint32_t)(fake_dwt.CYCCNT - before) >= 360000u);
    CHECK(hal_fake.registers[0][0] == 0xC0u && !port.awaiting_reply);
    CHECK(TMC2209_Port_Transmit(NULL, request, 4u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Port_Transmit(&port, NULL, 4u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Port_Transmit(&port, request, 3u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Port_Receive(NULL, data, 8u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Port_Receive(&port, NULL, 8u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Port_FlushRx(NULL) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Port_Deinit(&port) == TMC2209_OK);
    CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Port_Receive(&port, data, 8u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Port_FlushRx(&port) == TMC2209_ERR_PARAM);
    port = hal_port();
    CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_ERR_PARAM);
    port = start(); fake_dwt.CTRL = 0u;
    CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_ERR_PARAM);
}

static void test_tx_failures(void)
{
    uint8_t data[8];
    for (unsigned result = HAL_ERROR; result <= HAL_TIMEOUT; ++result) {
        tmc2209_port_t port = start();
        hal_fake.tx_mode_result = (HAL_StatusTypeDef)result;
        hal_fake.rx_mode_result = HAL_ERROR; /* Cleanup cannot replace first error. */
        tmc2209_status_t expected = result == HAL_TIMEOUT ? TMC2209_ERR_TIMEOUT :
                                    (result == HAL_BUSY ? TMC2209_ERR_BUSY : TMC2209_ERR_PORT);
        CHECK(TMC2209_Port_Transmit(&port, request, 4u) == expected);
        CHECK(hal_fake.writes == 0u && !port.awaiting_reply);
    }
    for (unsigned byte = 0u; byte < 8u; ++byte) {
        tmc2209_port_t port = start();
        hal_fake.block_txe_at = byte;
        CHECK(TMC2209_Port_Transmit(&port, write_frame, 8u) == TMC2209_ERR_TIMEOUT);
        CHECK(hal_fake.writes == byte && !port.awaiting_reply);
        hal_fake.block_txe_at = UINT_MAX;
        CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_OK);
        CHECK(TMC2209_Port_Receive(&port, data, 8u) == TMC2209_OK);
    }
    tmc2209_port_t port = start();
    hal_fake.stuck_tc = true; fake_dwt.CYCCNT = UINT32_MAX - 20000u;
    CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_ERR_TIMEOUT);
    CHECK(!port.awaiting_reply);
    hal_fake.stuck_tc = false;
    CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_OK);
    CHECK(TMC2209_Port_Receive(&port, data, 8u) == TMC2209_OK);
    port = start(); hal_fake.txe_delay = 3u; hal_fake.tc_delay = 20u;
    CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_OK);
    hal_fake.rx_delay = 3u;
    CHECK(TMC2209_Port_Receive(&port, data, 8u) == TMC2209_OK);
    port = start(); hal_fake.cycle_step = 0u;
    CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_ERR_TIMEOUT);
    port = start(); hal_fake.stuck_rx = true;
    CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_ERR_TIMEOUT);
    CHECK(hal_fake.writes == 0u);
    port = start(); hal_fake.stuck_rx_after_tx = true;
    CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_ERR_TIMEOUT);
    CHECK(!port.awaiting_reply);
    port = start(); hal_fake.rx_mode_result = HAL_ERROR;
    CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_ERR_PORT);
    CHECK(!port.awaiting_reply);
    for (unsigned phase = 0u; phase <= 8u; ++phase) {
        port = start(); hal_fake.tx_error_at = phase;
        CHECK(TMC2209_Port_Transmit(&port, write_frame, 8u) == TMC2209_ERR_PORT);
        CHECK(!port.awaiting_reply);
    }
}

static void test_rx_failures(void)
{
    uint8_t data[8];
    for (unsigned bytes = 0u; bytes < 8u; ++bytes) {
        tmc2209_port_t port = start(); hal_fake.reply_bytes = bytes;
        CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_OK);
        CHECK(TMC2209_Port_Receive(&port, data, 8u) == TMC2209_ERR_TIMEOUT);
        CHECK(!port.awaiting_reply);
        hal_fake.reply_bytes = 8u;
        CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_OK);
        CHECK(TMC2209_Port_Receive(&port, data, 8u) == TMC2209_OK);
    }
    for (unsigned flag = 0u; flag < 4u; ++flag) {
        for (unsigned phase = 0u; phase <= 8u; ++phase) {
            tmc2209_port_t port = start();
            CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_OK);
            hal_fake.rx_error_at = phase; hal_fake.inject_error = 1u << flag;
            hal_fake.rx_mode_result = HAL_TIMEOUT;
            CHECK(TMC2209_Port_Receive(&port, data, 8u) == TMC2209_ERR_PORT);
            CHECK(!port.awaiting_reply);
        }
    }
    tmc2209_port_t port = start();
    CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_OK);
    hal_fake.reply_bytes = 0u;
    hal_fake.fifo_size = 0u; hal_fake.cycle_step = 0u;
    CHECK(TMC2209_Port_Receive(&port, data, 8u) == TMC2209_ERR_TIMEOUT);
    CHECK(hal_fake.status_calls < 600u); /* Frozen counter cannot loop forever. */
    port = start(); hal_fake.cycle_step = 180000u;
    CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_OK);
    hal_fake.cycle_step = 720000u; /* Bytes ready, but whole-frame deadline exceeded. */
    CHECK(TMC2209_Port_Receive(&port, data, 8u) == TMC2209_ERR_TIMEOUT);
    port = start();
    CHECK(TMC2209_Port_Transmit(&port, request, 4u) == TMC2209_OK);
    fake_dwt.CYCCNT = UINT32_MAX - 36000u;
    CHECK(TMC2209_Port_Receive(&port, data, 8u) == TMC2209_OK);
}

int main(void)
{
    RUN(test_init_parameters);
    RUN(test_init_failures_and_clock);
    RUN(test_turnaround_and_echo);
    RUN(test_tx_failures);
    RUN(test_rx_failures);
    SUMMARY("Production HAL port");
    return EXIT_SUCCESS;
}
