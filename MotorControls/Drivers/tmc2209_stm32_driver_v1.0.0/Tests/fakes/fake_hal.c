#include "fake_hal.h"
#include "tmc2209_test_mmio.h"
#include "test_support.h"

fake_hal_t hal_fake;
DWT_Type fake_dwt;
CoreDebug_Type fake_debug;
uint32_t SystemCoreClock;

void hal_chip_reset(uint8_t address)
{
    REQUIRE(address < 4u);
    uint32_t ioin = hal_fake.registers[address][6];
    memset(hal_fake.registers[address], 0, sizeof(hal_fake.registers[address]));
    hal_fake.registers[address][0] = 0x101u;
    hal_fake.registers[address][1] = 1u;
    hal_fake.registers[address][6] = ioin;
    hal_fake.registers[address][0x10] = 0x1F00u;
    hal_fake.registers[address][0x11] = 20u;
    hal_fake.registers[address][0x6C] = 0x10000053u;
    hal_fake.registers[address][0x70] = 0xC10D0024u;
}

void hal_reset(void)
{
    memset(&hal_fake, 0, sizeof(hal_fake));
    memset(&fake_dwt, 0, sizeof(fake_dwt));
    memset(&fake_debug, 0, sizeof(fake_debug));
    SystemCoreClock = 180000000u;
    hal_fake.uart.Instance = &hal_fake.uart_regs;
    hal_fake.uart.Init.BaudRate = 115200u;
    hal_fake.uart.Init.Mode = UART_MODE_TX_RX;
    hal_fake.uart.gState = HAL_UART_STATE_READY;
    hal_fake.uart.RxState = HAL_UART_STATE_READY;
    hal_fake.uart_regs.CR1 = USART_CR1_UE;
    hal_fake.uart_regs.CR3 = USART_CR3_HDSEL;
    hal_fake.cycle_step = 18000u;
    hal_fake.reply_bytes = 8u;
    hal_fake.block_txe_at = UINT_MAX;
    hal_fake.rx_error_at = UINT_MAX;
    hal_fake.tx_error_at = UINT_MAX;
    hal_fake.inject_error = USART_SR_ORE;
    for (uint8_t a = 0u; a < 4u; ++a) {
        hal_fake.registers[a][6] = 0x21000000u;
        hal_chip_reset(a);
    }
}

tmc2209_port_t hal_port(void)
{
    tmc2209_port_t port = {&hal_fake.uart, 115200u, 20u, 20u, 0u, false, false};
    return port;
}

uint32_t fake_hal_cycles(void)
{
    fake_dwt.CYCCNT += hal_fake.cycle_step;
    return fake_dwt.CYCCNT;
}

uint32_t fake_hal_status(UART_HandleTypeDef *h)
{
    ++hal_fake.status_calls;
    uint32_t sr = h->Instance->SR & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE);
    if ((h->Instance->CR1 & USART_CR1_TE) != 0u) {
        if (hal_fake.txe_delay != 0u) --hal_fake.txe_delay;
        else if (hal_fake.tx_size < hal_fake.block_txe_at) sr |= USART_SR_TXE;
        if (hal_fake.tc_delay != 0u) --hal_fake.tc_delay;
        else if (!hal_fake.stuck_tc && hal_fake.tx_size == hal_fake.expected_size)
            sr |= USART_SR_TC;
        if (hal_fake.tx_size >= hal_fake.tx_error_at) sr |= hal_fake.inject_error;
    } else if ((h->Instance->CR1 & USART_CR1_RE) != 0u &&
               hal_fake.rx_read_count >= hal_fake.rx_error_at) {
        sr |= hal_fake.inject_error;
    }
    if (hal_fake.stuck_rx || (hal_fake.stuck_rx_after_tx &&
        hal_fake.tx_size == hal_fake.expected_size)) sr |= USART_SR_RXNE;
    else if (hal_fake.rx_delay != 0u) --hal_fake.rx_delay;
    else if (hal_fake.fifo_pos < hal_fake.fifo_size) sr |= USART_SR_RXNE;
    return sr;
}

uint8_t fake_hal_read(UART_HandleTypeDef *h)
{
    (void)h;
    ++hal_fake.reads;
    if (hal_fake.fifo_pos >= hal_fake.fifo_size) return 0u;
    if ((hal_fake.uart_regs.CR1 & USART_CR1_RE) != 0u) ++hal_fake.rx_read_count;
    return hal_fake.fifo[hal_fake.fifo_pos++];
}

void fake_hal_write(UART_HandleTypeDef *h, uint8_t value)
{
    REQUIRE((h->Instance->CR1 & USART_CR1_TE) != 0u);
    REQUIRE(hal_fake.tx_size < sizeof(hal_fake.tx));
    ++hal_fake.writes;
    hal_fake.tx[hal_fake.tx_size++] = value;
    if (hal_fake.echo && hal_fake.fifo_size < sizeof(hal_fake.fifo))
        hal_fake.fifo[hal_fake.fifo_size++] = value;
    if (hal_fake.tx_size == 3u)
        hal_fake.expected_size = (value & 128u) != 0u ? 8u : 4u;
    if (hal_fake.tx_size != hal_fake.expected_size) return;
    ++hal_fake.frames;
    uint8_t *tx = hal_fake.tx;
    if (reference_crc(tx, hal_fake.tx_size - 1u) != tx[hal_fake.tx_size - 1u])
        return;
    uint8_t a = tx[1], r = (uint8_t)(tx[2] & 127u);
    REQUIRE(a <= 3u);
    if (hal_fake.expected_size == 8u) {
        if (hal_fake.drop_write == hal_fake.frames) return;
        uint32_t v = ((uint32_t)tx[3] << 24u) | ((uint32_t)tx[4] << 16u) |
                     ((uint32_t)tx[5] << 8u) | tx[6];
        if (r == 1u) {
            ++hal_fake.gstat_writes;
            uint32_t clear = v;
            if ((hal_fake.registers[a][0x6F] & 0x3Eu) != 0u) clear &= ~UINT32_C(2);
            hal_fake.registers[a][r] &= ~clear;
        } else {
            hal_fake.registers[a][r] = v;
            if (r == 0x6Cu && (v & 15u) == 0u)
                hal_fake.registers[a][0x6F] &= ~UINT32_C(0x3C);
        }
        hal_fake.registers[a][2] = (uint8_t)(hal_fake.registers[a][2] + 1u);
        ++hal_fake.accepted_writes;
        if (r == 0x6Cu && (v & 15u) != 0u) ++hal_fake.active_writes;
    } else {
        reference_reply(r, hal_fake.registers[a][r], hal_fake.reply);
        if (hal_fake.corrupt_crc) hal_fake.reply[7] ^= 1u;
        if (hal_fake.corrupt_identity) hal_fake.reply[2] |= 128u;
        hal_fake.pending_reply = true;
    }
}

void fake_hal_clear_errors(UART_HandleTypeDef *h)
{
    ++hal_fake.clears;
    h->Instance->SR &= ~(uint32_t)(USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE);
    /* Simulate the DR read in the real F4 clear sequence, including data loss. */
    if (hal_fake.fifo_pos < hal_fake.fifo_size) ++hal_fake.fifo_pos;
}

HAL_StatusTypeDef HAL_HalfDuplex_EnableTransmitter(UART_HandleTypeDef *h)
{
    ++hal_fake.tx_mode_calls;
    if (hal_fake.tx_mode_result != HAL_OK &&
        (hal_fake.fail_tx_mode_call == 0u || hal_fake.fail_tx_mode_call == hal_fake.tx_mode_calls))
        return hal_fake.tx_mode_result;
    h->Instance->CR1 = (h->Instance->CR1 & ~(uint32_t)USART_CR1_RE) | USART_CR1_TE;
    hal_fake.tx_size = 0u;
    hal_fake.expected_size = 4u;
    return HAL_OK;
}

HAL_StatusTypeDef HAL_HalfDuplex_EnableReceiver(UART_HandleTypeDef *h)
{
    ++hal_fake.rx_mode_calls;
    if (hal_fake.rx_mode_result != HAL_OK &&
        (hal_fake.fail_rx_mode_call == 0u || hal_fake.fail_rx_mode_call == hal_fake.rx_mode_calls))
        return hal_fake.rx_mode_result;
    h->Instance->CR1 = (h->Instance->CR1 & ~(uint32_t)USART_CR1_TE) | USART_CR1_RE;
    hal_fake.rx_read_count = 0u;
    if (hal_fake.pending_reply) {
        REQUIRE(hal_fake.reply_bytes <= 8u);
        hal_fake.fifo_size = hal_fake.reply_bytes;
        hal_fake.fifo_pos = 0u;
        memcpy(hal_fake.fifo, hal_fake.reply, hal_fake.reply_bytes);
        hal_fake.pending_reply = false;
    } else if (hal_fake.fifo_pos == hal_fake.fifo_size) {
        hal_fake.fifo_size = 0u;
        hal_fake.fifo_pos = 0u;
    }
    return HAL_OK;
}

void HAL_Delay(uint32_t ms) { hal_fake.delayed_ms += ms; hal_fake.tick += ms; }
uint32_t HAL_GetTick(void) { return hal_fake.tick; }
