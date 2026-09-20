#include "model_hal.h"
#include "test_support.h"

SPI_TypeDef fake_spi_instances[4] = {{0U},{1U},{2U},{3U}};
GPIO_TypeDef fake_gpio_ports[3] = {{0U},{1U},{2U}};
uint32_t SystemCoreClock;
uint32_t hal_pclk1, hal_pclk2;
SPI_HandleTypeDef hal_handles[4];
TMC2240_SPIConfig_t hal_configs[4];
HalTrace hal_trace[1024];
unsigned hal_trace_count, hal_calls, hal_fail_at, hal_fail_also;
unsigned hal_drop_write_at, hal_corrupt_at;
unsigned hal_status_at;
uint8_t hal_status_bits;
bool hal_fail_after_accept;
HAL_StatusTypeDef hal_failure;
uint64_t hal_nops;
bool hal_selected[4];

void hal_trace_reset(void)
{
    hal_trace_count = 0U;
    hal_calls = 0U;
    hal_fail_at = 0U;
    hal_fail_also = 0U;
    hal_drop_write_at = 0U;
    hal_corrupt_at = 0U;
    hal_status_at = 0U;
    hal_status_bits = 1U;
    hal_fail_after_accept = false;
    hal_failure = HAL_TIMEOUT;
}

void hal_model_reset(void)
{
    model_reset_devices();
    hal_trace_reset();
    SystemCoreClock = 80000000U;
    hal_pclk1 = 40000000U;
    hal_pclk2 = 80000000U;
    hal_nops = 0U;
    memset(hal_selected, 0, sizeof(hal_selected));
    for (unsigned i = 0U; i < 4U; ++i) {
        SPI_HandleTypeDef *spi = &hal_handles[i];
        memset(spi, 0, sizeof(*spi));
        spi->Instance = &fake_spi_instances[i];
        spi->Init.Mode = SPI_MODE_MASTER;
        spi->Init.CLKPolarity = SPI_POLARITY_HIGH;
        spi->Init.CLKPhase = SPI_PHASE_2EDGE;
        spi->Init.NSS = SPI_NSS_SOFT;
        spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
        spi->State = HAL_SPI_STATE_READY;
        hal_configs[i].hspi = &hal_handles[i == 1U ? 0U : i];
        hal_configs[i].cs_port = (i < 2U) ? GPIOA : ((i == 2U) ? GPIOB : GPIOC);
        hal_configs[i].cs_pin = (uint16_t)(1U << i);
        hal_configs[i].spi_timeout_ms = 10U;
    }
}

uint32_t HAL_RCC_GetPCLK1Freq(void) { return hal_pclk1; }
uint32_t HAL_RCC_GetPCLK2Freq(void) { return hal_pclk2; }
void model_nop(void) { ++hal_nops; }

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState state)
{
    unsigned id;
    CHECK(port != NULL && pin != 0U);
    for (id = 0U; id < 4U; ++id) {
        if (hal_configs[id].cs_port == port && hal_configs[id].cs_pin == pin) {
            break;
        }
    }
    CHECK(id < 4U && hal_trace_count < 1024U);
    HalTrace *trace = &hal_trace[hal_trace_count++];
    trace->kind = 'G';
    trace->id = id;
    trace->time = hal_nops;
    trace->level = state;
    if (state == GPIO_PIN_RESET) {
        CHECK(!hal_selected[id]);
    }
    hal_selected[id] = state == GPIO_PIN_RESET;
}

HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *spi, uint8_t *tx,
    uint8_t *rx, uint16_t length, uint32_t timeout)
{
    unsigned id;
    unsigned selected = 0U;
    CHECK(spi != NULL && tx != NULL && rx != NULL && tx != rx);
    CHECK(length == 5U && timeout > 0U && timeout <= INT32_MAX);
    for (id = 0U; id < 4U; ++id) {
        if (hal_selected[id] && hal_configs[id].hspi == spi) {
            selected = id;
            break;
        }
    }
    CHECK(id < 4U && hal_trace_count < 1024U);
    HalTrace *trace = &hal_trace[hal_trace_count++];
    trace->kind = 'T';
    trace->id = selected;
    trace->time = hal_nops;
    memcpy(trace->tx, tx, 5U);
    ++hal_calls;
    bool failure = (hal_calls == hal_fail_at) || (hal_calls == hal_fail_also);
    if (!failure || hal_fail_after_accept) {
        bool drop = hal_calls == hal_drop_write_at;
        model_devices[selected].ignore_write = drop;
        model_spi(selected, tx, rx);
        model_devices[selected].ignore_write = false;
        if (hal_calls == hal_corrupt_at) {
            rx[4] ^= 1U;
        }
        if (hal_calls == hal_status_at) {
            rx[0] |= hal_status_bits;
        }
    } else {
        memset(rx, 0xEE, 5U);
    }
    return failure ? hal_failure : HAL_OK;
}

void hal_check_trace(void)
{
    for (unsigned id = 0U; id < 4U; ++id) {
        CHECK(!hal_selected[id]);
    }
    uint64_t last_high[4] = {0U};
    for (unsigned i = 0U; i < hal_trace_count; ++i) {
        HalTrace *trace = &hal_trace[i];
        if (trace->kind == 'T') {
            CHECK(i != 0U && i + 1U < hal_trace_count);
            CHECK(hal_trace[i - 1U].kind == 'G' && hal_trace[i - 1U].level == GPIO_PIN_RESET);
            CHECK(hal_trace[i + 1U].kind == 'G' && hal_trace[i + 1U].level == GPIO_PIN_SET);
            CHECK(hal_trace[i - 1U].id == trace->id && hal_trace[i + 1U].id == trace->id);
            CHECK(trace->time - hal_trace[i - 1U].time >= SystemCoreClock / 1000000U);
            CHECK(hal_trace[i + 1U].time - trace->time >= SystemCoreClock / 1000000U);
        } else if (trace->level == GPIO_PIN_RESET) {
            CHECK(trace->time - last_high[trace->id] >= SystemCoreClock / 1000000U);
        } else {
            last_high[trace->id] = trace->time;
        }
    }
}
