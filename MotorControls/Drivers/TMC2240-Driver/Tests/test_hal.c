#include "model_hal.h"
#include "model_cache.h"
#include "test_support.h"

#if TMC2240_CACHE && TMC2240_ENABLE_TMC_CACHE && TMC2240_IC_CACHE_COUNT < 4
#define TEST_ICS TMC2240_IC_CACHE_COUNT
#else
#define TEST_ICS 4U
#endif

static const TMC2240_MotorConfig_t motor = {
    0x00000008U, 0x21U, 0U, 0x0401150AU, 0x14410150U, 0xC40C001DU
};

static void uninitialized(void)
{
    custom_cache_reset_faults();
    OK(tmc2240_hal_deinit());
    hal_model_reset();
}

static void fresh(void)
{
    uninitialized();
    OK(tmc2240_hal_init(hal_configs, (uint8_t)TEST_ICS));
    CHECK(hal_calls == 0U);
    for (unsigned i = 0U; i < TEST_ICS; ++i) {
        CHECK(model_devices[i].writes == 0U && model_devices[i].activations == 0U);
    }
    hal_trace_reset();
}

static void prepare(void)
{
    fresh();
    OK(tmc2240_clear_faults(0U, 0x1FU));
    OK(tmc2240_hal_configureMotor(0U, &motor));
    CHECK((model_devices[0].reg[0x6C] & 15U) == 0U);
    hal_trace_reset();
}

static void no_access(void)
{
    CHECK(hal_calls == 0U && hal_trace_count == 0U);
}

static void invalid_init(TMC2240Status expected, uint8_t count)
{
    STATUS(tmc2240_hal_init(hal_configs, count), expected);
    CHECK(!tmc2240_hal_isInitialized());
    no_access();
}

static void test_init_validation(void)
{
    uninitialized();
    CHECK(!tmc2240_hal_isInitialized());
    STATUS(tmc2240_hal_init(NULL, 1U), TMC2240_ERROR_ARGUMENT);
    invalid_init(TMC2240_ERROR_ARGUMENT, 0U);
    invalid_init(TMC2240_ERROR_ARGUMENT, TMC2240_HAL_MAX_ICS + 1U);
#if TEST_ICS < 4
    invalid_init(TMC2240_ERROR_RANGE, 4U);
#endif
    hal_configs[0].hspi = NULL;
    invalid_init(TMC2240_ERROR_ARGUMENT, 1U);
    hal_model_reset();
    hal_configs[0].cs_port = NULL;
    invalid_init(TMC2240_ERROR_ARGUMENT, 1U);
    hal_model_reset();
    GPIO_TypeDef foreign_gpio = {7U};
    hal_configs[0].cs_port = &foreign_gpio;
    invalid_init(TMC2240_ERROR_ARGUMENT, 1U);
    hal_model_reset();
    hal_handles[0].Instance = NULL;
    invalid_init(TMC2240_ERROR_ARGUMENT, 1U);
    hal_model_reset();
    SPI_TypeDef foreign_spi = {7U};
    hal_handles[0].Instance = &foreign_spi;
    invalid_init(TMC2240_ERROR_ARGUMENT, 1U);
    hal_model_reset();
    hal_configs[0].cs_pin = 0U;
    invalid_init(TMC2240_ERROR_ARGUMENT, 1U);
    hal_configs[0].cs_pin = 3U;
    invalid_init(TMC2240_ERROR_ARGUMENT, 1U);
    hal_model_reset();
    hal_configs[0].spi_timeout_ms = 0U;
    invalid_init(TMC2240_ERROR_RANGE, 1U);
    hal_configs[0].spi_timeout_ms = UINT32_MAX;
    invalid_init(TMC2240_ERROR_RANGE, 1U);
    hal_model_reset();
    uint32_t *settings[] = {
        &hal_handles[0].Init.Mode, &hal_handles[0].Init.Direction,
        &hal_handles[0].Init.DataSize, &hal_handles[0].Init.CLKPolarity,
        &hal_handles[0].Init.CLKPhase, &hal_handles[0].Init.NSS,
        &hal_handles[0].Init.FirstBit, &hal_handles[0].Init.TIMode,
        &hal_handles[0].Init.CRCCalculation, &hal_handles[0].Init.BaudRatePrescaler
    };
    for (unsigned i = 0U; i < sizeof(settings) / sizeof(settings[0]); ++i) {
        hal_model_reset();
        *settings[i] = UINT32_MAX;
        invalid_init(TMC2240_ERROR_ARGUMENT, 1U);
    }
    hal_model_reset();
    hal_handles[0].Init.BaudRatePrescaler = 9U;
    invalid_init(TMC2240_ERROR_ARGUMENT, 1U);
    hal_model_reset();
    hal_handles[0].State = HAL_SPI_STATE_BUSY;
    invalid_init(TMC2240_ERROR_BUSY, 1U);
    hal_model_reset();
    SystemCoreClock = 999999U;
    invalid_init(TMC2240_ERROR_RANGE, 1U);
    SystemCoreClock = 1000000001U;
    invalid_init(TMC2240_ERROR_RANGE, 1U);
    hal_model_reset();
    hal_pclk2 = 999999U;
    invalid_init(TMC2240_ERROR_RANGE, 1U);
    hal_pclk2 = 80000001U;
    invalid_init(TMC2240_ERROR_RANGE, 1U);
    hal_model_reset();
    hal_configs[0].hspi = &hal_handles[1];
    hal_pclk1 = 80000001U;
    invalid_init(TMC2240_ERROR_RANGE, 1U);
#if TEST_ICS > 1
    hal_model_reset();
    hal_configs[1].cs_port = hal_configs[0].cs_port;
    hal_configs[1].cs_pin = hal_configs[0].cs_pin;
    invalid_init(TMC2240_ERROR_ARGUMENT, 2U);
    hal_model_reset();
    hal_handles[1].Instance = SPI1;
    hal_configs[1].hspi = &hal_handles[1];
    invalid_init(TMC2240_ERROR_ARGUMENT, 2U);
    hal_model_reset();
    hal_configs[TEST_ICS - 1U].cs_pin = 0U;
    invalid_init(TMC2240_ERROR_ARGUMENT, (uint8_t)TEST_ICS);
#endif
#if TMC2240_CACHE && !TMC2240_ENABLE_TMC_CACHE
    hal_model_reset();
    custom_cache_fail_at = 2U;
    invalid_init(TMC2240_ERROR_IO, 4U);
    custom_cache_reset_faults();
#endif
    hal_model_reset();
    hal_configs[0].cs_pin = 0x8000U;
    hal_configs[0].spi_timeout_ms = INT32_MAX;
    model_devices[0].reg[0x6C] = 0x10410157U;
    OK(tmc2240_hal_init(hal_configs, 1U));
    CHECK(tmc2240_hal_isInitialized() && hal_calls == 0U && hal_trace_count == 1U);
    CHECK(model_devices[0].reg[0x6C] == 0x10410157U); /* Init is NOT a hardware disable. */
    CHECK(model_devices[0].reg[1] == 0x1DU && model_devices[0].writes == 0U);
    hal_trace_reset();
    STATUS(tmc2240_hal_init(hal_configs, 1U), TMC2240_ERROR_STATE);
    no_access();
    CHECK(tmc2240_hal_isInitialized());
    OK(tmc2240_hal_deinit());
    CHECK(!tmc2240_hal_isInitialized() && hal_calls == 0U);
    hal_trace_reset();
    OK(tmc2240_hal_deinit());
    no_access();
}

static void test_spi(void)
{
    uint8_t frame[5] = {0x39U,0U,0U,0U,0U};
    uint8_t status_byte = 99U;
    TMC2240BusType bus = IC_BUS_WLAN;
    uninitialized();
    STATUS(tmc2240_readWriteSPI(0U, frame, 5U), TMC2240_ERROR_NOT_INITIALIZED);
    STATUS(tmc2240_getBusType(0U, &bus), TMC2240_ERROR_NOT_INITIALIZED);
    STATUS(tmc2240_hal_getSPIStatus(0U, &status_byte), TMC2240_ERROR_NOT_INITIALIZED);
    CHECK(tmc2240_hal_spiStatus(0U) == HAL_ERROR);
    CHECK(bus == IC_BUS_WLAN && status_byte == 99U);
    no_access();
    fresh();
    CHECK(tmc2240_hal_spiStatus(0U) == HAL_ERROR);
    STATUS(tmc2240_getBusType(0U, NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_getBusType(UINT16_MAX, &bus), TMC2240_ERROR_ID);
    OK(tmc2240_getBusType(0U, &bus));
    CHECK(bus == IC_BUS_SPI);
    STATUS(tmc2240_hal_getSPIStatus(0U, NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_hal_getSPIStatus(UINT16_MAX, &status_byte), TMC2240_ERROR_ID);
    STATUS(tmc2240_hal_getSPIStatus(0U, &status_byte), TMC2240_ERROR_STATE);
    STATUS(tmc2240_readWriteSPI(UINT16_MAX, frame, 5U), TMC2240_ERROR_ID);
    STATUS(tmc2240_readWriteSPI(0U, NULL, 5U), TMC2240_ERROR_ARGUMENT);
    const size_t bad_lengths[] = {0U,1U,4U,6U,SIZE_MAX};
    for (size_t i = 0U; i < sizeof(bad_lengths) / sizeof(bad_lengths[0]); ++i) {
        STATUS(tmc2240_readWriteSPI(0U, frame, bad_lengths[i]), TMC2240_ERROR_ARGUMENT);
    }
    STATUS(tmc2240_readWriteUART(0U, frame, 4U, 8U), TMC2240_ERROR_UNSUPPORTED);
    STATUS(tmc2240_readWriteUART(UINT16_MAX, NULL, 0U, SIZE_MAX), TMC2240_ERROR_UNSUPPORTED);
    STATUS(tmc2240_getNodeAddress(0U, &status_byte), TMC2240_ERROR_UNSUPPORTED);
    STATUS(tmc2240_getNodeAddress(UINT16_MAX, NULL), TMC2240_ERROR_UNSUPPORTED);
    no_access();
    hal_handles[0].Init.CLKPhase = 0U;
    STATUS(tmc2240_readWriteSPI(0U, frame, 5U), TMC2240_ERROR_ARGUMENT);
    hal_handles[0].Init.CLKPhase = SPI_PHASE_2EDGE;
    no_access();
    for (unsigned id = 0U; id < TEST_ICS; ++id) {
        hal_trace_reset();
        model_devices[id].reg[0x39] = 0x89ABCDEFU + id;
        uint32_t value;
        OK(tmc2240_readRegister((uint16_t)id, 0x39U, &value));
        CHECK(value == 0x89ABCDEFU + id && hal_calls == 2U);
        CHECK(hal_trace_count == 6U);
        CHECK(hal_trace[1].tx[0] == 0x39U && model_unpack(&hal_trace[1].tx[1]) == 0U);
        CHECK(memcmp(hal_trace[1].tx, hal_trace[4].tx, 5U) == 0);
        OK(tmc2240_hal_getSPIStatus((uint16_t)id, &status_byte));
        CHECK(status_byte == 9U);
        CHECK(tmc2240_hal_spiStatus((uint16_t)id) == HAL_OK);
        hal_check_trace();
    }
    const HAL_StatusTypeDef errors[] = {HAL_ERROR,HAL_BUSY,HAL_TIMEOUT,(HAL_StatusTypeDef)99};
    const TMC2240Status expected[] = {TMC2240_ERROR_IO,TMC2240_ERROR_BUSY,
                                    TMC2240_ERROR_TIMEOUT,TMC2240_ERROR_IO};
    for (unsigned e = 0U; e < 4U; ++e) {
        for (unsigned phase = 1U; phase <= 2U; ++phase) {
            hal_trace_reset();
            hal_failure = errors[e];
            hal_fail_at = phase;
            uint32_t output = 0xCAFEBABEU;
            STATUS(tmc2240_readRegister(0U, 0x39U, &output), expected[e]);
            CHECK(output == 0xCAFEBABEU && hal_calls == phase);
            CHECK(tmc2240_hal_spiStatus(0U) == errors[e]);
            STATUS(tmc2240_hal_getSPIStatus(0U, &status_byte), TMC2240_ERROR_STATE);
            hal_check_trace();
        }
    }
    hal_trace_reset();
    hal_fail_at = 1U;
    const uint8_t saved[5] = {0x39U,0U,0U,0U,0U};
    memcpy(frame, saved, sizeof(frame));
    STATUS(tmc2240_readWriteSPI(0U, frame, sizeof(frame)), TMC2240_ERROR_TIMEOUT);
    CHECK(memcmp(frame, saved, sizeof(frame)) == 0);
    hal_check_trace();
    uninitialized();
    hal_pclk2 = 1000000U;
    hal_handles[0].Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    OK(tmc2240_hal_init(hal_configs, 1U));
    hal_trace_reset();
    OK(tmc2240_readWriteSPI(0U, frame, 5U));
    CHECK(hal_trace[1].time - hal_trace[0].time >= SystemCoreClock / (1000000U / 256U));
    hal_check_trace();
}

typedef struct { uint32_t u32; int32_t s32; uint16_t u16; int16_t a,b; uint8_t u8; } Outputs;
static Outputs outputs;

static void setup_operation(unsigned op)
{
    if (op == 32U || op == 33U) {
        prepare();
    } else {
        fresh();
        model_devices[0].reg[1] = 0U;
        model_devices[0].status_pipeline = 8U;
        model_devices[0].reg[0x0A] = 1U;
        if (op == 30U) {
            model_devices[0].reg[0x6C] |= 3U;
            model_devices[0].reg[0x10] = 0x04011F1FU;
        }
    }
    memset(&outputs, 0xA5, sizeof(outputs));
}

static TMC2240Status operation(unsigned op, uint16_t id)
{
    switch (op) {
    case 0: return tmc2240_hal_setCurrent(id, 1000U, 12000U, 50U, 5U);
    case 1: return tmc2240_hal_setMicrosteps(id, 16U);
    case 2: return tmc2240_stealthchop_enable(id, 1U);
    case 3: return tmc2240_stealthchop_enable(id, 0U);
    case 4: return tmc2240_stallguard_set_threshold(id, 75U);
    case 5: return tmc2240_stallguard_read(id, &outputs.u16);
    case 6: return tmc2240_coolstep_configure(id, 2U, 3U, 1U, 2U, -6);
    case 7: return tmc2240_driver_status(id, &outputs.u32);
    case 8: return tmc2240_check_faults(id, &outputs.u8);
    case 9: return tmc2240_clear_faults(id, 0x1FU);
    case 10: return tmc2240_read_vsupply(id, &outputs.u16);
    case 11: return tmc2240_get_vsupply_mV(id, &outputs.u32);
    case 12: return tmc2240_read_temperature(id, &outputs.u16);
    case 13: return tmc2240_get_temperature_c10(id, &outputs.a);
    case 14: return tmc2240_encoder_init(id, -65536);
    case 15: return tmc2240_encoder_read(id, &outputs.s32);
    case 16: return tmc2240_encoder_read_latch(id, &outputs.s32);
    case 17: return tmc2240_encoder_get_status(id, &outputs.u8);
    case 18: return tmc2240_encoder_clear_n_event(id);
    case 19: return tmc2240_set_tpwmthrs(id, 1000U);
    case 20: return tmc2240_set_tcoolthrs(id, 1000U);
    case 21: return tmc2240_set_thigh(id, 1000U);
    case 22: return tmc2240_set_chopper(id, 0U, 5U, 2U, 2U);
    case 23: return tmc2240_set_tpowerdown(id, 10U);
    case 24: return tmc2240_diag_configure(id, 1U, 1U, 1U, 1U, 1U, 1U, 1U);
    case 25: return tmc2240_pwm_get_scale(id, &outputs.u16, &outputs.a);
    case 26: return tmc2240_get_microstep_counter(id, &outputs.u16);
    case 27: return tmc2240_get_microstep_current(id, &outputs.a, &outputs.b);
    case 28: return tmc2240_hal_testConnection(id);
    case 29: return tmc2240_hal_disableMotor(id);
    case 30: return tmc2240_stealthchop_enable(id, 1U);
    case 31: return tmc2240_hal_configureMotor(id, &motor);
    case 32: return tmc2240_hal_activateMotor(id, 3U);
    case 33: return tmc2240_set_chopper(id, 3U, 4U, 1U, 3U);
    default: return TMC2240_ERROR_ARGUMENT;
    }
}

static void test_operation_failure_matrix(void)
{
    const HAL_StatusTypeDef injected[] = {HAL_ERROR,HAL_BUSY,HAL_TIMEOUT};
    const TMC2240Status expected[] = {
        TMC2240_ERROR_IO,TMC2240_ERROR_BUSY,TMC2240_ERROR_TIMEOUT
    };
    for (unsigned op = 0U; op < 34U; ++op) {
        setup_operation(op);
        OK(operation(op, 0U));
        unsigned phases = hal_calls;
        CHECK(phases != 0U);
        hal_check_trace();
        setup_operation(op);
        STATUS(operation(op, UINT16_MAX), TMC2240_ERROR_ID);
        no_access();
        for (unsigned accept = 0U; accept < 2U; ++accept) {
            for (unsigned phase = 1U; phase <= phases; ++phase) {
                for (unsigned error = 0U; error < 3U; ++error) {
                    setup_operation(op);
                    Outputs unchanged;
                    memcpy(&unchanged, &outputs, sizeof(outputs));
                    hal_fail_at = phase;
                    hal_fail_after_accept = accept != 0U;
                    hal_failure = injected[error];
                    TMC2240Status status = operation(op, 0U);
                    if (status != expected[error]) {
                        fprintf(stderr, "operation %u phase %u accepted %u error %u -> status %d\n",
                                op, phase, accept, error, (int)status);
                    }
                    CHECK(status == expected[error]);
                    CHECK(memcmp(&outputs, &unchanged, sizeof(outputs)) == 0);
                    CHECK((model_devices[0].reg[0x6C] & 15U) == 0U || op == 30U);
                    hal_check_trace();
                }
            }
        }
    }
    uninitialized();
    for (unsigned op = 0U; op < 34U; ++op) {
        STATUS(operation(op, 0U), TMC2240_ERROR_NOT_INITIALIZED);
    }
    no_access();
}

static void test_current(void)
{
    uint32_t bits = 0xA5A5A5A5U;
    fresh();
    STATUS(tmc2240_calculateCurrent(1000U,12000U,1U,0U,50U,5U,4U,NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_calculateCurrent(1000U,12000U,4U,0U,50U,5U,4U,&bits), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_calculateCurrent(1000U,12000U,1U,0U,50U,5U,16U,&bits), TMC2240_ERROR_RANGE);
    const uint16_t run[] = {0U,65535U,1000U,1000U,1000U,1000U,1000U};
    const uint16_t rref[] = {12000U,12000U,11999U,60001U,12000U,12000U,12000U};
    const uint8_t hold[] = {50U,50U,50U,50U,0U,101U,50U};
    const uint8_t delay[] = {5U,5U,5U,5U,5U,5U,16U};
    for (unsigned i = 0U; i < sizeof(run) / sizeof(run[0]); ++i) {
        STATUS(tmc2240_hal_setCurrent(0U,run[i],rref[i],hold[i],delay[i]), TMC2240_ERROR_RANGE);
        STATUS(tmc2240_calculateCurrent(run[i],rref[i],1U,0U,hold[i],delay[i],4U,&bits),
               TMC2240_ERROR_RANGE);
    }
    CHECK(bits == 0xA5A5A5A5U);
    no_access();
    OK(tmc2240_calculateCurrent(1132U,12000U,1U,0U,50U,5U,4U,&bits));
    CHECK(bits == 0x0405180BU); /* 25 current units; 50% hold = 12, not 13. */
    OK(tmc2240_calculateCurrent(2000U,12000U,3U,0U,50U,5U,4U,&bits));
    CHECK(((bits >> 8U) & 31U) == 29U); /* Range3 has KIFS=36, not 11.75. */
    for (unsigned gs = 1U; gs < 32U; ++gs) {
        STATUS(tmc2240_calculateCurrent(100U,12000U,1U,(uint8_t)gs,50U,5U,4U,&bits),
               TMC2240_ERROR_RANGE);
    }
    STATUS(tmc2240_calculateCurrent(1U,60000U,0U,0U,100U,0U,0U,&bits), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_calculateCurrent(2122U,12000U,3U,0U,100U,0U,0U,&bits), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_calculateCurrent(100U,12000U,1U,0U,1U,0U,0U,&bits), TMC2240_ERROR_RANGE);
    model_devices[0].reg[0x0A] = 3U;
    model_devices[0].reg[0x0B] = 31U;
    STATUS(tmc2240_hal_setCurrent(0U, 500U, 12000U, 50U, 5U), TMC2240_ERROR_RANGE);
    CHECK(model_devices[0].writes == 0U);
    model_devices[0].reg[0x0B] = 0U;
    model_devices[0].reg[0x10] = 0x09011F08U;
    OK(tmc2240_hal_setCurrent(0U, 2000U, 12000U, 50U, 3U));
    CHECK((model_devices[0].reg[0x10] >> 24U) == 9U);
    static const long double kifs[] = {11750.0L,24000.0L,36000.0L,36000.0L};
    for (unsigned i = 0U; i < 100000U; ++i) {
        uint16_t request = (uint16_t)(1U + test_random() % 3000U);
        uint16_t resistor = (uint16_t)(12000U + test_random() % 48001U);
        uint8_t range = (uint8_t)(test_random() % 4U);
        uint8_t gs = (i % 11U == 0U) ? 0U : (uint8_t)(32U + test_random() % 224U);
        uint8_t pct = (uint8_t)(1U + test_random() % 100U);
        uint8_t hd = (uint8_t)(test_random() % 16U);
        uint8_t rd = (uint8_t)(test_random() % 16U);
        long double fullscale = kifs[range] / resistor * 1000.0L /
            1.4142135623730950488L * (gs == 0U ? 1.0L : gs / 256.0L);
        long double counts = request / fullscale * 32.0L;
        uint32_t count = (uint32_t)counts;
        uint32_t hold_count = count * pct / 100U;
        bool valid = counts >= 1.0L && counts <= 32.0L && hold_count != 0U;
        bits = 0xA5A5A5A5U;
        TMC2240Status status = tmc2240_calculateCurrent(request,resistor,range,gs,pct,hd,rd,&bits);
        if (valid) {
            CHECK(status == TMC2240_OK);
            CHECK(bits == ((hold_count - 1U) | ((count - 1U) << 8U) |
                          ((uint32_t)hd << 16U) | ((uint32_t)rd << 24U)));
        } else {
            CHECK(status == TMC2240_ERROR_RANGE && bits == 0xA5A5A5A5U);
        }
    }
}

static void test_helper_values(void)
{
    uint32_t word;
    uint16_t small;
    int16_t a,b;
    int32_t position;
    uint8_t byte;
    fresh();
    STATUS(tmc2240_stallguard_read(0U,NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_driver_status(0U,NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_check_faults(0U,NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_read_vsupply(0U,NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_get_vsupply_mV(0U,NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_read_temperature(0U,NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_get_temperature_c10(0U,NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_encoder_read(0U,NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_encoder_read_latch(0U,NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_encoder_get_status(0U,NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_get_microstep_counter(0U,NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_get_microstep_current(0U,NULL,NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_pwm_get_scale(0U,NULL,NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_stealthchop_enable(0U,2U), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_encoder_init(0U,INT32_MIN), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_clear_faults(0U,32U), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_set_tpwmthrs(0U,0x100000U), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_set_tcoolthrs(0U,UINT32_MAX), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_set_thigh(0U,UINT32_MAX), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_hal_setMicrosteps(0U,3U), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_hal_setMicrosteps(0U,257U), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_set_chopper(0U,16U,5U,2U,2U), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_set_chopper(0U,0U,8U,2U,2U), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_set_chopper(0U,0U,5U,16U,2U), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_set_chopper(0U,0U,5U,2U,4U), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_set_chopper(0U,1U,5U,2U,1U), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_coolstep_configure(0U,16U,0U,0U,0U,0), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_coolstep_configure(0U,0U,16U,0U,0U,0), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_coolstep_configure(0U,0U,0U,4U,0U,0), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_coolstep_configure(0U,0U,0U,0U,4U,0), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_coolstep_configure(0U,0U,0U,0U,0U,-65), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_coolstep_configure(0U,0U,0U,0U,0U,64), TMC2240_ERROR_RANGE);
    for (unsigned field = 0U; field < 7U; ++field) {
        uint8_t invalid[7] = {0U};
        invalid[field] = 2U;
        STATUS(tmc2240_diag_configure(0U,invalid[0],invalid[1],invalid[2],invalid[3],
               invalid[4],invalid[5],invalid[6]), TMC2240_ERROR_RANGE);
    }
    no_access();
    const uint16_t steps[] = {0U,1U,2U,4U,8U,16U,32U,64U,128U,256U};
    const uint8_t mres[] = {8U,8U,7U,6U,5U,4U,3U,2U,1U,0U};
    for (unsigned i = 0U; i < sizeof(steps) / sizeof(steps[0]); ++i) {
        hal_trace_reset();
        OK(tmc2240_hal_setMicrosteps(0U,steps[i]));
        CHECK(((model_devices[0].reg[0x6C] >> 24U) & 15U) == mres[i]);
        CHECK((model_devices[0].reg[0x6C] & 0xF0FFFFFFU) == 0x10410150U);
    }
    for (unsigned i = 0U; i < 128U; ++i) {
        hal_trace_reset();
        model_devices[0].reg[0] = 0x1C01EU;
        OK(tmc2240_diag_configure(0U,(uint8_t)(i&1U),(uint8_t)((i>>1U)&1U),
            (uint8_t)((i>>2U)&1U),(uint8_t)((i>>3U)&1U),(uint8_t)((i>>4U)&1U),
            (uint8_t)((i>>5U)&1U),(uint8_t)((i>>6U)&1U)));
        uint32_t expected = 0x1C01EU | ((i & 63U) << 5U) |
                            (((i >> 6U) & 1U) * 0x3000U);
        CHECK(model_devices[0].reg[0] == expected && hal_calls == 3U);
    }
    for (int sgt = -64; sgt <= 63; ++sgt) {
        hal_trace_reset();
        model_devices[0].reg[0x6D] = 0x01008000U;
        OK(tmc2240_coolstep_configure(0U,15U,15U,3U,3U,(int8_t)sgt));
        CHECK(model_devices[0].reg[0x6D] == (0x0100EF6FU | (((uint32_t)sgt & 127U) << 16U)));
    }
    for (unsigned threshold = 0U; threshold < 256U; ++threshold) {
        hal_trace_reset();
        model_devices[0].reg[0x74] = 0x100U;
        OK(tmc2240_stallguard_set_threshold(0U,(uint8_t)threshold));
        CHECK(model_devices[0].reg[0x74] == (0x100U | threshold));
    }
    hal_trace_reset();
    model_devices[0].reg[0x75] = 0xFFFF01FEU;
    OK(tmc2240_stallguard_read(0U, &small));
    CHECK(small == 510U);
    model_devices[0].reg[0x6F] = UINT32_MAX;
    OK(tmc2240_driver_status(0U, &word));
    CHECK(word == UINT32_MAX);
    OK(tmc2240_check_faults(0U, &byte));
    CHECK(byte == 0x1DU && model_devices[0].reg[1] == 0x1DU);
    uint32_t writes = model_devices[0].writes;
    OK(tmc2240_check_faults(0U, &byte));
    CHECK(writes == model_devices[0].writes);
    model_devices[0].persistent_faults = 2U;
    STATUS(tmc2240_clear_faults(0U,0x1FU), TMC2240_ERROR_FAULT);
    model_devices[0].persistent_faults = 0U;
    OK(tmc2240_clear_faults(0U,2U));
    OK(tmc2240_clear_faults(0U,0U));
    model_devices[0].reg[0x04] |= 0x8000U;
    small = 123U;
    STATUS(tmc2240_read_vsupply(0U,&small), TMC2240_ERROR_FAULT);
    STATUS(tmc2240_read_temperature(0U,&small), TMC2240_ERROR_FAULT);
    CHECK(small == 123U);
    model_devices[0].reg[0x04] &= ~0x8000U;
    for (unsigned adc = 0U; adc < 8192U; ++adc) {
        hal_trace_reset();
        model_devices[0].reg[0x50] = 0xFFFFE000U | adc;
        model_devices[0].reg[0x51] = 0xFFFFE000U | adc;
        OK(tmc2240_read_vsupply(0U,&small));
        CHECK(small == adc);
        OK(tmc2240_get_vsupply_mV(0U,&word));
        CHECK(word == adc * 9732U / 1000U);
        OK(tmc2240_get_temperature_c10(0U,&a));
        CHECK(a == ((int32_t)adc - 2038) * 100 / 77);
    }
    for (unsigned raw = 0U; raw < 512U; ++raw) {
        hal_trace_reset();
        model_devices[0].reg[0x6B] = (raw << 16U) | (511U - raw);
        model_devices[0].reg[0x71] = (raw << 16U) | 1023U;
        OK(tmc2240_get_microstep_current(0U,&a,&b));
        CHECK(a == (raw >= 256U ? (int32_t)raw - 512 : (int32_t)raw));
        CHECK(b == (511U - raw >= 256U ? (int32_t)(511U - raw) - 512 : (int32_t)(511U - raw)));
        OK(tmc2240_pwm_get_scale(0U,&small,&a));
        CHECK(small == 1023U && a == (raw >= 256U ? (int32_t)raw - 512 : (int32_t)raw));
    }
    hal_trace_reset();
    OK(tmc2240_pwm_get_scale(0U,&small,NULL));
    OK(tmc2240_pwm_get_scale(0U,NULL,&a));
    OK(tmc2240_get_microstep_current(0U,&a,NULL));
    OK(tmc2240_get_microstep_current(0U,NULL,&b));
    model_devices[0].reg[0x6A] = UINT32_MAX;
    OK(tmc2240_get_microstep_counter(0U,&small));
    CHECK(small == 1023U);
    const int32_t scales[] = {-INT32_MAX,-65536,-32768,0,32768,65536,INT32_MAX};
    for (unsigned i = 0U; i < sizeof(scales) / sizeof(scales[0]); ++i) {
        hal_trace_reset();
        model_devices[0].reg[0x39] = 0x12345678U;
        OK(tmc2240_encoder_init(0U,scales[i]));
        CHECK(hal_calls == 4U && model_devices[0].reg[0x39] == 0U);
        CHECK(model_devices[0].reg[0x3A] == (uint32_t)scales[i]);
        CHECK(model_devices[0].reg[0x38] == 0x1E8U);
        model_devices[0].reg[0x39] = UINT32_MAX;
        OK(tmc2240_encoder_read(0U,&position));
        CHECK(position == -1);
        model_n_event(0U);
        CHECK(model_devices[0].reg[0x39] == 0U);
        OK(tmc2240_encoder_read_latch(0U,&position));
        CHECK(position == -1);
        OK(tmc2240_encoder_get_status(0U,&byte));
        CHECK(byte == 1U);
        OK(tmc2240_encoder_clear_n_event(0U));
        OK(tmc2240_encoder_get_status(0U,&byte));
        CHECK(byte == 0U);
        model_devices[0].reg[0x39] = 9U;
        model_n_event(0U);
        CHECK(model_devices[0].reg[0x39] == 9U); /* One-shot latch/clear. */
    }
    hal_trace_reset();
    model_devices[0].reg[0x39] = 0x80000000U;
    OK(tmc2240_encoder_read(0U,&position));
    CHECK(position == INT32_MIN);
    model_devices[0].reg[0x3C] = INT32_MAX;
    OK(tmc2240_encoder_read_latch(0U,&position));
    CHECK(position == INT32_MAX);
    model_devices[0].reg[0x70] = 0xA40C1250U;
    model_devices[0].reg[0x6C] = 0x10410153U;
    model_devices[0].reg[0x6F] = 0U;
    STATUS(tmc2240_stealthchop_enable(0U,1U), TMC2240_ERROR_STATE);
    model_devices[0].reg[0x6F] = 0x80000000U;
    model_devices[0].reg[0x10] = 0x04011F08U;
    STATUS(tmc2240_stealthchop_enable(0U,1U), TMC2240_ERROR_STATE);
    model_devices[0].reg[0x10] = 0x04011F1FU;
    OK(tmc2240_stealthchop_enable(0U,1U));
    CHECK(model_devices[0].reg[0x70] == 0xA40C1250U);
    CHECK((model_devices[0].reg[0] & 4U) != 0U);
    OK(tmc2240_stealthchop_enable(0U,0U));
    CHECK((model_devices[0].reg[0] & 4U) == 0U);
    OK(tmc2240_set_tpwmthrs(0U,0U));
    OK(tmc2240_set_tcoolthrs(0U,0xFFFFFU));
    OK(tmc2240_set_thigh(0U,0xFFFFFU));
    OK(tmc2240_set_tpowerdown(0U,0U));
    OK(tmc2240_set_tpowerdown(0U,255U));
    const uint8_t versions[] = {0U,0x10U,0x3FU,0x40U,0x41U,0xFFU};
    for (unsigned i = 0U; i < sizeof(versions); ++i) {
        hal_trace_reset();
        model_devices[0].reg[4] = (uint32_t)versions[i] << 24U;
        STATUS(tmc2240_hal_testConnection(0U),
               versions[i] == 0x40U ? TMC2240_OK : TMC2240_ERROR_VERIFY);
    }
}

static void test_motor_gate(void)
{
    fresh();
    STATUS(tmc2240_hal_configureMotor(0U,NULL), TMC2240_ERROR_ARGUMENT);
    STATUS(tmc2240_hal_activateMotor(0U,0U), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_hal_activateMotor(0U,16U), TMC2240_ERROR_RANGE);
    STATUS(tmc2240_hal_activateMotor(0U,3U), TMC2240_ERROR_STATE);
    for (unsigned bit = 1U; bit <= 2U; ++bit) {
        for (unsigned phase = 1U; phase <= 17U; ++phase) {
            prepare();
            hal_status_at = phase;
            hal_status_bits = (uint8_t)bit;
            STATUS(tmc2240_hal_activateMotor(0U,3U), TMC2240_ERROR_STATE);
            CHECK((model_devices[0].reg[0x6C] & 15U) == 0U);
            if (phase < 15U) {
                CHECK(model_devices[0].activations == 0U);
            }
        }
        for (unsigned phase = 15U; phase <= 19U; ++phase) {
            prepare();
            hal_status_at = phase;
            hal_status_bits = (uint8_t)bit;
            STATUS(tmc2240_set_chopper(0U,3U,5U,2U,2U), TMC2240_ERROR_STATE);
            CHECK((model_devices[0].reg[0x6C] & 15U) == 0U);
            if (phase < 17U) {
                CHECK(model_devices[0].activations == 0U);
            }
        }
    }
    fresh();
    STATUS(tmc2240_set_chopper(0U,3U,5U,2U,2U), TMC2240_ERROR_STATE);
    TMC2240_MotorConfig_t bad = motor;
    bad.chopconf |= 3U;
    STATUS(tmc2240_hal_configureMotor(0U,&bad), TMC2240_ERROR_RANGE);
    bad = motor;
    bad.global_scaler = 1U;
    STATUS(tmc2240_hal_configureMotor(0U,&bad), TMC2240_ERROR_RANGE);
    bad = motor;
    bad.chopconf = (bad.chopconf & ~0x7F0U) | 0x7F0U;
    bad.ihold_irun |= 0x1F00U;
    STATUS(tmc2240_hal_configureMotor(0U,&bad), TMC2240_ERROR_RANGE);
    bad.ihold_irun = motor.ihold_irun | 31U;
    STATUS(tmc2240_hal_configureMotor(0U,&bad), TMC2240_ERROR_RANGE);
    no_access();
    model_devices[0].reg[4] = UINT32_MAX;
    STATUS(tmc2240_hal_configureMotor(0U,&motor), TMC2240_ERROR_VERIFY);
    CHECK(model_devices[0].writes == 0U);
    fresh();
    OK(tmc2240_hal_configureMotor(0U,&motor));
    STATUS(tmc2240_hal_activateMotor(0U,3U), TMC2240_ERROR_FAULT);
    CHECK(model_devices[0].activations == 0U);
    prepare();
    OK(tmc2240_hal_activateMotor(0U,3U));
    CHECK(model_devices[0].activations == 1U);
    CHECK(model_devices[0].reg[0x6C] == (motor.chopconf | 3U));
    OK(tmc2240_hal_disableMotor(0U));
    CHECK(model_devices[0].reg[0x6C] == motor.chopconf);
    OK(tmc2240_hal_activateMotor(0U,1U));
    OK(tmc2240_hal_disableMotor(0U));
    OK(tmc2240_set_chopper(0U,1U,5U,2U,2U));
    OK(tmc2240_hal_disableMotor(0U));
    OK(tmc2240_hal_activateMotor(0U,15U));
    OK(tmc2240_hal_disableMotor(0U));
    OK(tmc2240_set_chopper(0U,0U,7U,15U,3U));
    CHECK(model_devices[0].reg[0x6C] == 0x144187F0U);
    hal_check_trace();
    static const uint8_t guarded[] = {0x6CU,0x0AU,0x0BU,0x10U,0x70U,0U};
    for (unsigned i = 0U; i < sizeof(guarded); ++i) {
        prepare();
        model_devices[0].reg[guarded[i]] ^= (guarded[i] == 0U ? 2U : 1U);
        STATUS(tmc2240_hal_activateMotor(0U,3U), TMC2240_ERROR_VERIFY);
        CHECK(model_devices[0].activations == 0U);
        hal_trace_reset();
        STATUS(tmc2240_hal_activateMotor(0U,3U), TMC2240_ERROR_STATE);
        no_access();
    }
    prepare();
    model_devices[0].reg[1] = 2U;
    STATUS(tmc2240_hal_activateMotor(0U,3U), TMC2240_ERROR_FAULT);
    prepare();
    model_devices[0].status_pipeline = 1U;
    STATUS(tmc2240_hal_activateMotor(0U,3U), TMC2240_ERROR_STATE);
    prepare();
    model_devices[0].reg[1] = 8U;
    uint8_t flags;
    OK(tmc2240_check_faults(0U,&flags));
    STATUS(tmc2240_hal_activateMotor(0U,3U), TMC2240_ERROR_STATE);
    prepare();
    hal_drop_write_at = 15U; /* GSTAT + six readbacks = 14 frames, then activation. */
    STATUS(tmc2240_hal_activateMotor(0U,3U), TMC2240_ERROR_VERIFY);
    CHECK((model_devices[0].reg[0x6C] & 15U) == 0U);
    prepare();
    hal_fail_at = 15U;
    hal_fail_after_accept = true;
    hal_fail_also = 16U; /* Cleanup also fails: the original TIMEOUT remains. */
    STATUS(tmc2240_hal_activateMotor(0U,3U), TMC2240_ERROR_TIMEOUT);
    hal_trace_reset();
    STATUS(tmc2240_hal_activateMotor(0U,3U), TMC2240_ERROR_STATE);
    OK(tmc2240_hal_disableMotor(0U));
    prepare();
    hal_fail_at = 17U;
    STATUS(tmc2240_hal_activateMotor(0U,3U), TMC2240_ERROR_TIMEOUT);
    CHECK(tmc2240_hal_spiStatus(0U) == HAL_OK); /* Cleanup success is not the operation result. */
    CHECK((model_devices[0].reg[0x6C] & 15U) == 0U);
    fresh();
    bad = motor;
    bad.chopconf &= ~0x18000U;
    OK(tmc2240_clear_faults(0U,0x1FU));
    OK(tmc2240_hal_configureMotor(0U,&bad));
    hal_trace_reset();
    STATUS(tmc2240_hal_activateMotor(0U,1U), TMC2240_ERROR_RANGE);
    no_access();
    prepare();
    bad = motor;
    bad.ihold_irun |= 0x1F00U;
    OK(tmc2240_hal_configureMotor(0U,&bad));
    STATUS(tmc2240_set_chopper(0U,3U,7U,12U,2U), TMC2240_ERROR_RANGE);
    CHECK(model_devices[0].activations == 0U);
    OK(tmc2240_set_chopper(0U,0U,7U,12U,2U));
    hal_trace_reset();
    STATUS(tmc2240_hal_activateMotor(0U,3U), TMC2240_ERROR_RANGE);
    no_access();
    OK(tmc2240_set_chopper(0U,0U,7U,11U,2U));
    OK(tmc2240_hal_activateMotor(0U,3U));
    OK(tmc2240_hal_disableMotor(0U));
    bad.chopconf |= 0x47F0U; /* CHM=1: HSTRT/HEND are TFD/offset, not hysteresis. */
    OK(tmc2240_hal_configureMotor(0U,&bad));
    OK(tmc2240_hal_activateMotor(0U,3U));
#if TMC2240_CACHE && !TMC2240_ENABLE_TMC_CACHE
    fresh();
    custom_cache_fail_op = TMC2240_CACHE_CLEAR;
    STATUS(tmc2240_hal_deinit(), TMC2240_ERROR_IO);
    CHECK(!tmc2240_hal_isInitialized());
    custom_cache_reset_faults();
#endif
}

static void test_recovery(void)
{
    fresh();
    uint32_t expected[4] = {0U};
    for (unsigned i = 0U; i < TEST_ICS; ++i) {
        model_devices[i].reg[1] = 0U;
        model_devices[i].status_pipeline = 8U;
    }
    for (unsigned i = 0U; i < 10000U; ++i) {
        uint16_t id = (uint16_t)(i % TEST_ICS);
        uint32_t value = test_random();
        uint32_t out = 0x1234U;
        hal_trace_reset();
        hal_fail_at = 1U + i % 2U;
        hal_fail_after_accept = (i & 1U) != 0U;
        STATUS(tmc2240_readRegister(id, 0x39U, &out), TMC2240_ERROR_TIMEOUT);
        CHECK(out == 0x1234U);
        hal_check_trace();
        hal_trace_reset();
        OK(tmc2240_writeRegisterVerified(id, 0x39U, value));
        expected[id] = value;
        CHECK(model_devices[id].reg[0x39] == value);
#if TMC2240_CACHE
        TMC2240CacheEntry entry;
        OK(tmc2240_getCachedRegister(id,0x39U,&entry));
        CHECK(entry.confirmed_valid && !entry.dirty && entry.confirmed == value);
#endif
        for (unsigned other = 0U; other < TEST_ICS; ++other) {
            CHECK(model_devices[other].activations == 0U);
            CHECK(model_devices[other].reg[0x39] == expected[other]);
        }
        hal_check_trace();
    }
}

int main(void)
{
    test_init_validation();
    test_spi();
    test_operation_failure_matrix();
    test_current();
    test_helper_values();
    test_motor_gate();
    test_recovery();
    OK(tmc2240_hal_deinit());
    test_summary("TMC2240 HAL", 100000U, 10000U);
    return EXIT_SUCCESS;
}
