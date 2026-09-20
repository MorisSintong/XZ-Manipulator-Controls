/*******************************************************************************
 * Copyright © 2024 STM32 HAL Integration for TMC2240
 *******************************************************************************/
#include "tmc2240_hal.h"
#include <limits.h>
#include <string.h>

static TMC2240_SPIConfig_t spiConfigs[TMC2240_HAL_MAX_ICS];
static HAL_StatusTypeDef lastHalStatus[TMC2240_HAL_MAX_ICS];
static uint8_t lastSpiByte[TMC2240_HAL_MAX_ICS];
static bool spiByteValid[TMC2240_HAL_MAX_ICS];
static bool motorPrepared[TMC2240_HAL_MAX_ICS];
static uint32_t preparedValues[TMC2240_HAL_MAX_ICS][6];
static uint8_t icCount;
static const uint8_t motorRegisters[6] = {
    TMC2240_CHOPCONF, TMC2240_DRV_CONF, TMC2240_GLOBAL_SCALER,
    TMC2240_IHOLD_IRUN, TMC2240_PWMCONF, TMC2240_GCONF
};

static TMC2240Status validID(uint16_t icID)
{
    if (icCount == 0U) {
        return TMC2240_ERROR_NOT_INITIALIZED;
    }
    return (icID < icCount) ? TMC2240_OK : TMC2240_ERROR_ID;
}

static uint32_t peripheralClock(const SPI_HandleTypeDef *spi)
{
    bool apb2 = (spi->Instance == SPI1);
#ifdef SPI4
    apb2 = apb2 || (spi->Instance == SPI4);
#endif
#ifdef SPI5
    apb2 = apb2 || (spi->Instance == SPI5);
#endif
#ifdef SPI6
    apb2 = apb2 || (spi->Instance == SPI6);
#endif
    return apb2 ? HAL_RCC_GetPCLK2Freq() : HAL_RCC_GetPCLK1Freq();
}

static TMC2240Status validConfig(const TMC2240_SPIConfig_t *config)
{
    uint32_t divisor;
    uint32_t pclk;
    const SPI_HandleTypeDef *spi = config->hspi;
    if ((spi == NULL) || (config->cs_port == NULL) ||
        !IS_GPIO_ALL_INSTANCE(config->cs_port) ||
        !IS_SPI_ALL_INSTANCE(spi->Instance)) {
        return TMC2240_ERROR_ARGUMENT;
    }
    if ((config->cs_pin == 0U) || ((config->cs_pin & (config->cs_pin - 1U)) != 0U)) {
        return TMC2240_ERROR_ARGUMENT;
    }
    if ((config->spi_timeout_ms == 0U) || (config->spi_timeout_ms > INT32_MAX)) {
        return TMC2240_ERROR_RANGE;
    }
    if ((spi->Init.Mode != SPI_MODE_MASTER) ||
        (spi->Init.Direction != SPI_DIRECTION_2LINES) ||
        (spi->Init.DataSize != SPI_DATASIZE_8BIT) ||
        (spi->Init.CLKPolarity != SPI_POLARITY_HIGH) ||
        (spi->Init.CLKPhase != SPI_PHASE_2EDGE) ||
        (spi->Init.NSS != SPI_NSS_SOFT) ||
        (spi->Init.FirstBit != SPI_FIRSTBIT_MSB) ||
        (spi->Init.TIMode != SPI_TIMODE_DISABLE) ||
        (spi->Init.CRCCalculation != SPI_CRCCALCULATION_DISABLE) ||
        !IS_SPI_BAUDRATE_PRESCALER(spi->Init.BaudRatePrescaler)) {
        return TMC2240_ERROR_ARGUMENT;
    }
    if (spi->State != HAL_SPI_STATE_READY) {
        return TMC2240_ERROR_BUSY;
    }
    divisor = 2U << (spi->Init.BaudRatePrescaler >> 3U);
    pclk = peripheralClock(spi);
    if ((SystemCoreClock < 1000000U) || (SystemCoreClock > 1000000000U) ||
        (pclk < 1000000U) || (pclk > divisor * 10000000U)) {
        return TMC2240_ERROR_RANGE;
    }
    return TMC2240_OK;
}

/* At least one SCK period for tCC, and >=1us for 4*tCLK (fCLK>=8MHz).
 * NOPs plus volatile loop overhead can only lengthen this interval. Accurate
 * SystemCoreClock/RCC configuration is a caller prerequisite, not a measurement.
 */
static void spiGuard(const TMC2240_SPIConfig_t *config)
{
    uint32_t divisor = 2U << (config->hspi->Init.BaudRatePrescaler >> 3U);
    uint32_t sck = peripheralClock(config->hspi) / divisor;
    uint32_t cycles = SystemCoreClock / sck + 1U;
    uint32_t oneUs = SystemCoreClock / 1000000U + 1U;
    if (cycles < oneUs) {
        cycles = oneUs;
    }
    for (volatile uint32_t i = 0U; i < cycles; ++i) {
        __NOP();
    }
}

TMC2240Status tmc2240_hal_init(const TMC2240_SPIConfig_t *configs, uint8_t count)
{
    TMC2240Status status;
    if ((configs == NULL) || (count == 0U) || (count > TMC2240_HAL_MAX_ICS)) {
        return TMC2240_ERROR_ARGUMENT;
    }
#if TMC2240_CACHE && TMC2240_ENABLE_TMC_CACHE
    if (count > TMC2240_IC_CACHE_COUNT) {
        return TMC2240_ERROR_RANGE;
    }
#endif
    if (icCount != 0U) {
        return TMC2240_ERROR_STATE;
    }
    for (uint8_t i = 0U; i < count; ++i) {
        status = validConfig(&configs[i]);
        if (status != TMC2240_OK) {
            return status;
        }
        for (uint8_t j = 0U; j < i; ++j) {
            if (((configs[i].cs_port == configs[j].cs_port) &&
                 (configs[i].cs_pin == configs[j].cs_pin)) ||
                ((configs[i].hspi->Instance == configs[j].hspi->Instance) &&
                 (configs[i].hspi != configs[j].hspi))) {
                return TMC2240_ERROR_ARGUMENT;
            }
        }
    }
    status = tmc2240_initCache(count);
    if (status != TMC2240_OK) {
        return status;
    }
    memset(motorPrepared, 0, sizeof(motorPrepared));
    memset(spiByteValid, 0, sizeof(spiByteValid));
    memset(preparedValues, 0, sizeof(preparedValues));
    for (uint8_t i = 0U; i < count; ++i) {
        spiConfigs[i] = configs[i];
        lastHalStatus[i] = HAL_ERROR; /* No SPI frame has completed yet. */
        HAL_GPIO_WritePin(configs[i].cs_port, configs[i].cs_pin, GPIO_PIN_SET);
        spiGuard(&configs[i]);
    }
    icCount = count;
    return TMC2240_OK;
}

TMC2240Status tmc2240_hal_deinit(void)
{
    TMC2240Status status = TMC2240_OK;
    uint8_t previousCount = icCount;
    icCount = 0U;
    for (uint8_t i = 0U; i < previousCount; ++i) {
        HAL_GPIO_WritePin(spiConfigs[i].cs_port, spiConfigs[i].cs_pin, GPIO_PIN_SET);
    }
    if (previousCount != 0U) {
        status = tmc2240_initCache(previousCount);
    }
    memset(spiConfigs, 0, sizeof(spiConfigs));
    memset(motorPrepared, 0, sizeof(motorPrepared));
    memset(spiByteValid, 0, sizeof(spiByteValid));
    return status;
}

bool tmc2240_hal_isInitialized(void)
{
    return icCount != 0U;
}

TMC2240Status tmc2240_readWriteSPI(uint16_t icID, uint8_t *data, size_t dataLength)
{
    uint8_t tx[TMC2240_SPI_FRAME_SIZE];
    uint8_t rx[TMC2240_SPI_FRAME_SIZE] = { 0U };
    TMC2240Status status = validID(icID);
    HAL_StatusTypeDef halStatus;
    if (status != TMC2240_OK) {
        return status;
    }
    if ((data == NULL) || (dataLength != TMC2240_SPI_FRAME_SIZE)) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = validConfig(&spiConfigs[icID]);
    if (status != TMC2240_OK) {
        return status;
    }
    memcpy(tx, data, sizeof(tx));
    spiGuard(&spiConfigs[icID]);
    HAL_GPIO_WritePin(spiConfigs[icID].cs_port, spiConfigs[icID].cs_pin, GPIO_PIN_RESET);
    spiGuard(&spiConfigs[icID]);
    halStatus = HAL_SPI_TransmitReceive(spiConfigs[icID].hspi, tx, rx, sizeof(tx),
                                        spiConfigs[icID].spi_timeout_ms);
    spiGuard(&spiConfigs[icID]);
    HAL_GPIO_WritePin(spiConfigs[icID].cs_port, spiConfigs[icID].cs_pin, GPIO_PIN_SET);
    spiGuard(&spiConfigs[icID]);
    lastHalStatus[icID] = halStatus;
    spiByteValid[icID] = false;
    switch (halStatus) {
    case HAL_OK:
        memcpy(data, rx, sizeof(rx));
        lastSpiByte[icID] = rx[0];
        spiByteValid[icID] = true;
        if ((rx[0] & 3U) != 0U) {
            motorPrepared[icID] = false;
        }
        return TMC2240_OK;
    case HAL_TIMEOUT:
        motorPrepared[icID] = false;
        return TMC2240_ERROR_TIMEOUT;
    case HAL_BUSY:
        motorPrepared[icID] = false;
        return TMC2240_ERROR_BUSY;
    default:
        motorPrepared[icID] = false;
        return TMC2240_ERROR_IO;
    }
}

TMC2240Status tmc2240_readWriteUART(uint16_t icID, uint8_t *data,
                                   size_t writeLength, size_t readLength)
{
    (void)icID;
    (void)data;
    (void)writeLength;
    (void)readLength;
    return TMC2240_ERROR_UNSUPPORTED;
}

TMC2240Status tmc2240_getBusType(uint16_t icID, TMC2240BusType *bus)
{
    TMC2240Status status;
    if (bus == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = validID(icID);
    if (status == TMC2240_OK) {
        *bus = IC_BUS_SPI;
    }
    return status;
}

TMC2240Status tmc2240_getNodeAddress(uint16_t icID, uint8_t *node)
{
    (void)icID;
    (void)node;
    return TMC2240_ERROR_UNSUPPORTED;
}

HAL_StatusTypeDef tmc2240_hal_spiStatus(uint16_t icID)
{
    return (validID(icID) == TMC2240_OK) ? lastHalStatus[icID] : HAL_ERROR;
}

TMC2240Status tmc2240_hal_getSPIStatus(uint16_t icID, uint8_t *statusByte)
{
    TMC2240Status status;
    if (statusByte == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = validID(icID);
    if (status != TMC2240_OK) {
        return status;
    }
    if (!spiByteValid[icID]) {
        return TMC2240_ERROR_STATE;
    }
    *statusByte = lastSpiByte[icID];
    return TMC2240_OK;
}

TMC2240Status tmc2240_hal_testConnection(uint16_t icID)
{
    uint32_t ioin;
    TMC2240Status status = tmc2240_readRegister(icID, TMC2240_IOIN, &ioin);
    if (status != TMC2240_OK) {
        return status;
    }
    return ((ioin >> 24U) == 0x40U) ? TMC2240_OK : TMC2240_ERROR_VERIFY;
}

static void disableAfterFailure(uint16_t icID)
{
    motorPrepared[icID] = false;
    (void)tmc2240_writeRegister(icID, TMC2240_CHOPCONF, 0U);
}

static TMC2240Status validMotorChopper(uint32_t chop, uint32_t current)
{
    bool fullScale = ((current & 31U) == 31U) || (((current >> 8U) & 31U) == 31U);
    uint32_t encodedSum = ((chop >> 4U) & 7U) + ((chop >> 7U) & 15U);
    /* Rev.2 pp.44,112: effective HSTRT+HEND <=16 at CS=31.
     * Encoded HSTRT adds 1 and encoded HEND subtracts 3. At CS<=30
     * the sum is unrestricted; CHM=1 uses these bits for different purposes.
     */
    if (((chop & TMC2240_CHM_MASK) == 0U) && fullScale && (encodedSum > 18U)) {
        return TMC2240_ERROR_RANGE;
    }
    return TMC2240_OK;
}

TMC2240Status tmc2240_hal_configureMotor(uint16_t icID,
                                        const TMC2240_MotorConfig_t *config)
{
    TMC2240Status status;
    uint32_t oldChop;
    uint32_t values[6];
    if (config == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    values[0] = config->chopconf;
    values[1] = config->drv_conf;
    values[2] = config->global_scaler;
    values[3] = config->ihold_irun;
    values[4] = config->pwmconf;
    values[5] = config->gconf;
    if ((config->chopconf & TMC2240_TOFF_MASK) != 0U) {
        return TMC2240_ERROR_RANGE;
    }
    for (size_t i = 0U; i < 6U; ++i) {
        status = tmc2240_validateRegisterValue(motorRegisters[i], values[i]);
        if (status != TMC2240_OK) {
            return status;
        }
    }
    status = validMotorChopper(config->chopconf, config->ihold_irun);
    if (status != TMC2240_OK) {
        return status;
    }
    status = validID(icID);
    if (status != TMC2240_OK) {
        return status;
    }
    motorPrepared[icID] = false;
    status = tmc2240_hal_testConnection(icID);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_readRegister(icID, TMC2240_CHOPCONF, &oldChop);
    if (status == TMC2240_OK) {
        status = tmc2240_writeRegisterVerified(icID, TMC2240_CHOPCONF,
                                               oldChop & 0xFFFDDFF0U);
    }
    for (size_t i = 0U; (i < 6U) && (status == TMC2240_OK); ++i) {
        status = tmc2240_writeRegisterVerified(icID, motorRegisters[i], values[i]);
    }
    if (status != TMC2240_OK) {
        disableAfterFailure(icID);
        return status;
    }
    memcpy(preparedValues[icID], values, sizeof(values));
    motorPrepared[icID] = true;
    return TMC2240_OK;
}

static TMC2240Status activationCheck(uint16_t icID)
{
    uint8_t faults;
    TMC2240Status status;
    if (!motorPrepared[icID]) {
        return TMC2240_ERROR_STATE;
    }
    status = tmc2240_check_faults(icID, &faults);
    if (status != TMC2240_OK) {
        return status;
    }
    if (faults != 0U) {
        motorPrepared[icID] = false;
        return TMC2240_ERROR_FAULT;
    }
    for (size_t i = 0U; i < 6U; ++i) {
        TMC2240RegisterInfo info;
        uint32_t actual;
        status = tmc2240_readRegister(icID, motorRegisters[i], &actual);
        if (status != TMC2240_OK) {
            return status;
        }
        (void)tmc2240_getRegisterInfo(motorRegisters[i], &info);
        if ((actual & info.write_mask) != preparedValues[icID][i]) {
            motorPrepared[icID] = false;
            return TMC2240_ERROR_VERIFY;
        }
    }
    /* A reset/driver error in any SPI status invalidates the snapshot. */
    return motorPrepared[icID] ? TMC2240_OK : TMC2240_ERROR_STATE;
}

TMC2240Status tmc2240_hal_activateMotor(uint16_t icID, uint8_t toff)
{
    uint32_t chop;
    TMC2240Status status;
    if ((toff == 0U) || (toff > 15U)) {
        return TMC2240_ERROR_RANGE;
    }
    status = validID(icID);
    if (status != TMC2240_OK) {
        return status;
    }
    chop = preparedValues[icID][0] | toff;
    status = tmc2240_validateRegisterValue(TMC2240_CHOPCONF, chop);
    if (status != TMC2240_OK) {
        return status;
    }
    status = validMotorChopper(chop, preparedValues[icID][3]);
    if (status != TMC2240_OK) {
        return status;
    }
    status = activationCheck(icID);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_writeRegisterVerified(icID, TMC2240_CHOPCONF, chop);
    if ((status == TMC2240_OK) && !motorPrepared[icID]) {
        status = TMC2240_ERROR_STATE;
    }
    if (status != TMC2240_OK) {
        disableAfterFailure(icID);
    }
    return status;
}

TMC2240Status tmc2240_hal_disableMotor(uint16_t icID)
{
    uint32_t chop;
    TMC2240Status status = validID(icID);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_readRegister(icID, TMC2240_CHOPCONF, &chop);
    if (status == TMC2240_OK) {
        status = tmc2240_writeRegisterVerified(icID, TMC2240_CHOPCONF,
                                               chop & 0xFFFDDFF0U);
    }
    if (status != TMC2240_OK) {
        disableAfterFailure(icID);
    }
    return status;
}

TMC2240Status tmc2240_calculateCurrent(uint16_t runCurrent_mA, uint16_t rRef_Ohm,
    uint8_t currentRange, uint8_t globalScaler, uint8_t holdPercent,
    uint8_t holdDelay, uint8_t runDelay, uint32_t *ihold_irun)
{
    static const uint32_t kifs[] = { 11750U, 24000U, 36000U, 36000U };
    uint32_t gs = (globalScaler == 0U) ? 256U : globalScaler;
    uint64_t numerator;
    uint64_t denominator;
    uint32_t runCount;
    uint32_t holdCount;
    if (ihold_irun == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    if ((rRef_Ohm < 12000U) || (rRef_Ohm > 60000U) ||
        (currentRange > 3U) || (gs < 32U) ||
        (runCurrent_mA == 0U) || (runCurrent_mA > 3000U) ||
        (holdPercent == 0U) || (holdPercent > 100U) ||
        (holdDelay > 15U) || (runDelay > 15U)) {
        return TMC2240_ERROR_RANGE;
    }
    /* 32*sqrt(2)*1e6 rounded down; worst numerator < 2.086e18.
     * The early 3A bound is above every possible RMS full-scale setting.
     */
    numerator = (uint64_t)runCurrent_mA * 256U * rRef_Ohm * 45254833U;
    denominator = (uint64_t)gs * kifs[currentRange] * 1000000000U;
    if ((numerator < denominator) || (numerator > denominator * 32U)) {
        return TMC2240_ERROR_RANGE;
    }
    runCount = (uint32_t)(numerator / denominator);
    holdCount = (runCount * holdPercent) / 100U;
    if (holdCount == 0U) {
        return TMC2240_ERROR_RANGE;
    }
    *ihold_irun = (holdCount - 1U) | ((runCount - 1U) << 8U) |
                 ((uint32_t)holdDelay << 16U) | ((uint32_t)runDelay << 24U);
    return TMC2240_OK;
}

TMC2240Status tmc2240_hal_setCurrent(uint16_t icID, uint16_t runCurrent_mA,
    uint16_t rRef_Ohm, uint8_t holdPercent, uint8_t holdDelay)
{
    uint32_t drv;
    uint32_t scaler;
    uint32_t current;
    uint32_t value;
    TMC2240Status status;
    if ((runCurrent_mA == 0U) || (runCurrent_mA > 3000U) ||
        (rRef_Ohm < 12000U) || (rRef_Ohm > 60000U) ||
        (holdPercent == 0U) || (holdPercent > 100U) || (holdDelay > 15U)) {
        return TMC2240_ERROR_RANGE;
    }
    status = tmc2240_readRegister(icID, TMC2240_DRV_CONF, &drv);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_readRegister(icID, TMC2240_GLOBAL_SCALER, &scaler);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_readRegister(icID, TMC2240_IHOLD_IRUN, &current);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_calculateCurrent(runCurrent_mA, rRef_Ohm, (uint8_t)(drv & 3U),
        (uint8_t)scaler, holdPercent, holdDelay, (uint8_t)((current >> 24U) & 15U),
        &value);
    return (status == TMC2240_OK)
        ? tmc2240_writeRegisterVerified(icID, TMC2240_IHOLD_IRUN, value) : status;
}

TMC2240Status tmc2240_hal_setMicrosteps(uint16_t icID, uint16_t usteps)
{
    uint8_t mres = 0U;
    if (usteps == 0U) {
        usteps = 1U;
    }
    if ((usteps > 256U) || ((usteps & (usteps - 1U)) != 0U)) {
        return TMC2240_ERROR_RANGE;
    }
    for (uint16_t count = 256U; count > usteps; count >>= 1U) {
        ++mres;
    }
    return tmc2240_fieldWrite(icID, TMC2240_MRES_FIELD, mres);
}

TMC2240Status tmc2240_stealthchop_enable(uint16_t icID, uint8_t enable)
{
    uint32_t chop;
    uint32_t driver;
    uint32_t current;
    TMC2240Status status;
    if (enable > 1U) {
        return TMC2240_ERROR_RANGE;
    }
    if (enable != 0U) {
        status = tmc2240_readRegister(icID, TMC2240_CHOPCONF, &chop);
        if (status != TMC2240_OK) {
            return status;
        }
        if ((chop & TMC2240_TOFF_MASK) != 0U) {
            status = tmc2240_driver_status(icID, &driver);
            if (status != TMC2240_OK) {
                return status;
            }
            if ((driver & TMC2240_STST_MASK) == 0U) {
                return TMC2240_ERROR_STATE;
            }
            status = tmc2240_readRegister(icID, TMC2240_IHOLD_IRUN, &current);
            if (status != TMC2240_OK) {
                return status;
            }
            if ((current & 31U) != ((current >> 8U) & 31U)) {
                return TMC2240_ERROR_STATE;
            }
        }
    }
    return tmc2240_fieldWrite(icID, TMC2240_EN_PWM_MODE_FIELD, enable);
}

TMC2240Status tmc2240_stallguard_set_threshold(uint16_t icID, uint8_t threshold)
{
    return tmc2240_fieldWrite(icID, TMC2240_SG4_THRS_FIELD, threshold);
}

static TMC2240Status readUnsigned16(uint16_t icID, RegisterField field, uint16_t *out)
{
    int64_t value;
    TMC2240Status status;
    if (out == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = tmc2240_fieldRead(icID, field, &value);
    if (status == TMC2240_OK) {
        *out = (uint16_t)value;
    }
    return status;
}

TMC2240Status tmc2240_stallguard_read(uint16_t icID, uint16_t *result)
{
    return readUnsigned16(icID, TMC2240_SG4_RESULT_FIELD, result);
}

TMC2240Status tmc2240_coolstep_configure(uint16_t icID, uint8_t semin, uint8_t semax,
    uint8_t seup, uint8_t sedn, int8_t stall_thr)
{
    uint32_t bits;
    if ((semin > 15U) || (semax > 15U) || (seup > 3U) || (sedn > 3U) ||
        (stall_thr < -64) || (stall_thr > 63)) {
        return TMC2240_ERROR_RANGE;
    }
    bits = semin | ((uint32_t)seup << 5U) | ((uint32_t)semax << 8U) |
           ((uint32_t)sedn << 13U) | (((uint32_t)stall_thr & 127U) << 16U);
    return tmc2240_updateRegister(icID, TMC2240_COOLCONF, 0x007F6F6FU, bits);
}

TMC2240Status tmc2240_driver_status(uint16_t icID, uint32_t *status)
{
    return tmc2240_readRegister(icID, TMC2240_DRVSTATUS, status);
}

TMC2240Status tmc2240_check_faults(uint16_t icID, uint8_t *faults)
{
    uint32_t gstat;
    TMC2240Status status;
    if (faults == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = tmc2240_readRegister(icID, TMC2240_GSTAT, &gstat);
    if (status == TMC2240_OK) {
        *faults = (uint8_t)(gstat & 0x1FU);
        if ((gstat & 9U) != 0U) {
            motorPrepared[icID] = false;
        }
    }
    return status;
}

TMC2240Status tmc2240_clear_faults(uint16_t icID, uint8_t mask)
{
    uint8_t faults;
    TMC2240Status status;
    if ((mask & ~0x1FU) != 0U) {
        return TMC2240_ERROR_RANGE;
    }
    status = tmc2240_writeRegister(icID, TMC2240_GSTAT, mask);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_check_faults(icID, &faults);
    if (status != TMC2240_OK) {
        return status;
    }
    return ((faults & mask) == 0U) ? TMC2240_OK : TMC2240_ERROR_FAULT;
}

static TMC2240Status readADC(uint16_t icID, RegisterField field, uint16_t *adc)
{
    uint32_t ioin;
    TMC2240Status status;
    if (adc == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = tmc2240_readRegister(icID, TMC2240_IOIN, &ioin);
    if (status != TMC2240_OK) {
        return status;
    }
    if ((ioin & 0x8000U) != 0U) {
        return TMC2240_ERROR_FAULT;
    }
    return readUnsigned16(icID, field, adc);
}

TMC2240Status tmc2240_read_vsupply(uint16_t icID, uint16_t *adc)
{
    return readADC(icID, TMC2240_ADC_VSUPPLY_FIELD, adc);
}

TMC2240Status tmc2240_get_vsupply_mV(uint16_t icID, uint32_t *millivolts)
{
    uint16_t adc;
    TMC2240Status status;
    if (millivolts == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = tmc2240_read_vsupply(icID, &adc);
    if (status == TMC2240_OK) {
        *millivolts = ((uint32_t)adc * 9732U) / 1000U;
    }
    return status;
}

TMC2240Status tmc2240_read_temperature(uint16_t icID, uint16_t *adc)
{
    return readADC(icID, TMC2240_ADC_TEMP_FIELD, adc);
}

TMC2240Status tmc2240_get_temperature_c10(uint16_t icID, int16_t *temperature)
{
    uint16_t adc;
    TMC2240Status status;
    if (temperature == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = tmc2240_read_temperature(icID, &adc);
    if (status == TMC2240_OK) {
        *temperature = (int16_t)(((int32_t)adc - 2038) * 100 / 77);
    }
    return status;
}

TMC2240Status tmc2240_encoder_init(uint16_t icID, int32_t enc_constant)
{
    TMC2240Status status;
    if (enc_constant == INT32_MIN) {
        return TMC2240_ERROR_RANGE;
    }
    status = tmc2240_writeRegister(icID, TMC2240_ENCMODE, 0U);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_writeRegister(icID, TMC2240_ENC_CONST, (uint32_t)enc_constant);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_writeRegister(icID, TMC2240_XENC, 0U);
    if (status != TMC2240_OK) {
        return status;
    }
    return tmc2240_writeRegister(icID, TMC2240_ENCMODE,
        TMC2240_EM_CLR_XENC | TMC2240_EM_CLR_ONCE | TMC2240_EM_POS_EDGE |
        TMC2240_EM_NEG_EDGE | TMC2240_EM_IGNORE_AB);
}

static TMC2240Status readSigned32(uint16_t icID, uint8_t address, int32_t *position)
{
    uint32_t value;
    TMC2240Status status;
    if (position == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = tmc2240_readRegister(icID, address, &value);
    if (status == TMC2240_OK) {
        int64_t signedValue = (int64_t)value;
        if (value > INT32_MAX) {
            signedValue -= INT64_C(4294967296);
        }
        *position = (int32_t)signedValue;
    }
    return status;
}

TMC2240Status tmc2240_encoder_read(uint16_t icID, int32_t *position)
{
    return readSigned32(icID, TMC2240_XENC, position);
}

TMC2240Status tmc2240_encoder_read_latch(uint16_t icID, int32_t *position)
{
    return readSigned32(icID, TMC2240_ENC_LATCH, position);
}

TMC2240Status tmc2240_encoder_get_status(uint16_t icID, uint8_t *n_event)
{
    uint32_t value;
    TMC2240Status status;
    if (n_event == NULL) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = tmc2240_readRegister(icID, TMC2240_ENC_STATUS, &value);
    if (status == TMC2240_OK) {
        *n_event = (uint8_t)(value & 1U);
    }
    return status;
}

TMC2240Status tmc2240_encoder_clear_n_event(uint16_t icID)
{
    return tmc2240_writeRegister(icID, TMC2240_ENC_STATUS, 1U);
}

TMC2240Status tmc2240_set_tpwmthrs(uint16_t icID, uint32_t threshold)
{
    return tmc2240_writeRegister(icID, TMC2240_TPWMTHRS, threshold);
}

TMC2240Status tmc2240_set_tcoolthrs(uint16_t icID, uint32_t threshold)
{
    return tmc2240_writeRegister(icID, TMC2240_TCOOLTHRS, threshold);
}

TMC2240Status tmc2240_set_thigh(uint16_t icID, uint32_t threshold)
{
    return tmc2240_writeRegister(icID, TMC2240_THIGH, threshold);
}

TMC2240Status tmc2240_set_chopper(uint16_t icID, uint8_t toff,
                                uint8_t hstrt, uint8_t hend, uint8_t tbl)
{
    uint32_t chop;
    TMC2240Status status;
    if ((toff > 15U) || (hstrt > 7U) || (hend > 15U) || (tbl > 3U) ||
        ((toff == 1U) && (tbl < 2U))) {
        return TMC2240_ERROR_RANGE;
    }
    status = validID(icID);
    if (status != TMC2240_OK) {
        return status;
    }
    if (toff != 0U) {
        status = activationCheck(icID);
        if (status != TMC2240_OK) {
            return status;
        }
    }
    status = tmc2240_readRegister(icID, TMC2240_CHOPCONF, &chop);
    if (status != TMC2240_OK) {
        return status;
    }
    if ((toff != 0U) && !motorPrepared[icID]) {
        return TMC2240_ERROR_STATE;
    }
    chop = (chop & 0xFFFC5800U) | toff | ((uint32_t)hstrt << 4U) |
           ((uint32_t)hend << 7U) | ((uint32_t)tbl << 15U);
    if (toff != 0U) {
        status = validMotorChopper(chop, preparedValues[icID][3]);
        if (status != TMC2240_OK) {
            return status;
        }
    }
    status = tmc2240_writeRegisterVerified(icID, TMC2240_CHOPCONF, chop);
    if ((status == TMC2240_OK) && (toff != 0U) && !motorPrepared[icID]) {
        status = TMC2240_ERROR_STATE;
    }
    if (status != TMC2240_OK) {
        disableAfterFailure(icID);
    } else {
        preparedValues[icID][0] = chop & ~TMC2240_TOFF_MASK;
    }
    return status;
}

TMC2240Status tmc2240_set_tpowerdown(uint16_t icID, uint8_t delay)
{
    return tmc2240_writeRegister(icID, TMC2240_TPOWERDOWN, delay);
}

TMC2240Status tmc2240_diag_configure(uint16_t icID, uint8_t diag0_error,
    uint8_t diag0_otpw, uint8_t diag0_stall, uint8_t diag1_stall,
    uint8_t diag1_index, uint8_t diag1_onstate, uint8_t pushpull)
{
    uint32_t bits;
    if ((diag0_error > 1U) || (diag0_otpw > 1U) || (diag0_stall > 1U) ||
        (diag1_stall > 1U) || (diag1_index > 1U) || (diag1_onstate > 1U) ||
        (pushpull > 1U)) {
        return TMC2240_ERROR_RANGE;
    }
    bits = ((uint32_t)diag0_error << 5U) | ((uint32_t)diag0_otpw << 6U) |
           ((uint32_t)diag0_stall << 7U) | ((uint32_t)diag1_stall << 8U) |
           ((uint32_t)diag1_index << 9U) | ((uint32_t)diag1_onstate << 10U) |
           ((uint32_t)pushpull << 12U) | ((uint32_t)pushpull << 13U);
    return tmc2240_updateRegister(icID, TMC2240_GCONF, 0x37E0U, bits);
}

static int16_t signed9(uint32_t value)
{
    value &= 511U;
    return (int16_t)((value < 256U) ? (int32_t)value : (int32_t)value - 512);
}

TMC2240Status tmc2240_pwm_get_scale(uint16_t icID, uint16_t *sum, int16_t *autoScale)
{
    uint32_t value;
    TMC2240Status status;
    if ((sum == NULL) && (autoScale == NULL)) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = tmc2240_readRegister(icID, TMC2240_PWM_SCALE, &value);
    if (status == TMC2240_OK) {
        if (sum != NULL) {
            *sum = (uint16_t)(value & 1023U);
        }
        if (autoScale != NULL) {
            *autoScale = signed9(value >> 16U);
        }
    }
    return status;
}

TMC2240Status tmc2240_get_microstep_counter(uint16_t icID, uint16_t *counter)
{
    return readUnsigned16(icID, TMC2240_MSCNT_FIELD, counter);
}

TMC2240Status tmc2240_get_microstep_current(uint16_t icID, int16_t *cur_a, int16_t *cur_b)
{
    uint32_t value;
    TMC2240Status status;
    if ((cur_a == NULL) && (cur_b == NULL)) {
        return TMC2240_ERROR_ARGUMENT;
    }
    status = tmc2240_readRegister(icID, TMC2240_MSCURACT, &value);
    if (status == TMC2240_OK) {
        if (cur_a != NULL) {
            *cur_a = signed9(value >> 16U);
        }
        if (cur_b != NULL) {
            *cur_b = signed9(value);
        }
    }
    return status;
}
