#include "tmc2240_demo_startup.h"

TMC2240Status tmc_demo_prepare_motor(uint16_t id, uint16_t run_mA, uint16_t rref_ohm,
                                   uint8_t acknowledge_startup_flags)
{
    if ((acknowledge_startup_flags & ~0x1FU) != 0U) {
        return TMC2240_ERROR_ARGUMENT;
    }
    TMC2240_MotorConfig_t motor = {
        .gconf = TMC2240_EN_PWM_MODE_MASK,
        .drv_conf = 2U << TMC2240_SLOPE_CONTROL_SHIFT,
        .global_scaler = 0U,
        .ihold_irun = 0U,
        .chopconf = 0x14410150U,
        .pwmconf = 0xC40C001DU
    };
    TMC2240Status status = tmc2240_calculateCurrent(run_mA, rref_ohm, 0U, 0U,
                                                  50U, 5U, 5U, &motor.ihold_irun);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_hal_testConnection(id);
    if (status != TMC2240_OK) {
        return status;
    }
    uint8_t faults = 0U;
    status = tmc2240_check_faults(id, &faults);
    if (status != TMC2240_OK) {
        return status;
    }
    if ((faults & (uint8_t)~acknowledge_startup_flags) != 0U) {
        return TMC2240_ERROR_FAULT;
    }
    if (faults != 0U) {
        status = tmc2240_clear_faults(id, faults);
        if (status != TMC2240_OK) {
            return status;
        }
    }
    for (uint8_t address = 0x60U; address <= 0x69U; address++) {
        TMC2240RegisterInfo info;
        status = tmc2240_getRegisterInfo(address, &info);
        if (status != TMC2240_OK) {
            return status;
        }
        if ((info.reset_mask & info.write_mask) != info.write_mask) {
            return TMC2240_ERROR_STATE;
        }
        status = tmc2240_writeRegisterVerified(id, address, info.reset_value);
        if (status != TMC2240_OK) {
            return status;
        }
    }
    status = tmc2240_set_tpwmthrs(id, 0U);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_set_thigh(id, 0U);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_set_tpowerdown(id, 20U);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_set_tcoolthrs(id, 3000U);
    if (status != TMC2240_OK) {
        return status;
    }
    status = tmc2240_stallguard_set_threshold(id, 200U);
    if (status != TMC2240_OK) {
        return status;
    }
    return tmc2240_hal_configureMotor(id, &motor);
}
