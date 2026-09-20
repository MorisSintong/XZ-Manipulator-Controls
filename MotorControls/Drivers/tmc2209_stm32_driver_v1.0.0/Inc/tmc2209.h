/**
 * Task-context TMC2209 driver. Zero-init bus/unit; externally assert ENN first.
 * BusInit -> UnitInit -> observe/explicitly acknowledge faults -> Configure -> Activate.
 * Transport init NEVER writes a chip register. Reset defaults can energize a
 * motor: only external ENN/hardware control can guarantee disabled outputs.
 * Mirrors belong to the calling task; serialize ALL operations on a shared unit.
 */
#ifndef TMC2209_H
#define TMC2209_H
#include "tmc2209_ll.h"
#ifdef __cplusplus
extern "C" {
#endif

tmc2209_status_t TMC2209_BusInit(tmc2209_bus_t *bus, UART_HandleTypeDef *huart,
                                uint32_t baud);
/** External native mutex is never deleted. Required when OS_CREATE_MUTEX=0. */
tmc2209_status_t TMC2209_BusInitWithLock(tmc2209_bus_t *bus,
                                       UART_HandleTypeDef *huart, uint32_t baud,
                                       tmc2209_mutex_t lock);
/** Deinit all bound units first. No chip registers are written. */
tmc2209_status_t TMC2209_BusDeinit(tmc2209_bus_t *bus);
tmc2209_status_t TMC2209_UnitInit(tmc2209_unit_t *unit, tmc2209_bus_t *bus,
                                 uint8_t addr);
/** Does not disable a motor; use external ENN and/or Deactivate first. */
tmc2209_status_t TMC2209_UnitDeinit(tmc2209_unit_t *unit);

/** Explicit values; no universal motor/current default is provided. */
typedef struct {
    uint32_t gconf;      /**< Require pdn_disable and mstep_reg_select. */
    uint32_t slaveconf;  /**< Require SENDDELAY >= 2, including multi-slave use. */
    uint32_t ihold_irun;
    uint32_t chopconf;   /**< TOFF must be zero. */
    uint32_t pwmconf;
    uint32_t coolconf;
    uint32_t tcoolthrs;
    uint32_t tpwmthrs;
    uint32_t sgthrs;
    uint32_t tpowerdown;
} tmc2209_motor_config_t;

/**
 * Replaces SetupDefault. Validates every input, verifies VERSION=0x21 before
 * any write, confirms CHOPCONF.TOFF=0 first, then all configuration writes.
 * Always confirms VACTUAL=0 for STEP/DIR operation; nonzero UART motion is
 * deliberately unsupported, including raw-register writes.
 * Read DRV_STATUS/GSTAT and resolve causes first. Explicitly acknowledge GSTAT
 * reset/drv_err with ClearGSTAT before Configure, never between Configure/Activate.
 * Configure checks DRV_STATUS bits 0..5 and GSTAT bits 0..2 before/after writes,
 * returning ERR_FAULT for observed flags without clearing them.
 * Failure disarms activation. A failed disable cannot guarantee bridges off.
 * ENN must stay externally asserted throughout configuration and activation,
 * until Activate returns OK. On any error keep ENN asserted and reconfigure
 * only after diagnosis and deliberate acknowledgement of the latched flags.
 */
tmc2209_status_t TMC2209_Configure(tmc2209_unit_t *unit,
                                 const tmc2209_motor_config_t *config);
/** Enable TOFF=1..15 after Configure; recheck identity, fault/reset preconditions,
 * acceptance/readback and final fault/reset state. No flags are auto-acknowledged.
 * A failure after an on-write may leave TOFF nonzero: keep external ENN asserted.
 * These synchronous observations cannot detect a new event after the final sample.
 */
tmc2209_status_t TMC2209_Activate(tmc2209_unit_t *unit, uint8_t toff);
/** Verified TOFF=0, requires an existing desired CHOPCONF; disarms activation. */
tmc2209_status_t TMC2209_Deactivate(tmc2209_unit_t *unit);
/** Wrong silicon version returns ERR_NODEV and false. Any failed presence check
 * disarms activation. Hardware reset/brownout still requires explicit reconfiguration;
 * configured is not continuous hardware-health monitoring.
 */
tmc2209_status_t TMC2209_Available(tmc2209_unit_t *unit, bool *present);

/**
 * Typed writes mean transmission, NOT confirmed device acceptance.
 * UINT32_MAX retries the desired mirror; before first request it is ERR_STATE.
 * Invalid values never replace mirrors; transport failures retain desired values.
 */
tmc2209_status_t TMC2209_WriteGCONF(tmc2209_unit_t *unit, uint32_t value);
tmc2209_status_t TMC2209_WriteSLAVECONF(tmc2209_unit_t *unit, uint32_t value);
tmc2209_status_t TMC2209_WriteIHOLD_IRUN(tmc2209_unit_t *unit, uint32_t value);
tmc2209_status_t TMC2209_WriteCHOPCONF(tmc2209_unit_t *unit, uint32_t value);
tmc2209_status_t TMC2209_WritePWMCONF(tmc2209_unit_t *unit, uint32_t value);
tmc2209_status_t TMC2209_WriteCOOLCONF(tmc2209_unit_t *unit, uint32_t value);
tmc2209_status_t TMC2209_WriteTCOOLTHRS(tmc2209_unit_t *unit, uint32_t value);
tmc2209_status_t TMC2209_WriteTPWMTHRS(tmc2209_unit_t *unit, uint32_t value);
tmc2209_status_t TMC2209_WriteSGTHRS(tmc2209_unit_t *unit, uint32_t value);
tmc2209_status_t TMC2209_WriteTPOWERDOWN(tmc2209_unit_t *unit, uint32_t value);

tmc2209_status_t TMC2209_ReadIOIN(tmc2209_unit_t *unit, uint32_t *data);
tmc2209_status_t TMC2209_ReadSG_RESULT(tmc2209_unit_t *unit, uint32_t *data);
tmc2209_status_t TMC2209_ReadIFCNT(tmc2209_unit_t *unit, uint32_t *data);
tmc2209_status_t TMC2209_ReadGSTAT(tmc2209_unit_t *unit, uint32_t *data);
tmc2209_status_t TMC2209_ReadDRV_STATUS(tmc2209_unit_t *unit, uint32_t *data);
/** Deliberate W1C acknowledgement of reset/drv_err (bits 0/1); never RMW GSTAT.
 * Any valid GSTAT write attempt disarms the previous profile, even if it fails.
 * Configure must succeed again before any activating write. Acknowledgement
 * acceptance does not prove the fault cause is gone; Configure checks it.
 */
tmc2209_status_t TMC2209_ClearGSTAT(tmc2209_unit_t *unit, uint8_t flags);
#ifdef __cplusplus
}
#endif
#endif
