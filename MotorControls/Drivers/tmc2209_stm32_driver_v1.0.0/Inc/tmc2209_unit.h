/**
 * @file tmc2209_unit.h
 * @brief TMC2209 bus and unit (per-chip instance) definitions.
 *
 * A bus represents one physical UART; a unit represents one TMC2209
 * chip (address 0..3) on that bus.  Multiple units may share one bus;
 * FreeRTOS wire access is serialised on the bus mutex. Bare metal requires
 * one execution context. Use exactly one bus per physical UART.
 *
 * Lock order (deadlock prevention):
 *   bus->lock  →  UART ops  →  unlock bus
 *
 * The unit register mirrors are owned by the calling task; if several
 * tasks touch one unit, the application must serialise them.
 */

#ifndef TMC2209_UNIT_H
#define TMC2209_UNIT_H

#include "tmc2209_os.h"
#include "tmc2209_port.h"
#include "tmc2209_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================== */
/*  Bus — one physical UART + port + bus mutex                         */
/* ================================================================== */

/**
 * @brief Physical UART bus shared by one or more TMC2209 units.
 *
 * Caller owns storage and MUST zero-initialise it before first use.
 * Never copy a live bus/unit. Lifecycle operations require exclusive access.
 */
typedef struct {
  tmc2209_port_t port;  /**< UART transport config          */
  tmc2209_mutex_t lock; /**< Bus mutex (one per UART wire)  */
  bool initialized;
  bool owns_lock;
  uint8_t bound_addresses; /**< One unit per address; deinit units before bus. */
} tmc2209_bus_t;

/* ================================================================== */
/*  Unit — one TMC2209 chip instance                                   */
/* ================================================================== */

/**
 * @brief Per-chip state: address + register mirrors.
 *
 * Caller owns zero-initialised storage; initialise with TMC2209_UnitInit().
 * The mirrors are task-owned. Control fields (bus, address, configured,
 * desired_valid) must not be modified by the application.
 */
typedef struct {
  tmc2209_bus_t *bus; /**< Back-pointer to shared bus       */
  uint8_t address;    /**< UART slave address 0..3          */
  bool configured;   /**< Full explicit configuration was accepted; not a hardware monitor. */
  uint16_t desired_valid; /**< Which writable mirrors contain a requested value. */

  /* ---- WRITE register mirrors ---- */
  tmc2209_gconf_t gconf;           /**< GCONF               */
  tmc2209_slaveconf_t slaveconf;   /**< SLAVECONF           */
  tmc2209_ihold_irun_t ihold_irun; /**< IHOLD_IRUN          */
  tmc2209_chopconf_t chopconf;     /**< CHOPCONF            */
  tmc2209_pwmconf_t pwmconf;       /**< PWMCONF             */
  tmc2209_coolconf_t coolconf;     /**< COOLCONF            */
  tmc2209_thrs_t tcoolthrs;        /**< TCOOLTHRS           */
  tmc2209_thrs_t tpwmthrs;         /**< TPWMTHRS            */
  uint8_t sgthrs;                  /**< SGTHRS              */
  uint8_t tpowerdown;              /**< TPOWERDOWN           */

  /* ---- READ register mirrors ---- */
  tmc2209_gstat_t gstat;           /**< GSTAT (last read)    */
  tmc2209_ioin_t ioin;             /**< IOIN (last read)     */
  tmc2209_drv_status_t drv_status; /**< DRV_STATUS (last)    */
  uint16_t sg_result;              /**< SG_RESULT (last)     */
  uint8_t ifcnt;                   /**< IFCNT (last read)    */
} tmc2209_unit_t;

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_UNIT_H */
