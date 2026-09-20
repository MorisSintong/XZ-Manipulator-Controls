/**
 * @file tmc2209_reg.h
 * @brief TMC2209 register addresses, bitfield unions, and default values.
 *
 * Ported from the Arduino TMC2209_REG.h to plain C (GCC, Cortex-M4).
 * Bitfield layout assumes GCC little-endian on ARM (CubeIDE default).
 *
 * @see TMC2209 datasheet (docs/TMC2209-Datasheet.pdf)
 * @author Original: Anton Khrustalev, 2023 — GPLv3
 */

#ifndef TMC2209_REG_H
#define TMC2209_REG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================== */
/*  Register bitfield unions                                           */
/* ================================================================== */

/**
 * @brief GCONF — Global configuration flags: R/W, datasheet §5.1, p.23.
 */
typedef union {
    struct {
        uint32_t i_scale_analog    : 1;  /**< 0: internal; 1: external VREF            */
        uint32_t internal_rsense   : 1;  /**< 0: external R; 1: internal R              */
        uint32_t enable_spread_cycle:1;  /**< 0: StealthChop; 1: SpreadCycle             */
        uint32_t shaft             : 1;  /**< 0: normal; 1: invert direction             */
        uint32_t index_otpw        : 1;  /**< INDEX output: 0: microstep; 1: overtemp    */
        uint32_t index_step        : 1;  /**< INDEX output: 0: index_otpw; 1: steps      */
        uint32_t pdn_disable       : 1;  /**< 1: UART mode enabled (for UART use)        */
        uint32_t mstep_reg_select  : 1;  /**< 0: MS pins; 1: internal register           */
        uint32_t multistep_filt    : 1;  /**< 0: disable; 1: enable step pulse filter    */
        uint32_t test_mode         : 1;  /**< Must be zero for user operation. */
        uint32_t reserved          : 22; /**< DO NOT USE */
    } REG;
    uint32_t UINT32;
} tmc2209_gconf_t;

/**
 * @brief GSTAT — Read and explicit write-one-to-clear, §5.1, p.24.
 */
typedef union {
    struct {
        uint8_t reset     : 1;  /**< Latched reset, acknowledge with W1C. */
        uint8_t drv_err   : 1;  /**< Latched driver fault, W1C after cause clears. */
        uint8_t uv_cp     : 1;  /**< Live charge-pump undervoltage; not latched. */
        uint8_t reserved  : 5;  /**< DO NOT USE                       */
    } REG;
    uint8_t UINT8;
} tmc2209_gstat_t;

/**
 * @brief SLAVECONF — SENDDELAY for read access: WRITE
 * @see Datasheet §5.1, p.24
 */
typedef union {
    struct {
        uint16_t reserved1 : 8;  /**< DO NOT USE               */
        uint16_t senddelay : 4;  /**< (2*floor(value/2)+1)*8 bit times; >=2 for multi-slave. */
        uint16_t reserved2 : 4;  /**< DO NOT USE               */
    } REG;
    uint16_t UINT16;
} tmc2209_slaveconf_t;

/**
 * @brief IHOLD_IRUN — Driver current control: WRITE
 * @see Datasheet §5.2, p.28
 */
typedef union {
    struct {
        uint32_t ihold      : 5;  /**< Standstill current 0=1/32 … 31=32/32  */
        uint32_t reserved1  : 3;  /**< DO NOT USE                             */
        uint32_t irun       : 5;  /**< Run current 0=1/32 … 31=32/32         */
        uint32_t reserved2  : 3;  /**< DO NOT USE                             */
        uint32_t iholddelay : 4;  /**< Delay per reduction step: n * 2^18 clocks. */
        uint32_t reserved3  : 12; /**< DO NOT USE                             */
    } REG;
    uint32_t UINT32;
} tmc2209_ihold_irun_t;

/**
 * @brief IOIN — Driver pin status: READ
 * @see Datasheet §5.1, pp.24-25
 */
typedef union {
    struct {
        uint32_t enn        : 1;  /**< ENN pin state      */
        uint32_t reserved1  : 1;  /**< DO NOT USE         */
        uint32_t ms1        : 1;  /**< MS1 pin state      */
        uint32_t ms2        : 1;  /**< MS2 pin state      */
        uint32_t diag       : 1;  /**< DIAG pin state     */
        uint32_t reserved2  : 1;  /**< DO NOT USE         */
        uint32_t pdn_serial : 1;  /**< PDN_UART pin state */
        uint32_t step       : 1;  /**< STEP pin state     */
        uint32_t spread_en  : 1;  /**< SPREAD pin state   */
        uint32_t dir        : 1;  /**< DIR pin state      */
        uint32_t reserved3  : 14; /**< DO NOT USE         */
        uint32_t version    : 8;  /**< Chip version (0x21)*/
    } REG;
    uint32_t UINT32;
} tmc2209_ioin_t;

/**
 * @brief DRV_STATUS — Driver status flags and current level: READ
 * @see Datasheet §5.5.3 (DRV_STATUS table)
 */
typedef union {
    struct {
        uint32_t otpw       : 1;  /**< 0  Overtemperature prewarning          */
        uint32_t ot         : 1;  /**< 1  Overtemperature flag                */
        uint32_t s2ga       : 1;  /**< 2  Short to GND phase A                */
        uint32_t s2gb       : 1;  /**< 3  Short to GND phase B                */
        uint32_t s2vsa      : 1;  /**< 4  Low-side short phase A              */
        uint32_t s2vsb      : 1;  /**< 5  Low-side short phase B              */
        uint32_t ola        : 1;  /**< 6  Open load phase A                   */
        uint32_t olb        : 1;  /**< 7  Open load phase B                   */
        uint32_t t120       : 1;  /**< 8  Overtemperature 120°C comparator    */
        uint32_t t143       : 1;  /**< 9  Overtemperature 143°C comparator    */
        uint32_t t150       : 1;  /**< 10 Overtemperature 150°C comparator    */
        uint32_t t157       : 1;  /**< 11 Overtemperature 157°C comparator    */
        uint32_t reserved1  : 4;  /**< 12..15  DO NOT USE                     */
        uint32_t cs_actual  : 5;  /**< 16..20  Actual current scaling         */
        uint32_t reserved2  : 9;  /**< 21..29  DO NOT USE                     */
        uint32_t stealth    : 1;  /**< 30 StealthChop indicator               */
        uint32_t stst       : 1;  /**< 31 Standstill indicator                */
    } REG;
    uint32_t UINT32;
} tmc2209_drv_status_t;

/**
 * @brief CHOPCONF — Chopper configuration: R/W
 * @see Datasheet §5.5.1, pp.33-34
 */
typedef union {
    struct {
        uint32_t toff        : 4;  /**< 0: off; 1 requires TBL>=2; 2..15: off time. */
        uint32_t hstrt       : 3;  /**< Effective start addition = encoded + 1. */
        uint32_t hend        : 4;  /**< Effective end = encoded - 3; effective sum <=16. */
        uint32_t reserved1   : 4;  /**< DO NOT USE                                      */
        uint32_t tbl         : 2;  /**< Comparator blank time select                    */
        uint32_t vsense      : 1;  /**< 0: low sensitivity; 1: high sensitivity         */
        uint32_t reserved2   : 6;  /**< DO NOT USE                                      */
        uint32_t mres        : 4;  /**< Microstep resolution 0..8 → 256..1              */
        uint32_t interpolation: 1; /**< 0: off; 1: 256-interpolation enabled            */
        uint32_t dedge       : 1;  /**< 0: off; 1: double edge step pulses              */
        uint32_t diss2g      : 1;  /**< 0: GND short protection on; 1: off              */
        uint32_t diss2vs     : 1;  /**< 0: VSS short protection on; 1: off              */
    } REG;
    uint32_t UINT32;
} tmc2209_chopconf_t;

/**
 * @brief PWMCONF — StealthChop PWM configuration: R/W
 * @see Datasheet §5.5.2, pp.35-36
 */
typedef union {
    struct {
        uint32_t pwm_offset   : 8;  /**< User-defined amplitude offset                 */
        uint32_t pwm_grad     : 8;  /**< User-defined amplitude gradient               */
        uint32_t pwm_freq     : 2;  /**< 0..3: fCLK*2/{1024,683,512,410}. */
        uint32_t pwm_autoscale: 1;  /**< Automatic amplitude scaling                   */
        uint32_t pwm_autograd : 1;  /**< Automatic gradient adaptation                 */
        uint32_t freewheel    : 2;  /**< Standstill mode: 0b01 free; 0b10 passive brake*/
        uint32_t reserved     : 2;  /**< DO NOT USE                                    */
        uint32_t pwm_reg      : 4;  /**< Regulation loop gradient                      */
        uint32_t pwm_lim      : 4;  /**< Automatic scale limit at switch-on            */
    } REG;
    uint32_t UINT32;
} tmc2209_pwmconf_t;

/**
 * @brief COOLCONF — CoolStep configuration: WRITE
 * @see Datasheet §5.3.1, p.30
 */
typedef union {
    struct {
        uint32_t semin     : 4;  /**< CoolStep lower threshold                */
        uint32_t reserved1 : 1;  /**< DO NOT USE                              */
        uint32_t seup      : 2;  /**< Current increase step: 00..11 → 1,2,4,8 */
        uint32_t reserved2 : 1;  /**< DO NOT USE                              */
        uint32_t semax     : 4;  /**< CoolStep upper threshold                */
        uint32_t reserved3 : 1;  /**< DO NOT USE                              */
        uint32_t sedn      : 2;  /**< Decrease one step per 32,8,2,1 samples. */
        uint32_t seimin    : 1;  /**< Minimum current for smart control        */
        uint32_t reserved4 : 16; /**< DO NOT USE                              */
    } REG;
    uint32_t UINT32;
} tmc2209_coolconf_t;

/**
 * @brief TPWMTHRS / TCOOLTHRS — Threshold velocity registers: WRITE
 * @see Datasheet §5.2 / §5.3, pp.28-29
 */
typedef union {
    struct {
        uint32_t threshold : 20; /**< Threshold velocity value  */
        uint32_t reserved  : 12; /**< DO NOT USE               */
    } REG;
    uint32_t UINT32;
} tmc2209_thrs_t;

/* ================================================================== */
/*  Register addresses                                                 */
/* ================================================================== */

/**
 * @brief TMC2209 register addresses.
 *
 * All addresses are bare; only the frame builder adds the write bit.
 * All documented readable registers are available through raw reads.
 * OTP programming, factory-trim writes and nonzero UART velocity are unsupported.
 *
 * @see Datasheet §5 register map summary
 */
typedef enum {
    TMC2209_REG_GCONF       = 0x00u, /**< R/W */
    TMC2209_REG_SLAVECONF   = 0x03u, /**< W */
    TMC2209_REG_IHOLD_IRUN  = 0x10u, /**< W */
    TMC2209_REG_TPOWERDOWN  = 0x11u, /**< W */
    TMC2209_REG_TPWMTHRS    = 0x13u, /**< W */
    TMC2209_REG_TCOOLTHRS   = 0x14u, /**< W */
    TMC2209_REG_VACTUAL     = 0x22u, /**< W signed 24-bit; ONLY zero (STEP/DIR) supported. */
    TMC2209_REG_SGTHRS      = 0x40u, /**< W */
    TMC2209_REG_COOLCONF    = 0x42u, /**< W */
    TMC2209_REG_CHOPCONF    = 0x6Cu, /**< R/W */
    TMC2209_REG_PWMCONF     = 0x70u, /**< R/W */

    /* READ registers (bare address) */
    TMC2209_REG_GSTAT       = 0x01u,          /**< Read + W1C global status */
    TMC2209_REG_IFCNT       = 0x02u,          /**< Interface transmission counter     */
    TMC2209_REG_OTP_READ    = 0x05u,          /**< R: OTP bytes 23:0; no programming support */
    TMC2209_REG_IOIN        = 0x06u,          /**< Input / pin status                 */
    TMC2209_REG_FACTORY_CONF = 0x07u,         /**< Read trim fields; writes unsupported */
    TMC2209_REG_TSTEP       = 0x12u,          /**< R: measured step period 19:0 */
    TMC2209_REG_SG_RESULT   = 0x41u,          /**< StallGuard result                  */
    TMC2209_REG_MSCNT       = 0x6Au,          /**< R: microstep counter 9:0 */
    TMC2209_REG_MSCURACT    = 0x6Bu,          /**< R: signed 9-bit phase B 8:0, phase A 24:16 */
    TMC2209_REG_DRV_STATUS  = 0x6Fu,          /**< Driver status                      */
    TMC2209_REG_PWM_SCALE   = 0x71u,          /**< R: sum 7:0, signed 9-bit auto offset 24:16 */
    TMC2209_REG_PWM_AUTO    = 0x72u,          /**< R: offset 7:0, gradient 23:16 */
} tmc2209_reg_t;

/* ================================================================== */
/*  Supported writable masks / nominal reset values, Rev.1.08         */
/* ================================================================== */

#define TMC2209_GCONF_WRITE_MASK      UINT32_C(0x000001FF) /* test_mode deliberately forbidden */
#define TMC2209_GSTAT_WRITE_MASK      UINT32_C(0x00000003) /* latched flags only */
#define TMC2209_SLAVECONF_WRITE_MASK  UINT32_C(0x00000F00)
#define TMC2209_IHOLD_IRUN_WRITE_MASK UINT32_C(0x000F1F1F)
#define TMC2209_CHOPCONF_WRITE_MASK   UINT32_C(0xFF0387FF)
#define TMC2209_PWMCONF_WRITE_MASK    UINT32_C(0xFF3FFFFF)
#define TMC2209_COOLCONF_WRITE_MASK   UINT32_C(0x0000EF6F)
#define TMC2209_THRS_WRITE_MASK       UINT32_C(0x000FFFFF)
#define TMC2209_VACTUAL_WRITE_MASK    UINT32_C(0x00FFFFFF) /* additional policy: zero only */
#define TMC2209_BYTE_WRITE_MASK       UINT32_C(0x000000FF)
#define TMC2209_IOIN_VERSION          UINT32_C(0x21)
#define TMC2209_GSTAT_FAULT_MASK      UINT32_C(0x00000007) /* reset, drv_err, live uv_cp */
#define TMC2209_DRV_STATUS_FAULT_MASK UINT32_C(0x0000003F) /* otpw, ot, short-circuit flags */

/* Informational only, NEVER an application motor configuration.
 * OTP modifies GCONF, currents, chopper and PWM reset fields (§5.1 pp.26-27).
 * The nominal whole-register values below are the §5.5 table, p.32.
 */
#define TMC2209_CHOPCONF_RESET_U32 UINT32_C(0x10000053)
#define TMC2209_PWMCONF_RESET_U32  UINT32_C(0xC10D0024)
#define TMC2209_TPOWERDOWN_RESET_U8 UINT8_C(20)

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_REG_H */
