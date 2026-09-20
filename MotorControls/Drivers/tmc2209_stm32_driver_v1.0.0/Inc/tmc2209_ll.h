#ifndef TMC2209_LL_H
#define TMC2209_LL_H
#include "tmc2209_unit.h"
#ifdef __cplusplus
extern "C" {
#endif
#define TMC2209_SYNC 0x05u
#define TMC2209_REPLY_ADDR 0xFFu
#define TMC2209_WRITE_BIT 0x80u
#define TMC2209_CRC_START_A0 0x18u
#define TMC2209_CRC_START_A1 0x91u
#define TMC2209_CRC_START_A2 0xDFu
#define TMC2209_CRC_START_A3 0x56u
#define TMC2209_CRC_START_REPLY 0xEBu

uint8_t tmc2209_crc_start(uint8_t address);
uint8_t tmc2209_crc_byte(uint8_t byte, uint8_t crc);
/** Feed data MSB first, matching register payload wire order. */
uint8_t tmc2209_crc_u32(uint32_t data, uint8_t crc);
/** Frame builders accept addresses 0..3 and BARE registers 0..127. */
tmc2209_status_t tmc2209_pack_write(uint8_t addr, uint8_t reg, uint32_t data,
                                   uint8_t out[8]);
tmc2209_status_t tmc2209_pack_request(uint8_t addr, uint8_t reg, uint8_t out[4]);
/** Requires exact reply identity including the clear R/W bit. Outputs unchanged on error. */
tmc2209_status_t tmc2209_unpack_reply(const uint8_t reply[8], uint8_t expected_reg,
                                     uint32_t *data);
/**
 * Supported registers only, BARE addresses. Reject reserved/test bits and
 * unsupported accesses. Raw and typed writes share desired mirrors; a failed
 * transmission retains the request for retry. Neither a mirror nor OK from
 * this function confirms acceptance. All writes, including GSTAT acknowledgements,
 * invalidate configured. Nonzero CHOPCONF.TOFF requires successful Configure
 * and checked DRV_STATUS/GSTAT observations before and after the transmission.
 * Keep ENN externally disabled until the application's checked activation flow
 * succeeds; a failed on-write can already have reached the device.
 */
tmc2209_status_t TMC2209_WriteReg(tmc2209_unit_t *unit, uint8_t reg, uint32_t data);
/**
 * Holds one bus lock across IFCNT-before/write/IFCNT-after and RW readback.
 * Requires exactly one counter increment modulo 256. GSTAT W1C checks IFCNT
 * only. No other UART master or same-address software writer may interfere.
 * OK confirms acceptance at that time, NOT persistent device/hardware health.
 */
tmc2209_status_t TMC2209_WriteRegVerified(tmc2209_unit_t *unit, uint8_t reg,
                                        uint32_t data);
/** Read any documented readable register, including raw OTP/trim/sequencer/PWM
 * diagnostics. No typed conversion, masking or cache update is performed.
 * Reads do not acknowledge GSTAT. Output unchanged on error.
 */
tmc2209_status_t TMC2209_ReadReg(tmc2209_unit_t *unit, uint8_t reg, uint32_t *data);
#ifdef __cplusplus
}
#endif
#endif
