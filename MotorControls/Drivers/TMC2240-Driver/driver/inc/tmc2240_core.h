/*******************************************************************************
 * Copyright © 2017 TRINAMIC Motion Control GmbH & Co. KG
 * (now owned by Analog Devices, Inc.),
 * Copyright © 2024 Analog Devices, Inc.
 *
 * Recovered from the complete CubeIDE core header. This distinct filename is
 * intentional: an umbrella and a core header cannot differ only by letter case.
 *******************************************************************************/
#ifndef TMC2240_CORE_H_
#define TMC2240_CORE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "tmc2240_status.h"
#include "TMC2240_HW_Abstraction.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef TMC2240_CACHE
#define TMC2240_CACHE 1
#endif
#ifndef TMC2240_ENABLE_TMC_CACHE
#define TMC2240_ENABLE_TMC_CACHE 1
#endif
#ifndef TMC2240_IC_CACHE_COUNT
#define TMC2240_IC_CACHE_COUNT 4U
#endif
#ifndef TMC_API_EXTERNAL_CRC_TABLE
#define TMC_API_EXTERNAL_CRC_TABLE 0
#endif
#if (TMC_API_EXTERNAL_CRC_TABLE != 0) && (TMC_API_EXTERNAL_CRC_TABLE != 1)
#error TMC_API_EXTERNAL_CRC_TABLE must be 0 or 1
#endif
#if (TMC2240_CACHE != 0) && (TMC2240_CACHE != 1)
#error TMC2240_CACHE must be 0 or 1
#endif
#if (TMC2240_ENABLE_TMC_CACHE != 0) && (TMC2240_ENABLE_TMC_CACHE != 1)
#error TMC2240_ENABLE_TMC_CACHE must be 0 or 1
#endif
#if (TMC2240_IC_CACHE_COUNT < 1) || (TMC2240_IC_CACHE_COUNT > 255)
#error TMC2240_IC_CACHE_COUNT must be in 1..255
#endif

/* Serialize the whole public operation, including both SPI read frames and
 * complete RMW/multi-register sequences. There is no internal RTOS lock.
 * Read outputs are unchanged on failure. OK on an unverified write means only
 * transport completion, not device acceptance. All registers are readable.
 */
TMC2240Status tmc2240_readRegister(uint16_t icID, uint8_t address, uint32_t *value);
TMC2240Status tmc2240_writeRegister(uint16_t icID, uint8_t address, uint32_t value);
TMC2240Status tmc2240_writeRegisterVerified(uint16_t icID, uint8_t address,
                                          uint32_t value);
TMC2240Status tmc2240_updateRegister(uint16_t icID, uint8_t address,
                                   uint32_t mask, uint32_t value);

/* Signed fields use mathematical signed values, unsigned fields accept the
 * complete uint32_t domain. Invalid/noncontiguous descriptors are rejected.
 * W1C field writes emit only the requested bits, never a read-modify-write.
 */
TMC2240Status tmc2240_fieldExtract(uint32_t data, RegisterField field, int64_t *value);
TMC2240Status tmc2240_fieldUpdate(uint32_t data, RegisterField field,
                                int64_t value, uint32_t *result);
TMC2240Status tmc2240_fieldRead(uint16_t icID, RegisterField field, int64_t *value);
TMC2240Status tmc2240_fieldWrite(uint16_t icID, RegisterField field, int64_t value);

typedef struct {
    uint32_t read_mask;
    uint32_t write_mask;
    uint32_t reset_value;
    uint32_t reset_mask; /* Known reset bits; dynamic/unspecified bits excluded. */
    bool write_one_to_clear;
} TMC2240RegisterInfo;

TMC2240Status tmc2240_getRegisterInfo(uint8_t address, TMC2240RegisterInfo *info);
TMC2240Status tmc2240_validateRegisterValue(uint8_t address, uint32_t value);

typedef struct {
    uint32_t desired;
    uint32_t confirmed;
    bool desired_valid;
    bool confirmed_valid;
    bool dirty;
} TMC2240CacheEntry;

typedef enum {
    TMC2240_CACHE_GET,
    TMC2240_CACHE_DESIRE,     /* entry.desired; invalidate previous confirmation */
    TMC2240_CACHE_OBSERVE,    /* entry.confirmed; only after successful read */
    TMC2240_CACHE_INVALIDATE, /* One register, retain desired value */
    TMC2240_CACHE_RESET,      /* All registers for an IC, retain desired values */
    TMC2240_CACHE_CLEAR       /* All registers for an IC, forget desired values */
} TMC2240CacheOp;

/* With CACHE=1 and ENABLE_TMC_CACHE=0 the application supplies this hook.
 * GET/DESIRE/OBSERVE require entry; RESET/CLEAR ignore address and entry.
 * Return an error without changing state on failure. A write is never itself
 * an OBSERVE. dirty = desired_valid && (!confirmed_valid || desired!=confirmed).
 */
#if TMC2240_CACHE
TMC2240Status tmc2240_cache(uint16_t icID, TMC2240CacheOp operation,
                          uint8_t address, TMC2240CacheEntry *entry);
#endif
TMC2240Status tmc2240_initCache(uint16_t count);
TMC2240Status tmc2240_invalidateCache(uint16_t icID);
TMC2240Status tmc2240_getCachedRegister(uint16_t icID, uint8_t address,
                                      TMC2240CacheEntry *entry);

#ifdef __cplusplus
}
#endif
#endif
