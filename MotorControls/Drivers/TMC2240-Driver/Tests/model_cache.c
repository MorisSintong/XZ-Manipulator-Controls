#include "model_cache.h"
#include "model_device.h"
#include <string.h>

int custom_cache_fail_op = -1;
unsigned custom_cache_fail_at;
unsigned custom_cache_calls;

void custom_cache_reset_faults(void)
{
    custom_cache_fail_op = -1;
    custom_cache_fail_at = 0U;
    custom_cache_calls = 0U;
}

#if TMC2240_CACHE && !TMC2240_ENABLE_TMC_CACHE
static TMC2240CacheEntry custom_entries[4][128];
TMC2240Status tmc2240_cache(uint16_t icID, TMC2240CacheOp operation,
                          uint8_t address, TMC2240CacheEntry *entry)
{
    ++custom_cache_calls;
    if (((int)operation == custom_cache_fail_op) ||
        (custom_cache_calls == custom_cache_fail_at)) {
        if ((operation == TMC2240_CACHE_GET) && (entry != NULL)) {
            memset(entry, 0xA5, sizeof(*entry));
        }
        return TMC2240_ERROR_IO;
    }
    if (icID >= 4U) {
        return TMC2240_ERROR_ID;
    }
    if (operation == TMC2240_CACHE_CLEAR) {
        memset(custom_entries[icID], 0, sizeof(custom_entries[icID]));
        return TMC2240_OK;
    }
    if (operation == TMC2240_CACHE_RESET) {
        for (unsigned i = 0U; i < 128U; ++i) {
            custom_entries[icID][i].confirmed_valid = false;
            custom_entries[icID][i].dirty = custom_entries[icID][i].desired_valid;
        }
        return TMC2240_OK;
    }
    const SpecRegister *reg = model_register(address);
    if (reg == NULL) {
        return TMC2240_ERROR_ADDRESS;
    }
    if (entry == NULL && operation != TMC2240_CACHE_INVALIDATE) {
        return TMC2240_ERROR_ARGUMENT;
    }
    TMC2240CacheEntry *stored = &custom_entries[icID][address];
    switch (operation) {
    case TMC2240_CACHE_GET:
        *entry = *stored;
        return TMC2240_OK;
    case TMC2240_CACHE_DESIRE:
        stored->desired = entry->desired & reg->writable;
        stored->desired_valid = true;
        stored->confirmed_valid = false;
        break;
    case TMC2240_CACHE_OBSERVE:
        stored->confirmed = entry->confirmed & reg->readable;
        stored->confirmed_valid = true;
        break;
    case TMC2240_CACHE_INVALIDATE:
        stored->confirmed_valid = false;
        break;
    default:
        return TMC2240_ERROR_ARGUMENT;
    }
    stored->dirty = stored->desired_valid &&
        (!stored->confirmed_valid || stored->desired != (stored->confirmed & reg->writable));
    return TMC2240_OK;
}
#endif
