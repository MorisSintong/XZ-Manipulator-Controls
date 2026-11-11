#include "diag_record.h"
#include "crc16_ccitt.h"
#include <string.h>

static void put_u8(uint8_t *p, uint32_t v) { p[0] = (uint8_t)v; }
static void put_u16(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8U);
}
static void put_u32(uint8_t *p, uint32_t v)
{
    put_u16(p, v); put_u16(p + 2, v >> 16U);
}
static void put_i32(uint8_t *p, int32_t v) { put_u32(p, (uint32_t)v); }

size_t diag_record_pack(const diag_record_t *r, uint8_t *out, size_t capacity)
{
    if (r == NULL || out == NULL) { return 0U; }
    uint16_t payload;
    switch (r->type) {
    case DIAG_COMMAND_RESULT: payload = 312U; break;
    case DIAG_HOME_RESULT: payload = 80U; break;
    case DIAG_STATUS: payload = 72U; break;
    default: return 0U;
    }
    const size_t len = 44U + payload;
    if (capacity < len) { return 0U; }
    memset(out, 0, len);
    out[0] = 0xD3U; out[1] = 0x7EU; out[2] = 1U; out[3] = r->type;
    put_u16(out + 4, payload);
    put_u32(out + 6, r->record_seq); put_u32(out + 10, r->timestamp_ms);
    put_u32(out + 14, r->command_seq); put_u32(out + 18, r->home_epoch);
    put_u32(out + 22, r->status_bits); put_u16(out + 26, r->command.obj_id);
    out[28] = r->command.class_id; out[29] = r->command.type;
    put_u16(out + 30, (uint16_t)r->command.x_01mm);
    put_u16(out + 32, (uint16_t)r->command.y_01mm);
    put_u16(out + 34, r->command.angle_01deg);
    put_u16(out + 36, (uint16_t)r->command.corr_01deg);
    out[38] = r->type == DIAG_COMMAND_RESULT ? r->phase_count : 0U;
    out[39] = r->config_id;
    if (r->type == DIAG_COMMAND_RESULT) {
        for (size_t i = 0U; i < 3U; ++i) {
            uint8_t *p = out + 40U + i * 104U;
            const diag_phase_t *s = &r->data.phases[i];
#define PACK(t,n,o) put_##t(p + o, s->n);
            DIAG_PHASE_FIELDS(PACK)
#undef PACK
        }
    } else if (r->type == DIAG_HOME_RESULT) {
        for (size_t i = 0U; i < 2U; ++i) {
            uint8_t *p = out + 40U + i * 40U;
            const diag_home_t *s = &r->data.home[i];
#define PACK(t,n,o) put_##t(p + o, s->n);
            DIAG_HOME_FIELDS(PACK)
#undef PACK
        }
    } else {
        uint8_t *p = out + 40;
        const diag_status_t *s = &r->data.status;
#define PACK(t,n,o) put_##t(p + o, s->n);
        DIAG_STATUS_FIELDS(PACK)
#undef PACK
    }
    put_u16(out + 40U + payload, crc16_ccitt(out + 2, 38U + payload));
    out[42U + payload] = 13U; out[43U + payload] = 10U;
    return len;
}
