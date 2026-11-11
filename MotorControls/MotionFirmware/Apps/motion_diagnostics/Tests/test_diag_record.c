#include "diag_record.h"
#include "crc16_ccitt.h"
#include <assert.h>
#include <limits.h>
#include <string.h>

static uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (uint16_t)(p[1] << 8U)); }
static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8U) |
           ((uint32_t)p[2] << 16U) | ((uint32_t)p[3] << 24U);
}
static void framing(diag_record_t *r, size_t size, uint16_t payload)
{
    uint8_t bytes[DIAG_MAX_FRAME + 1U];
    memset(bytes, 0xA5, sizeof(bytes));
    assert(diag_record_pack(r, bytes, size - 1U) == 0U);
    assert(bytes[0] == 0xA5U);
    assert(diag_record_pack(r, bytes, size) == size && bytes[size] == 0xA5U);
    assert(bytes[0] == 0xD3U && bytes[1] == 0x7EU && bytes[2] == 1U && bytes[3] == r->type);
    assert(le16(bytes + 4) == payload);
    assert(le32(bytes + 6) == r->record_seq && le32(bytes + 10) == r->timestamp_ms);
    assert(le32(bytes + 14) == r->command_seq && le32(bytes + 18) == r->home_epoch);
    assert(le32(bytes + 22) == r->status_bits && le16(bytes + 26) == r->command.obj_id);
    assert(bytes[28] == r->command.class_id && bytes[29] == r->command.type);
    assert(le16(bytes + 30) == (uint16_t)r->command.x_01mm);
    assert(le16(bytes + 32) == (uint16_t)r->command.y_01mm);
    assert(le16(bytes + 34) == r->command.angle_01deg);
    assert(le16(bytes + 36) == (uint16_t)r->command.corr_01deg);
    assert(bytes[38] == (r->type == DIAG_COMMAND_RESULT ? r->phase_count : 0U));
    assert(bytes[39] == r->config_id && bytes[size-2U] == 13U && bytes[size-1U] == 10U);
    const uint16_t crc = le16(bytes + size - 4U);
    assert(crc == crc16_ccitt(bytes + 2, size - 6U));
    for (size_t i = 2U; i < size - 4U; ++i) {
        bytes[i] ^= 1U;
        assert(crc16_ccitt(bytes + 2, size - 6U) != crc);
        bytes[i] ^= 1U;
    }
    bytes[0] ^= 1U; bytes[1] ^= 1U;
    assert(crc16_ccitt(bytes + 2, size - 6U) == crc);
}
int main(void)
{
    diag_record_t r = {.type=DIAG_COMMAND_RESULT, .phase_count=3U, .config_id=1U,
        .record_seq=0x12345678U, .timestamp_ms=UINT32_MAX, .command_seq=0x7ED3U,
        .home_epoch=55U, .status_bits=DIAG_ABORTED, .command={1U,65535U,2U,INT16_MIN,INT16_MAX,3599U,-1800}};
    uint8_t b[DIAG_MAX_FRAME];
    for (uint8_t i = 0U; i < 3U; ++i) {
        diag_phase_t *p = &r.data.phases[i];
        p->axis = (uint8_t)(i == 0U ? 0U : 1U); p->phase = (uint8_t)(i+1U);
        p->axis_flags=0x3FFFU; p->requested_target_01mm=INT32_MIN;
        p->applied_target_01mm=INT32_MAX; p->emitted_delta_steps=-1;
        p->emitted_edge_count=UINT32_MAX; p->step_angle_udeg=112500U;
        p->quant_residual_nm=-12345; p->mscnt_start=1023U; p->mscnt_end=0U;
        p->raw_start=4095U; p->raw_end=0U; p->unwrap_start=INT32_MIN; p->unwrap_end=INT32_MAX;
        p->as_status_start=0x20U; p->as_status_end=0x20U; p->agc_start=64U;
        p->as_conf=0x0300U; p->raw_start_timestamp_us=UINT32_MAX; p->raw_end_timestamp_us=7U;
        p->health_start_age_ms=65535U; p->health_end_age_ms=1U;
    }
    framing(&r,356U,312U); assert(diag_record_pack(&r,b,sizeof(b))==356U);
    const unsigned bases[3]={40U,144U,248U};
    for (unsigned i=0U;i<3U;++i) {
        const uint8_t *p=b+bases[i];
        assert(p[0]==(i==0U?0U:1U) && p[1]==i+1U && le16(p+2)==0x3FFFU);
        assert(le32(p+4)==0x80000000U && le32(p+8)==0x7FFFFFFFU);
        assert(le32(p+24)==UINT32_MAX && le32(p+28)==UINT32_MAX);
        assert(le32(p+32)==112500U && le32(p+36)==(uint32_t)-12345);
        assert(le16(p+40)==1023U && le16(p+42)==0U);
        assert(le16(p+50)==4095U && le16(p+52)==0U);
        assert(p[54]==32U && p[55]==32U && p[56]==64U && le16(p+58)==768U);
        assert(le32(p+60)==0x80000000U && le32(p+64)==0x7FFFFFFFU);
        assert(le32(p+92)==UINT32_MAX && le32(p+96)==7U);
        assert(le16(p+100)==65535U && le16(p+102)==1U);
    }
    memset(&r.data,0,sizeof(r.data)); r.type=DIAG_HOME_RESULT;
    r.data.home[0]=(diag_home_t){.axis=0U,.result=1U,.sg_threshold=10U,
        .seek_emitted_steps=INT32_MIN,.backoff_steps=UINT32_MAX,.latch_emitted_steps=-1,
        .sg_baseline=1023U,.sg_trigger=0U,.mscnt_zero=0U,.raw_zero=4095U,
        .home_offset_counts=INT32_MIN,.home_emitted_origin_steps=INT32_MAX,
        .home_duration_us=UINT32_MAX,.axis_flags=0x203CU,.as_status=32U,.agc=64U};
    r.data.home[1].axis=1U; r.data.home[1].result=4U;
    framing(&r,124U,80U); assert(diag_record_pack(&r,b,sizeof(b))==124U);
    assert(b[42]==0U && b[43]==10U && le32(b+44)==0x80000000U && le32(b+48)==UINT32_MAX);
    assert(le32(b+64)==0x80000000U && le32(b+68)==0x7FFFFFFFU);
    assert(b[80]==1U && b[81]==4U && b[82]==0U);
    memset(&r.data,0,sizeof(r.data)); r.type=DIAG_STATUS;
    r.data.status=(diag_status_t){.state=4U,.command_queue_depth=4U,.homed_mask=0U,
        .rx_crc_errors=UINT32_MAX,.rx_overflows=0x12345678U,
        .x_position_steps=INT32_MIN,.z_position_steps=INT32_MAX,
        .x_unwrap_counts=-1,.z_unwrap_counts=1,.x_home_offset_counts=INT32_MIN,
        .z_home_offset_counts=INT32_MAX,.x_health_age_ms=65535U,.z_health_age_ms=0U};
    framing(&r,116U,72U); assert(diag_record_pack(&r,b,sizeof(b))==116U);
    assert(b[40]==4U && b[41]==4U && le32(b+44)==UINT32_MAX);
    assert(le32(b+52)==0x12345678U && le32(b+72)==0x80000000U && le32(b+76)==0x7FFFFFFFU);
    assert(le32(b+80)==UINT32_MAX && le32(b+84)==1U && le16(b+108)==65535U);
    r.type=0U; assert(diag_record_pack(&r,b,sizeof(b))==0U);
    return 0;
}
