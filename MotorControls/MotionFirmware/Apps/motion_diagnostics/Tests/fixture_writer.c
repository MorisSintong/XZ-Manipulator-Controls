#include "diag_record.h"
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static void fields(const diag_record_t *r)
{
    printf("\"version\":1,\"record_type\":%u,\"payload_len\":%u,"
           "\"record_seq\":%lu,\"timestamp_ms\":%lu,\"command_seq\":%lu,"
           "\"home_epoch\":%lu,\"status_bits\":%lu,\"obj_id\":%u,"
           "\"class_id\":%u,\"command_type\":%u,\"rx_x_01mm\":%d,"
           "\"rx_y_01mm\":%d,\"rx_angle_01deg\":%u,\"rx_corr_01deg\":%d,"
           "\"phase_count\":%u,\"config_id\":%u",
           r->type,r->type==DIAG_COMMAND_RESULT?312U:r->type==DIAG_HOME_RESULT?80U:72U,
           (unsigned long)r->record_seq,(unsigned long)r->timestamp_ms,
           (unsigned long)r->command_seq,(unsigned long)r->home_epoch,
           (unsigned long)r->status_bits,r->command.obj_id,r->command.class_id,r->command.type,
           r->command.x_01mm,r->command.y_01mm,r->command.angle_01deg,r->command.corr_01deg,
           r->type==DIAG_COMMAND_RESULT?r->phase_count:0U,r->config_id);
#define JSON_u8(n,o) printf("%s\"" #n "\":%u",first?"":",",s->n);first=false;
#define JSON_u16(n,o) printf("%s\"" #n "\":%u",first?"":",",s->n);first=false;
#define JSON_u32(n,o) printf("%s\"" #n "\":%lu",first?"":",",(unsigned long)s->n);first=false;
#define JSON_i32(n,o) printf("%s\"" #n "\":%ld",first?"":",",(long)s->n);first=false;
#define JSON(t,n,o) JSON_##t(n,o)
    if (r->type==DIAG_COMMAND_RESULT) {
        printf(",\"phases\":[");
        for (unsigned i=0U;i<3U;++i) {
            const diag_phase_t *s=&r->data.phases[i];
            bool first=true;
            printf("%s{",i==0U?"":","); DIAG_PHASE_FIELDS(JSON) printf("}");
        }
        printf("]");
    } else if (r->type==DIAG_HOME_RESULT) {
        printf(",\"home\":[");
        for (unsigned i=0U;i<2U;++i) {
            const diag_home_t *s=&r->data.home[i];
            bool first=true;
            printf("%s{",i==0U?"":","); DIAG_HOME_FIELDS(JSON) printf(",\"reserved\":0}");
        }
        printf("]");
    } else {
        const diag_status_t *s=&r->data.status;
        bool first=true;
        printf(",\"status\":{"); DIAG_STATUS_FIELDS(JSON) printf("}");
    }
#undef JSON
}
static void emit(const char *name, const diag_record_t *r, bool comma)
{
    uint8_t bytes[DIAG_MAX_FRAME];
    const size_t n=diag_record_pack(r,bytes,sizeof(bytes));
    printf("%s{\"name\":\"%s\",\"type\":%u,\"hex\":\"",comma?",\n":"",name,r->type);
    for (size_t i=0U;i<n;++i) { printf("%02x",bytes[i]); }
    printf("\",\"fields\":{"); fields(r); printf("}}");
}
int main(int argc, char **argv)
{
    if (argc == 2 && freopen(argv[1], "wb", stdout) == NULL) {
        perror("fixture output"); return 1;
    }
    diag_record_t r={.type=DIAG_COMMAND_RESULT,.phase_count=3U,.config_id=1U,
        .record_seq=UINT32_MAX,.timestamp_ms=UINT32_MAX,.command_seq=0x7ED3U,
        .home_epoch=3U,.status_bits=DIAG_SUCCESS|DIAG_MSCNT_UNQUALIFIED,
        .command={1U,65535U,2U,INT16_MIN,INT16_MAX,3599U,-1800}};
#define SET_u8(n,o) s->n=(uint8_t)((o)+1U);
#define SET_u16(n,o) s->n=(uint16_t)(65535U-(o));
#define SET_u32(n,o) s->n=UINT32_MAX-(o);
#define SET_i32(n,o) s->n=((o)%8U==0U)?INT32_MIN:INT32_MAX;
#define SET(t,n,o) SET_##t(n,o)
    for (unsigned i=0U;i<3U;++i) {
        diag_phase_t *s=&r.data.phases[i]; DIAG_PHASE_FIELDS(SET)
        s->axis=(uint8_t)(i==0U?0U:1U);s->phase=(uint8_t)(i+1U);
        s->axis_flags=0x7FFFU;s->mscnt_start=1023U;s->mscnt_end=0U;
        s->mscnt_check=3U;s->driver_mode=4U;s->raw_start=4095U;s->raw_end=0U;
        s->as_conf=768U;s->as_status_start=32U;s->as_status_end=32U;
    }
    printf("[\n"); emit("command_extremes",&r,false);
    memset(&r.data,0,sizeof(r.data));r.type=DIAG_HOME_RESULT;r.phase_count=0U;
    for (unsigned i=0U;i<2U;++i) {
        diag_home_t *s=&r.data.home[i];DIAG_HOME_FIELDS(SET)
        s->axis=(uint8_t)i;s->result=i==0U?1U:4U;s->raw_zero=4095U;
        s->mscnt_zero=1023U;s->axis_flags=0x7FFFU;
    }
    emit("home_extremes",&r,true);
    memset(&r.data,0,sizeof(r.data));r.type=DIAG_STATUS;
    {diag_status_t *s=&r.data.status;DIAG_STATUS_FIELDS(SET)
     s->state=4U;s->command_queue_depth=4U;s->tx_queue_depth=8U;s->homed_mask=0U;}
    emit("status_extremes",&r,true);
    memset(&r.data,0,sizeof(r.data));r.type=DIAG_COMMAND_RESULT;r.phase_count=2U;
    r.status_bits=DIAG_ABORTED|DIAG_POSITION_UNCERTAIN;r.record_seq=4U;
    r.data.phases[0]=(diag_phase_t){.axis=0U,.phase=1U,.axis_flags=PH_BEGUN|PH_COMPLETE,
        .requested_target_01mm=100,.applied_target_01mm=100,.target_position_steps=800,
        .commanded_delta_steps=800,.emitted_delta_steps=800,.emitted_edge_count=800U};
    r.data.phases[1]=(diag_phase_t){.axis=1U,.phase=2U,.axis_flags=PH_BEGUN,
        .requested_target_01mm=20,.applied_target_01mm=20,.target_position_steps=800,
        .commanded_delta_steps=800,.emitted_delta_steps=123,.emitted_edge_count=123U};
    r.data.phases[2]=(diag_phase_t){.axis=1U,.phase=3U,.target_position_steps=0};
    emit("command_aborted",&r,true);
    r.status_bits=DIAG_CANCELLED|DIAG_NOT_HOMED;r.phase_count=0U;r.record_seq=5U;
    for (unsigned i=0U;i<3U;++i) {
        r.data.phases[i].axis_flags=0U;r.data.phases[i].commanded_delta_steps=0;
        r.data.phases[i].emitted_delta_steps=0;r.data.phases[i].emitted_edge_count=0U;
    }
    emit("command_cancelled",&r,true);printf("\n]\n");
    return 0;
}
