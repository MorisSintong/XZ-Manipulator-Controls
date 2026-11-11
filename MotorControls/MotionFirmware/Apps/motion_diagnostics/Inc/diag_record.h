#ifndef DIAG_RECORD_H
#define DIAG_RECORD_H
#include "vision_frame.h"
#include "diag_record_fields.h"
#define DIAG_COMMAND_RESULT 0x81U
#define DIAG_HOME_RESULT 0x82U
#define DIAG_STATUS 0x83U
#define DIAG_MAX_FRAME 356U

enum {
    DIAG_SUCCESS=1U, DIAG_ABORTED=2U, DIAG_CANCELLED=4U, DIAG_REJECTED=8U,
    DIAG_CLAMPED=16U, DIAG_NOT_HOMED=32U, DIAG_DRIVER_FAULT=64U,
    DIAG_ENCODER_INVALID=128U, DIAG_SAMPLE_OVERRUN=256U,
    DIAG_STEP_TIMING_FAULT=512U, DIAG_MSCNT_MISMATCH=1024U,
    DIAG_MSCNT_UNQUALIFIED=2048U, DIAG_RX_OVERFLOW=4096U,
    DIAG_TX_BACKPRESSURE=8192U, DIAG_HOME_FAILED=16384U,
    DIAG_LIMIT_FAULT=32768U, DIAG_CONFIG_INVALID=65536U,
    DIAG_POSITION_UNCERTAIN=131072U
};
enum {
    PH_BEGUN=1U, PH_COMPLETE=2U, PH_START_ENC=4U, PH_END_ENC=8U,
    PH_CONTINUOUS=16U, PH_HOME=32U, PH_START_MSCNT=64U, PH_END_MSCNT=128U,
    PH_CLAMPED=256U, PH_OVERRUN=512U, PH_MAGNET=1024U, PH_I2C=2048U,
    PH_ENC_NEG=4096U, PH_MSCNT_NEG=8192U, PH_CONVERSION=16384U
};
typedef uint8_t diag_u8;
typedef uint16_t diag_u16;
typedef uint32_t diag_u32;
typedef int32_t diag_i32;
#define DECLARE_FIELD(t,n,o) diag_##t n;
typedef struct { DIAG_PHASE_FIELDS(DECLARE_FIELD) } diag_phase_t;
typedef struct { DIAG_HOME_FIELDS(DECLARE_FIELD) } diag_home_t;
typedef struct { DIAG_STATUS_FIELDS(DECLARE_FIELD) } diag_status_t;
#undef DECLARE_FIELD
typedef struct {
    uint8_t type, phase_count, config_id;
    uint32_t record_seq, timestamp_ms, command_seq, home_epoch, status_bits;
    vision_cmd_t command;
    union {
        diag_phase_t phases[3];
        diag_home_t home[2];
        diag_status_t status;
    } data;
} diag_record_t;
size_t diag_record_pack(const diag_record_t *r, uint8_t *out, size_t capacity);
#endif
