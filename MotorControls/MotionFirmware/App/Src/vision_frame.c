#include "vision_frame.h"
#include "crc16_ccitt.h"
#include <string.h>

static uint16_t read_u16(const uint8_t *b)
{
    return (uint16_t)((uint16_t)b[0] | (uint16_t)((uint16_t)b[1] << 8U));
}

static int16_t read_i16(const uint8_t *b)
{
    const uint16_t u = read_u16(b);
    const int32_t value = (u <= INT16_MAX) ? (int32_t)u : (int32_t)u - 65536;
    return (int16_t)value;
}

static void write_u16(uint8_t *b, uint16_t value)
{
    b[0] = (uint8_t)(value & UINT16_C(0xFF));
    b[1] = (uint8_t)(value >> 8U);
}

static int valid(const vision_cmd_t *cmd)
{
    return (cmd->type >= VISION_TYPE_PICK) && (cmd->type <= VISION_TYPE_HOME)
        && (cmd->class_id <= 2U) && (cmd->angle_01deg < 3600U);
}

size_t vision_frame_encode(const vision_cmd_t *cmd, uint8_t out[VISION_FRAME_SIZE])
{
    if ((cmd == NULL) || (out == NULL) || !valid(cmd)) {
        return 0U;
    }
    out[0] = 0xAAU;
    out[1] = 0x55U;
    out[2] = cmd->type;
    write_u16(&out[3], cmd->obj_id);
    out[5] = cmd->class_id;
    write_u16(&out[6], (uint16_t)cmd->x_01mm);
    write_u16(&out[8], (uint16_t)cmd->y_01mm);
    write_u16(&out[10], cmd->angle_01deg);
    write_u16(&out[12], (uint16_t)cmd->corr_01deg);
    write_u16(&out[14], crc16_ccitt(out, 14U));
    out[16] = 0x0DU;
    out[17] = 0x0AU;
    return VISION_FRAME_SIZE;
}

void vision_parser_init(vision_parser_t *p)
{
    *p = (vision_parser_t){0};
}

static void recover(vision_parser_t *p)
{
    ++p->resyncs;
    for (size_t i = 1U; i + 1U < p->used; ++i) {
        if ((p->bytes[i] == 0xAAU) && (p->bytes[i + 1U] == 0x55U)) {
            p->used -= i;
            memmove(p->bytes, &p->bytes[i], p->used);
            return;
        }
    }
    if (p->bytes[p->used - 1U] == 0xAAU) {
        p->bytes[0] = 0xAAU;
        p->used = 1U;
    } else {
        p->used = 0U;
    }
}

vision_parse_result_t vision_parser_feed(vision_parser_t *p, uint8_t byte,
                                        vision_cmd_t *out)
{
    if (p->used == 0U) {
        if (byte == 0xAAU) {
            p->bytes[0] = byte;
            p->used = 1U;
        }
        return VISION_PARSE_NONE;
    }
    if ((p->used == 1U) && (byte != 0x55U)) {
        ++p->resyncs;
        p->used = (byte == 0xAAU) ? 1U : 0U;
        return VISION_PARSE_NONE;
    }
    p->bytes[p->used] = byte;
    ++p->used;
    if (p->used < VISION_FRAME_SIZE) {
        return VISION_PARSE_NONE;
    }
    vision_parse_result_t result;
    vision_cmd_t cmd;
    if ((p->bytes[16] != 0x0DU) || (p->bytes[17] != 0x0AU)) {
        result = VISION_PARSE_FORMAT_ERR;
    } else if (read_u16(&p->bytes[14]) != crc16_ccitt(p->bytes, 14U)) {
        result = VISION_PARSE_CRC_ERR;
    } else {
        cmd.type = p->bytes[2];
        cmd.obj_id = read_u16(&p->bytes[3]);
        cmd.class_id = p->bytes[5];
        cmd.x_01mm = read_i16(&p->bytes[6]);
        cmd.y_01mm = read_i16(&p->bytes[8]);
        const int16_t angle = read_i16(&p->bytes[10]);
        cmd.angle_01deg = (uint16_t)angle;
        cmd.corr_01deg = read_i16(&p->bytes[12]);
        result = (valid(&cmd) && (angle >= 0))
            ? VISION_PARSE_FRAME_OK : VISION_PARSE_FORMAT_ERR;
    }
    if (result == VISION_PARSE_FRAME_OK) {
        *out = cmd;
        ++p->frames_ok;
        p->used = 0U;
    } else {
        if (result == VISION_PARSE_CRC_ERR) {
            ++p->crc_errors;
        } else {
            ++p->format_errors;
        }
        recover(p);
    }
    return result;
}
