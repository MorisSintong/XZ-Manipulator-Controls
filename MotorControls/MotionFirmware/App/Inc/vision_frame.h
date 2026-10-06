#ifndef APP_VISION_FRAME_H
#define APP_VISION_FRAME_H

#include <stddef.h>
#include <stdint.h>

#define VISION_FRAME_SIZE 18U
#define VISION_TYPE_PICK 1U
#define VISION_TYPE_STATUS 2U
#define VISION_TYPE_ABORT 3U
#define VISION_TYPE_HOME 4U

typedef struct {
    uint8_t type;
    uint16_t obj_id;
    uint8_t class_id;
    int16_t x_01mm;
    int16_t y_01mm;
    uint16_t angle_01deg;
    int16_t corr_01deg;
} vision_cmd_t;

typedef enum {
    VISION_PARSE_NONE,
    VISION_PARSE_FRAME_OK,
    VISION_PARSE_CRC_ERR,
    VISION_PARSE_FORMAT_ERR
} vision_parse_result_t;

typedef struct {
    uint8_t bytes[VISION_FRAME_SIZE];
    size_t used;
    uint32_t frames_ok;
    uint32_t crc_errors;
    uint32_t format_errors;
    uint32_t resyncs;
} vision_parser_t;

/** Reset the caller-owned parser and all counters. */
void vision_parser_init(vision_parser_t *p);
/** Feed one byte; out changes only on FRAME_OK. CRC covers bytes [0..13],
 * including preamble, matching the deployed Python encoder, before field validation.
 * Angle wire bits are signed int16, accepted only in [0,3599].
 * Resyncs count abandoned candidates, including unmatched first preambles.
 * No timeout is assumed: truncated candidates recover by retaining header suffixes.
 */
vision_parse_result_t vision_parser_feed(vision_parser_t *p, uint8_t byte,
                                        vision_cmd_t *out);
/** Encode a validated command; return 18, or zero without writing on invalid input.
 * CRC covers bytes [0..13]; angle is a nonnegative signed-int16 wire value.
 * Correction is transported as signed int16; mechanical limits belong to dispatch.
 */
size_t vision_frame_encode(const vision_cmd_t *cmd,
                           uint8_t out[VISION_FRAME_SIZE]);

#endif
