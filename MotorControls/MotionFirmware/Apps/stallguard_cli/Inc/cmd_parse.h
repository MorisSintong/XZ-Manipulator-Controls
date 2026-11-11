/**
 * @file    cmd_parse.h
 * @brief   Line assembly and tokenising for the ASCII command console
 *          (pure C, host testable).
 */
#ifndef CMD_PARSE_H
#define CMD_PARSE_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CMD_LINE_MAX 96
#define CMD_ARGV_MAX 6

typedef struct {
    char     buf[CMD_LINE_MAX];
    uint16_t len;
    bool     overflow;
    bool     ready;
} cmd_linebuf_t;

void cmd_linebuf_init(cmd_linebuf_t *lb);
/** Push one received character. Returns true when a complete, non-empty line
 *  is available in lb->buf (NUL terminated). Lines longer than CMD_LINE_MAX-1
 *  are discarded. CR, LF and CRLF terminate a line; backspace edits. */
bool cmd_linebuf_push(cmd_linebuf_t *lb, char c);

/** Split 'line' in place on spaces, tabs, commas and '='; lower-cases the
 *  tokens. Returns the number of tokens stored in argv (<= max_args). */
int  cmd_tokenize(char *line, char *argv[], int max_args);
/** Strict decimal parser: [+-]digits[.digits]. No exponent. */
bool cmd_parse_float(const char *s, float *out);
/** Strict unsigned decimal parser (fits in uint32). */
bool cmd_parse_u32(const char *s, uint32_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CMD_PARSE_H */
