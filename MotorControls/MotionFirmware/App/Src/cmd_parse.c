/**
 * @file    cmd_parse.c
 * @brief   Line assembly and tokenising for the ASCII command console.
 */
#include "cmd_parse.h"

#include <stddef.h>

void cmd_linebuf_init(cmd_linebuf_t *lb)
{
    lb->len = 0U;
    lb->overflow = false;
    lb->ready = false;
    lb->buf[0] = '\0';
}

bool cmd_linebuf_push(cmd_linebuf_t *lb, char c)
{
    if (lb->ready) {
        cmd_linebuf_init(lb);
    }
    if ((c == '\r') || (c == '\n')) {
        if (lb->overflow) {
            cmd_linebuf_init(lb);
            return false;
        }
        if (lb->len == 0U) {
            return false; /* empty line or second half of CRLF */
        }
        lb->buf[lb->len] = '\0';
        lb->ready = true;
        return true;
    }
    if ((c == '\b') || (c == (char)0x7F)) {
        if (lb->len > 0U) {
            lb->len--;
        }
        return false;
    }
    if (((unsigned char)c < 0x20U) || ((unsigned char)c > 0x7EU)) {
        return false; /* ignore other control and non-ASCII bytes */
    }
    if (lb->len >= (CMD_LINE_MAX - 1U)) {
        lb->overflow = true;
        return false;
    }
    lb->buf[lb->len++] = c;
    return false;
}

static bool is_sep(char c)
{
    return (c == ' ') || (c == '\t') || (c == ',') || (c == '=');
}

int cmd_tokenize(char *line, char *argv[], int max_args)
{
    int argc = 0;
    char *p = line;

    while ((*p != '\0') && (argc < max_args)) {
        while (is_sep(*p)) {
            *p++ = '\0';
        }
        if (*p == '\0') {
            break;
        }
        argv[argc++] = p;
        while ((*p != '\0') && !is_sep(*p)) {
            if ((*p >= 'A') && (*p <= 'Z')) {
                *p = (char)(*p - 'A' + 'a');
            }
            p++;
        }
    }
    /* Terminate the last token if we stopped because argv was full. */
    while ((*p != '\0') && !is_sep(*p)) {
        p++;
    }
    if (*p != '\0') {
        *p = '\0';
    }
    return argc;
}

bool cmd_parse_float(const char *s, float *out)
{
    float sign = 1.0f;
    float value = 0.0f;
    float scale = 0.1f;
    bool digits = false;

    if ((s == NULL) || (out == NULL)) {
        return false;
    }
    if ((*s == '+') || (*s == '-')) {
        if (*s == '-') {
            sign = -1.0f;
        }
        s++;
    }
    while ((*s >= '0') && (*s <= '9')) {
        value = value * 10.0f + (float)(*s - '0');
        digits = true;
        s++;
    }
    if (*s == '.') {
        s++;
        while ((*s >= '0') && (*s <= '9')) {
            value += (float)(*s - '0') * scale;
            scale *= 0.1f;
            digits = true;
            s++;
        }
    }
    if (!digits || (*s != '\0')) {
        return false;
    }
    *out = sign * value;
    return true;
}

bool cmd_parse_u32(const char *s, uint32_t *out)
{
    uint32_t value = 0U;

    if ((s == NULL) || (out == NULL) || (*s == '\0')) {
        return false;
    }
    while (*s != '\0') {
        uint32_t digit;
        if ((*s < '0') || (*s > '9')) {
            return false;
        }
        digit = (uint32_t)(*s - '0');
        if (value > (UINT32_MAX - digit) / 10U) {
            return false;
        }
        value = value * 10U + digit;
        s++;
    }
    *out = value;
    return true;
}
