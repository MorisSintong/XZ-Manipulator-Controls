/* Unit tests: StallGuard4 detector, encoder unwrap, TMC2240 math, console
 * line parsing. */
#include "test_common.h"

#include "cmd_parse.h"
#include "enc_track.h"
#include "stall_detect.h"
#include "tmc_calc.h"

#include <stdint.h>
#include <string.h>

static uint32_t g_rng = 42U;

static uint16_t sg_noise(uint16_t mean, uint16_t amp)
{
    g_rng = g_rng * 1103515245U + 12345U;
    return (uint16_t)(mean - amp + (uint16_t)((g_rng >> 16) % (2U * amp + 1U)));
}

static stall_cfg_t stall_cfg(void)
{
    stall_cfg_t c;

    c.blank_ms = 100U;
    c.baseline_samples = 16U;
    c.min_baseline = 40U;
    c.no_baseline_ms = 400U;
    c.abs_threshold = 20U;
    c.drop_ratio = 0.45f;
    c.confirm = 4U;
    c.ema_shift = 5U;
    return c;
}

/* ---- stall detector ---------------------------------------------------- */

static void test_stall_free_running_no_trigger(void)
{
    const stall_cfg_t cfg = stall_cfg();
    stall_det_t d;
    int32_t t;
    unsigned stalls = 0U;

    stall_det_reset(&d, &cfg);
    for (t = -200; t < 20000; t += 2) {
        if (stall_det_update(&d, t, sg_noise(300U, 60U), false) != STALL_NONE) {
            stalls++;
        }
    }
    CHECK(stalls == 0U);
    CHECK(d.baseline_valid);
    CHECK_NEAR(d.baseline, 300.0, 25.0);
    CHECK_NEAR(stall_det_hw_threshold(&d), 300.0 * 0.45 / 2.0, 8.0);
}

static void test_stall_drop_detected_after_confirm(void)
{
    const stall_cfg_t cfg = stall_cfg();
    stall_det_t d;
    int32_t t;
    int n;

    stall_det_reset(&d, &cfg);
    for (t = 100; t < 400; t += 2) {
        CHECK(stall_det_update(&d, t, sg_noise(300U, 30U), false) == STALL_NONE);
    }
    /* 3 low samples are not enough ... */
    for (n = 0; n < 3; n++) {
        CHECK(stall_det_update(&d, t, 100U, false) == STALL_NONE);
        t += 2;
    }
    CHECK(stall_det_update(&d, t, 300U, false) == STALL_NONE); /* ... dip ends */
    for (n = 0; n < 3; n++) {
        CHECK(stall_det_update(&d, t, 100U, false) == STALL_NONE);
        t += 2;
    }
    CHECK(stall_det_update(&d, t, 100U, false) == STALL_SG_DROP);
}

static void test_stall_blanking_and_flags(void)
{
    const stall_cfg_t cfg = stall_cfg();
    stall_det_t d;
    int n;

    stall_det_reset(&d, &cfg);
    for (n = 0; n < 50; n++) {
        CHECK(stall_det_update(&d, -1, 0U, true) == STALL_NONE); /* accelerating */
        CHECK(stall_det_update(&d, 99, 0U, true) == STALL_NONE); /* blanking */
    }
    for (n = 0; n < 3; n++) {
        CHECK(stall_det_update(&d, 100 + n, 250U, true) == STALL_NONE);
    }
    CHECK(stall_det_update(&d, 110, 250U, true) == STALL_HW_FLAG);

    stall_det_reset(&d, &cfg);
    for (n = 0; n < 3; n++) {
        CHECK(stall_det_update(&d, 200 + n, 10U, false) == STALL_NONE);
    }
    CHECK(stall_det_update(&d, 210, 10U, false) == STALL_SG_ABS); /* started blocked */
}

static void test_stall_no_baseline(void)
{
    const stall_cfg_t cfg = stall_cfg();
    stall_det_t d;
    int32_t t;
    stall_source_t s = STALL_NONE;

    stall_det_reset(&d, &cfg);
    for (t = 100; (t < 2000) && (s == STALL_NONE); t += 2) {
        s = stall_det_update(&d, t, sg_noise(30U, 5U), false); /* too low to trust */
    }
    CHECK(s == STALL_NO_BASELINE);
    CHECK_NEAR(t, 100 + 400, 4);
    CHECK(stall_det_hw_threshold(&d) == 0U);
}

static void test_stall_tracks_slow_drift(void)
{
    const stall_cfg_t cfg = stall_cfg();
    stall_det_t d;
    int i;
    unsigned stalls = 0U;

    stall_det_reset(&d, &cfg);
    for (i = 0; i < 3000; i++) {
        const uint16_t mean = (uint16_t)(300 - (i * 150) / 3000); /* friction build-up */
        if (stall_det_update(&d, 100 + 2 * i, sg_noise(mean, 20U), false) != STALL_NONE) {
            stalls++;
        }
    }
    CHECK(stalls == 0U);
    CHECK_NEAR(d.baseline, 150.0, 20.0);
}

/* ---- encoder unwrap ---------------------------------------------------- */

static void test_enc_unwrap(void)
{
    enc_track_t t;
    int i;
    uint16_t raw = 3000U;

    enc_track_reset(&t);
    CHECK(enc_track_update(&t, 4000U) == 4000);
    CHECK(enc_track_update(&t, 4090U) == 4090);
    CHECK(enc_track_update(&t, 10U) == 4106);   /* forward through 4095 -> 0 */
    CHECK(enc_track_update(&t, 4000U) == 4000); /* backward through 0 -> 4095 */
    CHECK(enc_track_delta(0U, 2047U) == 2047);
    CHECK(enc_track_delta(0U, 2048U) == -2048);
    CHECK(enc_track_delta(4095U, 0U) == 1);

    enc_track_reset(&t);
    (void)enc_track_update(&t, raw);
    for (i = 0; i < 400; i++) { /* 10 turns forward in 300-count steps (~0.07 rev) */
        raw = (uint16_t)((raw + 300U) % 4096U);
        (void)enc_track_update(&t, raw);
    }
    CHECK(t.counts == 3000 + 400 * 300);
    for (i = 0; i < 800; i++) { /* 20 turns backward */
        raw = (uint16_t)((raw + 4096U - 150U) % 4096U);
        (void)enc_track_update(&t, raw);
    }
    CHECK(t.counts == 3000 + 400 * 300 - 800 * 150);
}

/* ---- TMC2240 math ------------------------------------------------------ */

static void test_tmc_currents(void)
{
    tmc_current_t c;

    CHECK(tmc_calc_currents(12000U, 800U, 500U, 50U, &c));
    CHECK(c.range == 1U);   /* 2 A peak / 1.41 A RMS full scale */
    CHECK(c.gs == 145U);
    CHECK(c.irun == 31U);
    CHECK(c.ihold == 15U);
    CHECK(c.ihome == 19U);
    CHECK_NEAR(c.irun_ma, 800.0, 5.0);
    CHECK_NEAR(c.ihold_ma, 400.0, 15.0);
    CHECK_NEAR(c.ihome_ma, 500.0, 15.0);

    CHECK(tmc_calc_currents(12000U, 1000U, 600U, 50U, &c));
    CHECK(c.range == 1U && c.gs == 181U && c.irun == 31U);

    CHECK(tmc_calc_currents(12000U, 600U, 0U, 50U, &c));
    CHECK(c.range == 0U && c.gs == 222U && c.ihome == c.irun);

    CHECK(tmc_calc_currents(12000U, 2000U, 1000U, 30U, &c));
    CHECK(c.range == 2U && c.gs == 241U);

    CHECK(tmc_calc_currents(12000U, 20U, 10U, 50U, &c)); /* GS clamps to 32 */
    CHECK(c.gs == 32U && c.irun < 31U);
    CHECK_NEAR(c.irun_ma, 20.0, 3.0);

    CHECK(!tmc_calc_currents(12000U, 2500U, 0U, 50U, &c)); /* above 3 A peak */
    CHECK(!tmc_calc_currents(10000U, 800U, 0U, 50U, &c));  /* RREF out of range */
    CHECK(!tmc_calc_currents(12000U, 0U, 0U, 50U, &c));
    CHECK(tmc_calc_currents(24000U, 800U, 0U, 50U, &c));   /* larger RREF -> higher range */
    CHECK(c.range == 2U);
}

static void test_tmc_registers(void)
{
    CHECK(tmc_calc_tstep(4800.0f, 16U) == 163U);
    CHECK(tmc_calc_tstep(0.0f, 16U) == TMC_TSTEP_MAX);
    CHECK(tmc_calc_tstep(0.5f, 16U) == TMC_TSTEP_MAX);
    CHECK(tmc_calc_tstep(3200.0f, 256U) == 3906U);
    CHECK(tmc_calc_mres(256U) == 0U);
    CHECK(tmc_calc_mres(16U) == 4U);
    CHECK(tmc_calc_mres(1U) == 8U);
    CHECK(tmc_calc_mres(3U) == 0xFFU);
    CHECK(tmc_calc_mres(512U) == 0xFFU);
    CHECK(tmc_calc_chopconf(4U, true, true) == 0x34410150UL);
    CHECK(tmc_calc_chopconf(0U, false, false) == 0x00410150UL); /* datasheet reset minus intpol */
    CHECK(tmc_calc_gconf(false, true, true) == 0x0000008CUL);
    CHECK(tmc_calc_gconf(true, false, false) == 0x00000018UL);
    CHECK(tmc_calc_ihold_irun(15U, 31U, 6U, 4U) == 0x04061F0FUL);
    CHECK(tmc_calc_drv_conf(1U, 2U) == 0x21UL);
    CHECK(tmc_calc_coolconf(5U, 2U, 2U, 0U, false) == 0x245UL);
    CHECK(tmc_calc_coolconf(0U, 0U, 0U, 0U, true) == 0x8000UL);
    CHECK(tmc_calc_sg4_thrs(40U, true, true) == 0x328UL);
}

/* ---- console parsing ---------------------------------------------------- */

static bool push_str(cmd_linebuf_t *lb, const char *s)
{
    bool ready = false;

    while (*s != '\0') {
        ready = cmd_linebuf_push(lb, *s++);
    }
    return ready;
}

static void test_cmd_linebuf(void)
{
    cmd_linebuf_t lb;
    char longline[200];

    cmd_linebuf_init(&lb);
    CHECK(!push_str(&lb, "hom"));
    CHECK(push_str(&lb, "e\r"));
    CHECK(strcmp(lb.buf, "home") == 0);
    CHECK(!cmd_linebuf_push(&lb, '\n')); /* LF of CRLF is not an empty command */
    CHECK(push_str(&lb, "stpo\b\bop\n"));
    CHECK(strcmp(lb.buf, "stop") == 0);
    memset(longline, 'a', sizeof(longline) - 1U);
    longline[sizeof(longline) - 1U] = '\0';
    CHECK(!push_str(&lb, longline));
    CHECK(!cmd_linebuf_push(&lb, '\n')); /* overflow: discarded */
    CHECK(push_str(&lb, "\x01st\xFF" "atus\n"));
    CHECK(strcmp(lb.buf, "status") == 0);
}

static void test_cmd_tokenize_and_numbers(void)
{
    char line[64] = "  Move X=12.5, fast ";
    char *argv[CMD_ARGV_MAX];
    float f = 0.0f;
    uint32_t u = 0U;
    int argc = cmd_tokenize(line, argv, CMD_ARGV_MAX);

    CHECK(argc == 4);
    CHECK(strcmp(argv[0], "move") == 0);
    CHECK(strcmp(argv[1], "x") == 0);
    CHECK(strcmp(argv[2], "12.5") == 0);
    CHECK(strcmp(argv[3], "fast") == 0);

    strcpy(line, "a b c d e f g h");
    argc = cmd_tokenize(line, argv, 3);
    CHECK(argc == 3 && strcmp(argv[2], "c") == 0);

    CHECK(cmd_parse_float("12.5", &f) && fabsf(f - 12.5f) < 1e-5f);
    CHECK(cmd_parse_float("-3", &f) && fabsf(f + 3.0f) < 1e-6f);
    CHECK(cmd_parse_float("+.25", &f) && fabsf(f - 0.25f) < 1e-6f);
    CHECK(cmd_parse_float("5.", &f) && fabsf(f - 5.0f) < 1e-6f);
    CHECK(!cmd_parse_float("", &f));
    CHECK(!cmd_parse_float("+", &f));
    CHECK(!cmd_parse_float("1.2.3", &f));
    CHECK(!cmd_parse_float("12mm", &f));
    CHECK(cmd_parse_u32("4294967295", &u) && u == 4294967295U);
    CHECK(!cmd_parse_u32("4294967296", &u));
    CHECK(!cmd_parse_u32("-1", &u));
    CHECK(!cmd_parse_u32("", &u));
}

int main(void)
{
    RUN_TEST(test_stall_free_running_no_trigger);
    RUN_TEST(test_stall_drop_detected_after_confirm);
    RUN_TEST(test_stall_blanking_and_flags);
    RUN_TEST(test_stall_no_baseline);
    RUN_TEST(test_stall_tracks_slow_drift);
    RUN_TEST(test_enc_unwrap);
    RUN_TEST(test_tmc_currents);
    RUN_TEST(test_tmc_registers);
    RUN_TEST(test_cmd_linebuf);
    RUN_TEST(test_cmd_tokenize_and_numbers);
    return TEST_SUMMARY();
}
