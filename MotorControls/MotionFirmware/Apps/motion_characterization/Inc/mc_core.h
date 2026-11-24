/**
 * @file    mc_core.h
 * @brief   Hardware-independent motion characterization state machine,
 *          wire protocol, safety interlocks, and trajectory validation.
 *
 * Host-testable with a mock HAL seam (mc_hal_t) so every safety gate,
 * encoder check, lease timeout, B1 stop, duplicate-request guard, and
 * edge-inhibit path can be verified without hardware attached.
 */
#ifndef MC_CORE_H
#define MC_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cmd_parse.h"
#include "encoder_sampler.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MC_PROFILE_NAME        "motion_characterization"
#define MC_SCHEMA_VERSION      1U
#define MC_MAX_AXES            3U
#define MC_TX_RING_LINES       32U
#define MC_TX_LINE_MAX         384U
#define MC_LEASE_TIMEOUT_US    250000U   /* 250 ms missed host lease */
#define MC_ENC_POLL_US         2000U     /* 2 ms target encoder poll */
#define MC_ENC_HEALTH_US       20000U    /* 20 ms target health poll */
#define MC_DRV_POLL_US         50000U    /* 50 ms DRV_STATUS poll */
#define MC_ADC_POLL_US         250000U   /* 250 ms temp + VM poll */
#define MC_COMMISSION_MIN_CNT  64        /* >= 64 counts each direction */
#define MC_STATIC_SAMPLES      1024U
#define MC_RATE_WINDOW_OPS     1024U
#define MC_RATE_WINDOWS        3U
#define MC_REF_SAMPLES         32U
#define MC_THETA_MAX_MA        800U      /* Stage 2 bench limit for theta */

typedef enum {
    MC_AXIS_Z     = 0U, /* motor 0 / I2C4 */
    MC_AXIS_X     = 1U, /* motor 1 / I2C1 */
    MC_AXIS_THETA = 2U, /* motor 2 / I2C1 only in theta_fixture */
    MC_AXIS_NONE  = 0xFFU
} mc_axis_t;

typedef enum {
    MC_SESSION_XZ            = 0U,
    MC_SESSION_THETA_FIXTURE = 1U
} mc_session_mode_t;

typedef enum {
    MC_STATE_DISARMED = 0U,
    MC_STATE_ARMED    = 1U,
    MC_STATE_RUNNING  = 2U,
    MC_STATE_FAULT    = 3U
} mc_state_t;

typedef enum {
    MC_TEST_NONE = 0U,
    MC_TEST_COMMISSION,
    MC_TEST_ENCODER_RATE,
    MC_TEST_STATIC,
    MC_TEST_REFERENCE,
    MC_TEST_SPI_RATE,
    MC_TEST_JOG,
    MC_TEST_MOVE,
    MC_TEST_STRESS
} mc_test_kind_t;

typedef enum {
    MC_FAULT_NONE = 0U,
    MC_FAULT_HOST_STOP,
    MC_FAULT_B1_STOP,
    MC_FAULT_USB_DISCONNECT,
    MC_FAULT_LEASE_EXPIRED,
    MC_FAULT_SOFT_LIMIT,
    MC_FAULT_ENCODER_INVALID,
    MC_FAULT_ENCODER_STALE,
    MC_FAULT_ENCODER_GAP,
    MC_FAULT_FOLLOWING_ERROR,
    MC_FAULT_WRONG_DIRECTION,
    MC_FAULT_NO_PROGRESS,
    MC_FAULT_DRIVER_FAULT,
    MC_FAULT_DRIVER_RESET,
    MC_FAULT_DRIVER_SPI,
    MC_FAULT_TEMP_LIMIT,
    MC_FAULT_VM_OUT_OF_RANGE,
    MC_FAULT_TIMING_LATENESS,
    MC_FAULT_RING_OVERFLOW,
    MC_FAULT_RESTORE_FAILED
} mc_fault_t;

typedef struct {
    uint16_t run_current_ma;
    uint8_t  hold_pct;
    uint32_t vm_min_mv;
    uint32_t vm_max_mv;
    int16_t  temp_cutoff_c10;
    int32_t  win_neg_steps;    /* <= 0 */
    int32_t  win_pos_steps;    /* >= 0 */
    uint32_t max_speed_steps_s;
    uint32_t max_accel_steps_s2;
    uint32_t min_speed_steps_s;
    int8_t   encoder_sign;     /* +1 or -1 after commissioning */
    uint16_t static_noise_cnt; /* max(2, 3*SD) from commissioning */
    uint32_t follow_band_usteps;
    bool     commissioned;
    bool     datum_confirmed;
    bool     safety_confirmed;
    bool     configured;
    uint16_t applied_current_ma;
    uint8_t  applied_range;
    uint16_t applied_gs;
    uint8_t  applied_irun;
    uint8_t  applied_ihold;
    uint8_t  gstat_as_found;
} mc_axis_policy_t;

typedef struct {
    uint32_t edges_emitted;
    uint32_t edges_inhibited;
    uint32_t first_edge_us;
    uint32_t last_edge_us;
    uint32_t min_dt_us;
    uint32_t max_dt_us;
    uint32_t max_lateness_us;
    uint32_t late_edges;
    uint32_t ramp_up_end_us;
    uint32_t ramp_up_end_edges;
    uint32_t cruise_end_us;
    uint32_t cruise_end_edges;
    bool     lateness_fault;
} mc_step_stats_t;

typedef struct {
    uint32_t drv_status;
    uint32_t tstep;
    uint16_t sg4;
    uint16_t mscnt;
    uint8_t  spi_status;
    uint8_t  cs_actual;
    bool     stealth;
    bool     standstill;
    bool     otpw;
    bool     ot;
    bool     short_fault;
    bool     open_load;
    bool     reset_flag;
    bool     drv_err;
    bool     ok;
} mc_driver_snap_t;

typedef struct {
    int16_t  temp_c10;
    uint32_t vm_mv;
    bool     ok;
} mc_driver_adc_t;

typedef struct {
    uint16_t raw_angle;
    uint16_t angle;
    uint8_t  status;
    uint8_t  agc;
    uint16_t magnitude;
    uint8_t  err_code; /* 0 = OK */
} mc_enc_full_t;

typedef struct {
    uint32_t sysclk_hz;
    uint32_t tim2_hz;
    uint32_t tim5_hz;
    uint32_t spi_kernel_hz;
    uint32_t spi_prescaler;
    uint32_t i2c1_timingr;
    uint32_t i2c4_timingr;
    uint8_t  drv_version[MC_MAX_AXES];
    uint8_t  drv_gstat[MC_MAX_AXES];
} mc_board_info_t;

/** Hardware seam implemented by mc_bench.c on STM32 and by mocks in host CTest. */
typedef struct {
    void *ctx;
    uint32_t (*now_us)(void *ctx);
    uint32_t (*now_ms)(void *ctx);
    bool     (*usb_mounted)(void *ctx);
    uint32_t (*usb_write)(void *ctx, const uint8_t *data, uint32_t len);
    bool     (*b1_asserted)(void *ctx);
    void     (*get_board_info)(void *ctx, mc_board_info_t *out);

    /* Power stage ENN control: enable=false drives ENN HIGH (disabled). */
    void     (*set_driver_enable)(void *ctx, uint8_t axis, bool enable);
    void     (*disable_all_drivers)(void *ctx);

    /* Step generator seam (single active slot bound to selected axis). */
    bool     (*step_bind_axis)(void *ctx, uint8_t axis);
    void     (*step_set_inhibit)(void *ctx, bool inhibit);
    void     (*step_set_limits)(void *ctx, int32_t lo, int32_t hi);
    void     (*step_set_vmin)(void *ctx, float vmin);
    bool     (*step_set_position)(void *ctx, int32_t pos);
    int32_t  (*step_position)(void *ctx);
    bool     (*step_busy)(void *ctx);
    bool     (*step_validate_rate)(void *ctx, float vmax, float vmin);
    void     (*step_reset_stats)(void *ctx);
    void     (*step_get_stats)(void *ctx, mc_step_stats_t *out);
    void     (*step_move_to)(void *ctx, int32_t target, float vmax, float accel);
    void     (*step_halt)(void *ctx);
    bool     (*step_take_limit_hit)(void *ctx);

    /* Encoder seam (bus 0 = I2C4/Z, bus 1 = I2C1/X or theta_fixture). */
    bool     (*enc_configure)(void *ctx, uint8_t bus, uint16_t *conf_rb,
                              uint8_t *status, uint8_t *agc, uint16_t *mag,
                              uint16_t *raw);
    uint8_t  (*enc_read_raw)(void *ctx, uint8_t bus, uint16_t *raw);
    uint8_t  (*enc_read_health)(void *ctx, uint8_t bus, uint8_t *status,
                                uint8_t *agc, uint16_t *mag);
    uint8_t  (*enc_read_sample)(void *ctx, uint8_t bus, mc_enc_full_t *out);
    bool     (*enc_set_fast_mode)(void *ctx, uint8_t bus, bool fast,
                                  uint32_t *timingr_out);

    /* TMC2240 SPI seam. */
    bool     (*tmc_configure)(void *ctx, uint8_t axis, mc_axis_policy_t *policy);
    bool     (*tmc_poll_status)(void *ctx, uint8_t axis, mc_driver_snap_t *out);
    bool     (*tmc_poll_adc)(void *ctx, uint8_t axis, mc_driver_adc_t *out);
    bool     (*tmc_spi_set_prescaler)(void *ctx, uint32_t div, uint32_t *sck_hz);
    bool     (*tmc_spi_read_reg)(void *ctx, uint8_t axis, uint8_t reg,
                                 uint32_t *val);
} mc_hal_t;

typedef struct {
    char     lines[MC_TX_RING_LINES][MC_TX_LINE_MAX];
    uint16_t lens[MC_TX_RING_LINES];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
    uint16_t cursor;
    uint32_t overflow_count;
} mc_tx_ring_t;

typedef struct {
    mc_hal_t          hal;
    mc_state_t        state;
    mc_session_mode_t session_mode;
    mc_axis_t         armed_axis;
    mc_test_kind_t    active_test;
    mc_fault_t        fault;

    uint32_t          seq;
    uint32_t          last_request_id;
    bool              have_request_id;
    uint32_t          active_request_id;
    uint32_t          trial_id;
    uint32_t          last_keepalive_us;
    bool              lease_active;
    bool              b1_latched;

    mc_axis_policy_t  policy[MC_MAX_AXES];
    encoder_sample_t  enc_state[MC_MAX_AXES];

    /* Pending point parameters (updated via POINT between legs). */
    int32_t           point_target_steps;
    uint32_t          point_speed_steps_s;
    uint32_t          point_accel_steps_s2;
    bool              point_valid;

    /* Selector options set via SET before passive tests. */
    uint8_t           opt_enc_mode;     /* 0=raw, 1=sample */
    bool              opt_enc_fast;     /* bench-only Fast-mode trial */
    bool              opt_enc_fast_ok;  /* operator verified analyzer */
    uint32_t          opt_stress_s;     /* default 60 */
    bool              opt_stress_extra_spi;
    uint32_t          opt_stress_spi_div;

    /* Active motion trial supervision state. */
    int32_t           move_start_steps;
    int32_t           move_target_steps;
    int32_t           move_start_counts;
    int32_t           move_max_abs_err_usteps;
    uint32_t          move_start_us;
    uint32_t          move_last_enc_us;
    uint32_t          move_last_health_us;
    uint32_t          move_last_drv_us;
    uint32_t          move_last_adc_us;
    uint8_t           move_follow_strikes;
    uint8_t           move_spi_rr;
    bool              move_settling;
    uint32_t          move_settle_start_us;
    uint16_t          move_settle_samples;
    int32_t           move_settle_min_cnt;
    int32_t           move_settle_max_cnt;
    int64_t           move_settle_sum_cnt;

    /* Stress loop state. */
    uint32_t          stress_end_us;
    uint32_t          stress_legs_done;
    int32_t           stress_leg_a;
    int32_t           stress_leg_b;
    bool              stress_to_b;
    uint32_t          stress_extra_reads;
    uint32_t          stress_extra_errs;

    /* Line assembler & non-blocking TX ring. */
    cmd_linebuf_t     rx_line;
    mc_tx_ring_t      tx;
} mc_core_t;

void        mc_core_init(mc_core_t *mc, const mc_hal_t *hal);
void        mc_core_rx_bytes(mc_core_t *mc, const uint8_t *buf, uint32_t len);
void        mc_core_poll(mc_core_t *mc);
void        mc_core_interlock_tick_1ms(mc_core_t *mc);
void        mc_core_b1_irq(mc_core_t *mc);
void        mc_core_latch_fault(mc_core_t *mc, mc_fault_t fault, bool disable_stage);

/* Helpers exposed for unit testing. */
uint8_t     mc_axis_to_enc_bus(mc_session_mode_t mode, mc_axis_t axis);
bool        mc_axis_motion_allowed(mc_session_mode_t mode, mc_axis_t axis);
bool        mc_validate_stroke_geometry(int32_t start_steps, int32_t target_steps,
                                        uint32_t v_target, uint32_t v_start,
                                        uint32_t accel, bool require_ramp);
const char *mc_fault_name(mc_fault_t f);
const char *mc_axis_name(mc_axis_t a);

#ifdef __cplusplus
}
#endif

#endif /* MC_CORE_H */
