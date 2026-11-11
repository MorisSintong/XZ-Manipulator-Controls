#ifndef MOTION_CONFIG_H
#define MOTION_CONFIG_H
#include <stdbool.h>
#include <stdint.h>

/* Wire axis IDs; hardware motor IDs are deliberately the reverse. */
#define AXIS_X 0U
#define AXIS_Z 1U
#define MOTION_AXES 2U
#define MOTION_QUEUE_SIZE 4U
#define MOTION_TX_SLOTS 8U

typedef struct {
    uint32_t lead_um, max_rate, acceleration;
    int32_t usable_max_01mm, home_budget_steps;
    uint32_t home_timeout_us, backoff_steps, clearance_steps;
    int8_t dir_sign, encoder_sign;
    uint8_t motor, sg_threshold;
} motion_axis_config_t;
typedef struct {
    motion_axis_config_t axis[2];
    int32_t safe_z_01mm, pick_depth_01mm;
    uint32_t settle_us, dwell_us, pulse_high_us, pulse_low_us;
    uint32_t dir_setup_us, dir_hold_us, lateness_us;
    bool bounds_confirmed, clamp_x, mscnt_qualified;
    uint8_t id;
} motion_config_t;
extern const motion_config_t motion_default_config;
bool motion_config_valid(const motion_config_t *c);
bool motion_target_steps(const motion_axis_config_t *a, int32_t target_01mm,
                         int32_t *steps, int32_t *residual_nm);
#endif
