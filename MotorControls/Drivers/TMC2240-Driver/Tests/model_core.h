#ifndef TMC2240_MODEL_CORE_H
#define TMC2240_MODEL_CORE_H
#include "tmc2240_core.h"
#include "model_device.h"
typedef struct {
    uint16_t id;
    uint8_t bytes[8];
    size_t tx_length;
    size_t rx_length;
} CoreFrame;
extern CoreFrame core_frames[256];
extern unsigned core_frame_count;
extern unsigned core_fail_at;
extern bool core_fail_after_accept;
extern TMC2240Status core_failure;
extern TMC2240Status core_bus_failure;
extern TMC2240Status core_node_failure;
extern TMC2240BusType core_buses[4];
extern uint8_t core_nodes[4];
extern unsigned core_count;
extern int core_corrupt_bit;
extern uint8_t core_reply_reserved;
void core_model_reset(void);
void core_trace_reset(void);
#endif
