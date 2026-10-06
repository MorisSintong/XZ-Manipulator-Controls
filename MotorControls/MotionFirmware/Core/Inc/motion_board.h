#ifndef MOTION_BOARD_H
#define MOTION_BOARD_H
#include <stdint.h>
void motion_board_init(void);
void motion_board_poll(void);
void motion_board_timer_irq(uint8_t motor);
#endif
