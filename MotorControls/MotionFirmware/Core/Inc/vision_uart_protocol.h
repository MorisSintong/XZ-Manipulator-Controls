/**
  ******************************************************************************
  * @file    vision_uart_protocol.h
  * @brief   UART Packet Framing, Ring Buffer, and CRC16-CCITT Protocol Handler
  *          Target: STM32F446RE (USART2 @ 115200 8N1)
  *          Integrates with Python Vision System (Real-time YOLO + QC)
  ******************************************************************************
  */

#ifndef VISION_UART_PROTOCOL_H
#define VISION_UART_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Message Type Definitions */
#define VISION_MSG_TYPE_DETECTION   0x01U  /* Object detection & pick coordinates */
#define VISION_MSG_TYPE_HEARTBEAT   0x02U  /* Heartbeat / keepalive packet */

/* Frame Delimiters & Lengths */
#define VISION_FRAME_HEADER_BYTE0   0xAAU
#define VISION_FRAME_HEADER_BYTE1   0x55U
#define VISION_FRAME_TAIL_BYTE0     0x0DU  /* '\r' */
#define VISION_FRAME_TAIL_BYTE1     0x0AU  /* '\n' */

#define VISION_FRAME_TOTAL_LEN      18U    /* Total binary packet size */
#define VISION_FRAME_CRC_DATA_LEN   14U    /* Header (2) + Payload (12) */
#define VISION_RING_BUFFER_SIZE     256U   /* ISR-safe Ring Buffer Size (power of 2) */

/* Class ID Definitions */
typedef enum {
    VISION_CLASS_NON_ELCO = 0,             /* Non-Electrolytic Capacitor */
    VISION_CLASS_BENAR    = 1,             /* Correct Polarity (PASS) */
    VISION_CLASS_SALAH    = 2              /* Inverted Polarity (FAIL) */
} VisionClass_t;

/* Latest Vision Target Data Structure */
typedef struct {
    uint16_t object_id;                    /* Unique object tracking ID */
    uint8_t  class_id;                     /* 0=Non-Elco, 1=PASS, 2=FAIL */
    float    x_mm;                         /* Conveyor X coordinate in mm */
    float    y_mm;                         /* Conveyor Y coordinate in mm */
    float    angle_deg;                    /* Detected orientation angle [0, 360) */
    float    servo_corr_deg;               /* Required servo correction [0, 180] */
    uint32_t timestamp_ms;                 /* Timestamp of last valid reception */
    bool     valid;                        /* Indicates if new target is available */
} VisionTarget_t;

/* Public Function Prototypes */
void     VisionUart_Init(UART_HandleTypeDef *huart);
void     VisionUart_RxByteCallback(uint8_t byte);
void     VisionUart_Process(void);
uint16_t VisionUart_CRC16(const uint8_t *data, uint16_t length);
bool     VisionUart_GetLatestTarget(VisionTarget_t *out_target);
void     VisionUart_ClearTarget(void);

#ifdef __cplusplus
}
#endif

#endif /* VISION_UART_PROTOCOL_H */
