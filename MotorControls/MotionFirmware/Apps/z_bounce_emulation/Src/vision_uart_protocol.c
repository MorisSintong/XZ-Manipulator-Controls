/**
  ******************************************************************************
  * @file    vision_uart_protocol.c
  * @brief   Implementation of Vision UART Protocol Handler
  *          Features:
  *          - CRC16-CCITT (poly 0x1021, init 0xFFFF)
  *          - ISR-Safe Ring Buffer (256 bytes)
  *          - Non-blocking Packet Framing State Machine
  *          - Binary Framing (18 bytes) & Legacy ASCII Command Parsing
  *          - Target State Storage & Response Generation
  ******************************************************************************
  */

#include "vision_uart_protocol.h"
#include "SEGGER_RTT.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/* Internal State Machine States */
typedef enum {
    PARSE_STATE_IDLE = 0,
    PARSE_STATE_BINARY_HEADER,
    PARSE_STATE_BINARY_PAYLOAD,
    PARSE_STATE_ASCII_LINE
} ParseState_t;

/* UART Driver Instance */
static UART_HandleTypeDef *s_huart = NULL;

/* ISR-Safe Ring Buffer (Single Producer ISR, Single Consumer Main Thread) */
static volatile uint8_t  s_rx_ring[VISION_RING_BUFFER_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

/* Parsing State Variables */
static ParseState_t s_parse_state = PARSE_STATE_IDLE;
static uint8_t      s_bin_buf[VISION_FRAME_TOTAL_LEN];
static uint8_t      s_bin_len = 0U;

#define ASCII_CMD_MAX_LEN 64U
static char         s_ascii_buf[ASCII_CMD_MAX_LEN];
static uint8_t      s_ascii_len = 0U;
static uint32_t     s_last_rx_tick = 0U;

#define PARSER_TIMEOUT_MS 300U

/* Latest Received Vision Target */
static VisionTarget_t s_latest_target = {0};

/* Forward Declarations of Private Functions */
static void send_reply(const char *msg);
static void handle_binary_packet(const uint8_t *packet);
static void handle_ascii_command(const char *line);
static bool parse_legacy_ascii(const char *str, int *out_cls, float *out_angle, float *out_corr);

/* ---------------------------------------------------------------------------*/
/* CRC16-CCITT Calculation                                                    */
/* Polynomial: 0x1021 (X^16 + X^12 + X^5 + 1), Init: 0xFFFF                   */
/* Matches Python crc16_ccitt() implementation                                */
/* ---------------------------------------------------------------------------*/
uint16_t VisionUart_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < length; i++)
    {
        crc ^= ((uint16_t)data[i] << 8U);
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x8000U) != 0U)
            {
                crc = (uint16_t)((crc << 1U) ^ 0x1021U);
            }
            else
            {
                crc = (uint16_t)(crc << 1U);
            }
        }
    }

    return crc;
}

/* ---------------------------------------------------------------------------*/
/* Initialization                                                             */
/* ---------------------------------------------------------------------------*/
void VisionUart_Init(UART_HandleTypeDef *huart)
{
    s_huart = huart;
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_parse_state = PARSE_STATE_IDLE;
    s_bin_len = 0U;
    s_ascii_len = 0U;
    s_last_rx_tick = HAL_GetTick();

    memset((void *)&s_latest_target, 0, sizeof(s_latest_target));

    if (s_huart != NULL)
    {
        /* Enable UART RXNE (Receive Data register not empty) Interrupt */
        __HAL_UART_ENABLE_IT(s_huart, UART_IT_RXNE);
    }
}

/* ---------------------------------------------------------------------------*/
/* ISR Callback (Invoked from USART2_IRQHandler)                              */
/* ---------------------------------------------------------------------------*/
void VisionUart_RxByteCallback(uint8_t byte)
{
    uint16_t next_head = (uint16_t)((s_rx_head + 1U) % VISION_RING_BUFFER_SIZE);

    /* Push byte if ring buffer is not full */
    if (next_head != s_rx_tail)
    {
        s_rx_ring[s_rx_head] = byte;
        s_rx_head = next_head;
    }
    /* If buffer is full, drop byte to avoid corrupting unread data */
}

/* ---------------------------------------------------------------------------*/
/* Non-blocking Stream Parser (Invoked from main loop)                        */
/* ---------------------------------------------------------------------------*/
void VisionUart_Process(void)
{
    /* Timeout check: reset parser if incomplete frame stalled */
    if (s_parse_state != PARSE_STATE_IDLE)
    {
        if ((HAL_GetTick() - s_last_rx_tick) > PARSER_TIMEOUT_MS)
        {
            if ((s_parse_state == PARSE_STATE_BINARY_HEADER) ||
                (s_parse_state == PARSE_STATE_BINARY_PAYLOAD))
            {
                send_reply("NACK:FRAME_ERR\r\n");
            }
            s_parse_state = PARSE_STATE_IDLE;
            s_bin_len = 0U;
            s_ascii_len = 0U;
        }
    }

    /* Process all bytes currently in the ring buffer */
    while (s_rx_head != s_rx_tail)
    {
        uint8_t byte = s_rx_ring[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % VISION_RING_BUFFER_SIZE);
        s_last_rx_tick = HAL_GetTick();

        switch (s_parse_state)
        {
            case PARSE_STATE_IDLE:
                if (byte == VISION_FRAME_HEADER_BYTE0) /* 0xAA */
                {
                    s_bin_buf[0] = byte;
                    s_bin_len = 1U;
                    s_parse_state = PARSE_STATE_BINARY_HEADER;
                }
                else if ((byte == '\r') || (byte == '\n') || (byte == 0U))
                {
                    /* Ignore standalone CR, LF, and NULL delimiters */
                }
                else if (isprint((int)byte))
                {
                    s_ascii_buf[0] = (char)byte;
                    s_ascii_len = 1U;
                    s_parse_state = PARSE_STATE_ASCII_LINE;
                }
                break;

            case PARSE_STATE_BINARY_HEADER:
                if (byte == VISION_FRAME_HEADER_BYTE1) /* 0x55 */
                {
                    s_bin_buf[1] = byte;
                    s_bin_len = 2U;
                    s_parse_state = PARSE_STATE_BINARY_PAYLOAD;
                }
                else if (byte == VISION_FRAME_HEADER_BYTE0) /* Repeated 0xAA */
                {
                    s_bin_buf[0] = byte;
                    s_bin_len = 1U;
                }
                else
                {
                    /* Framing error: 0xAA followed by non-0x55 */
                    send_reply("NACK:FRAME_ERR\r\n");
                    s_parse_state = PARSE_STATE_IDLE;
                    s_bin_len = 0U;
                }
                break;

            case PARSE_STATE_BINARY_PAYLOAD:
                s_bin_buf[s_bin_len++] = byte;
                if (s_bin_len >= VISION_FRAME_TOTAL_LEN) /* 18 bytes received */
                {
                    handle_binary_packet(s_bin_buf);
                    s_parse_state = PARSE_STATE_IDLE;
                    s_bin_len = 0U;
                }
                break;

            case PARSE_STATE_ASCII_LINE:
                if ((byte == '\n') || (byte == '\r'))
                {
                    s_ascii_buf[s_ascii_len] = '\0';
                    if (s_ascii_len > 0U)
                    {
                        handle_ascii_command(s_ascii_buf);
                    }
                    s_parse_state = PARSE_STATE_IDLE;
                    s_ascii_len = 0U;
                }
                else if (byte == VISION_FRAME_HEADER_BYTE0) /* Interrupted by binary frame */
                {
                    s_bin_buf[0] = byte;
                    s_bin_len = 1U;
                    s_parse_state = PARSE_STATE_BINARY_HEADER;
                    s_ascii_len = 0U;
                }
                else
                {
                    if (s_ascii_len < (ASCII_CMD_MAX_LEN - 1U))
                    {
                        s_ascii_buf[s_ascii_len++] = (char)byte;
                    }
                    else
                    {
                        /* Buffer overflow, discard command */
                        s_parse_state = PARSE_STATE_IDLE;
                        s_ascii_len = 0U;
                    }
                }
                break;

            default:
                s_parse_state = PARSE_STATE_IDLE;
                s_bin_len = 0U;
                s_ascii_len = 0U;
                break;
        }
    }
}

/* ---------------------------------------------------------------------------*/
/* Binary Packet Verification and Handling                                    */
/* ---------------------------------------------------------------------------*/
static void handle_binary_packet(const uint8_t *packet)
{
    /* 1. Verify Tail: 0x0D ('\r'), 0x0A ('\n') */
    if ((packet[16] != VISION_FRAME_TAIL_BYTE0) || (packet[17] != VISION_FRAME_TAIL_BYTE1))
    {
        send_reply("NACK:FRAME_ERR\r\n");
        return;
    }

    /* 2. Verify CRC16 over Header + Payload (first 14 bytes) */
    uint16_t computed_crc = VisionUart_CRC16(packet, VISION_FRAME_CRC_DATA_LEN);
    uint16_t packet_crc   = (uint16_t)packet[14] | ((uint16_t)packet[15] << 8U);

    if (computed_crc != packet_crc)
    {
        send_reply("NACK:CRC_ERR\r\n");
        return;
    }

    /* 3. Unpack Message Type and Object ID */
    uint8_t  msg_type  = packet[2];
    uint16_t object_id = (uint16_t)packet[3] | ((uint16_t)packet[4] << 8U);

    if (msg_type == VISION_MSG_TYPE_DETECTION)
    {
        uint8_t class_id        = packet[5];
        int16_t x_mm_x10        = (int16_t)((uint16_t)packet[6]  | ((uint16_t)packet[7]  << 8U));
        int16_t y_mm_x10        = (int16_t)((uint16_t)packet[8]  | ((uint16_t)packet[9]  << 8U));
        int16_t angle_x10       = (int16_t)((uint16_t)packet[10] | ((uint16_t)packet[11] << 8U));
        int16_t servo_corr_x10  = (int16_t)((uint16_t)packet[12] | ((uint16_t)packet[13] << 8U));

        float x_mm      = (float)x_mm_x10 / 10.0f;
        float y_mm      = (float)y_mm_x10 / 10.0f;
        float angle     = (float)angle_x10 / 10.0f;
        float servo_cor = (float)servo_corr_x10 / 10.0f;

        /* Store into latest target state */
        s_latest_target.object_id      = object_id;
        s_latest_target.class_id       = class_id;
        s_latest_target.x_mm           = x_mm;
        s_latest_target.y_mm           = y_mm;
        s_latest_target.angle_deg      = angle;
        s_latest_target.servo_corr_deg = servo_cor;
        s_latest_target.timestamp_ms   = HAL_GetTick();
        s_latest_target.valid          = true;

        /* Reply ACK with object details */
        char reply[100];
        snprintf(reply, sizeof(reply),
                 "ACK:OBJ=%u,CLS=%u,X=%.1f,Y=%.1f,ANG=%.1f,SRV=%.1f,CRC=OK\r\n",
                 (unsigned int)object_id,
                 (unsigned int)class_id,
                 (double)x_mm,
                 (double)y_mm,
                 (double)angle,
                 (double)servo_cor);
        send_reply(reply);
    }
    else if (msg_type == VISION_MSG_TYPE_HEARTBEAT)
    {
        /* Reply ACK for Heartbeat */
        char reply[48];
        snprintf(reply, sizeof(reply),
                 "ACK:HEARTBEAT,SEQ=%u\r\n",
                 (unsigned int)object_id);
        send_reply(reply);
    }
    else
    {
        /* Unknown binary message type */
        send_reply("NACK:FRAME_ERR\r\n");
    }
}

/* ---------------------------------------------------------------------------*/
/* Legacy ASCII Command Handling                                              */
/* ---------------------------------------------------------------------------*/
static void handle_ascii_command(const char *line)
{
    if (strcmp(line, "PING") == 0)
    {
        send_reply("ACK:PONG\r\n");
    }
    else if (strcmp(line, "STATUS") == 0)
    {
        send_reply("ACK:STATUS,READY\r\n");
    }
    else
    {
        int   cls   = 0;
        float angle = 0.0f;
        float corr  = 0.0f;

        if (parse_legacy_ascii(line, &cls, &angle, &corr))
        {
            s_latest_target.object_id      = 0U;
            s_latest_target.class_id       = (uint8_t)cls;
            s_latest_target.x_mm           = 0.0f;
            s_latest_target.y_mm           = 0.0f;
            s_latest_target.angle_deg      = angle;
            s_latest_target.servo_corr_deg = corr;
            s_latest_target.timestamp_ms   = HAL_GetTick();
            s_latest_target.valid          = true;

            char reply[64];
            snprintf(reply, sizeof(reply),
                     "ACK:LEGACY,CLS=%d,CORR=%.1f\r\n",
                     cls,
                     (double)corr);
            send_reply(reply);
        }
        else
        {
            send_reply("NACK:FRAME_ERR\r\n");
        }
    }
}

/* ---------------------------------------------------------------------------*/
/* Parse Legacy ASCII: "K<class_id>,S<angle>,R<correction>"                   */
/* ---------------------------------------------------------------------------*/
static bool parse_legacy_ascii(const char *str, int *out_cls, float *out_angle, float *out_corr)
{
    if ((str == NULL) || (str[0] != 'K'))
    {
        return false;
    }

    char *endptr = NULL;
    long cls_val = strtol(str + 1, &endptr, 10);
    if ((endptr == (str + 1)) || (*endptr != ','))
    {
        return false;
    }

    if (endptr[1] != 'S')
    {
        return false;
    }

    char *angle_str = endptr + 2;
    float angle_val = strtof(angle_str, &endptr);
    if ((endptr == angle_str) || (*endptr != ','))
    {
        return false;
    }

    if (endptr[1] != 'R')
    {
        return false;
    }

    char *corr_str = endptr + 2;
    float corr_val = strtof(corr_str, &endptr);
    if (endptr == corr_str)
    {
        return false;
    }

    if (out_cls != NULL)
    {
        *out_cls = (int)cls_val;
    }
    if (out_angle != NULL)
    {
        *out_angle = angle_val;
    }
    if (out_corr != NULL)
    {
        *out_corr = corr_val;
    }

    return true;
}

/* ---------------------------------------------------------------------------*/
/* Transmit Reply String over USART2 & SEGGER RTT                             */
/* ---------------------------------------------------------------------------*/
static void send_reply(const char *msg)
{
    if ((s_huart != NULL) && (msg != NULL))
    {
        uint16_t len = (uint16_t)strlen(msg);
        HAL_UART_Transmit(s_huart, (const uint8_t *)msg, len, 100U);
        SEGGER_RTT_Write(0, msg, (unsigned int)len);
    }
}

/* ---------------------------------------------------------------------------*/
/* Access Latest Target State                                                 */
/* ---------------------------------------------------------------------------*/
bool VisionUart_GetLatestTarget(VisionTarget_t *out_target)
{
    if ((out_target != NULL) && s_latest_target.valid)
    {
        *out_target = s_latest_target;
        return true;
    }
    return false;
}

void VisionUart_ClearTarget(void)
{
    s_latest_target.valid = false;
}
