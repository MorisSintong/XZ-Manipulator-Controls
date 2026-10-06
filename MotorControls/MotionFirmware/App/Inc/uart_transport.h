#ifndef UART_TRANSPORT_H
#define UART_TRANSPORT_H
#include "diag_record.h"
#include "motion_config.h"
#define UART_RX_SIZE 512U
typedef struct {
    uint8_t bytes[DIAG_MAX_FRAME];
    uint16_t length;
    bool periodic;
} uart_tx_slot_t;
typedef struct {
    void *context;
    uint32_t (*lock)(void *);
    void (*unlock)(void *, uint32_t);
    bool (*start_tx)(void *, const uint8_t *, uint16_t);
    void (*command)(void *, const vision_cmd_t *);
} uart_transport_io_t;
typedef struct {
    void *context;
    bool (*abort_rx)(void *);
    void (*publish_rx)(void *);
    void (*drain_rx)(void *);
    bool (*start_rx)(void *);
    bool (*abort_tx)(void *);
} uart_recovery_io_t;
typedef struct {
    uint8_t rx[UART_RX_SIZE];
    volatile uint32_t producer;
    uint32_t consumer, rx_overflows, tx_drops, record_seq;
    vision_parser_t parser;
    uart_tx_slot_t tx[MOTION_TX_SLOTS];
    volatile uint32_t head, tail;
    volatile bool active, tx_starting;
    volatile bool rx_running, rx_error_pending, tx_error_pending;
    uint8_t reserved;
    uint32_t tx_start_us, tx_failure_us;
    bool tx_failed;
    uart_transport_io_t io;
} uart_transport_t;
void uart_transport_init(uart_transport_t *, uart_transport_io_t);
void uart_transport_rx_publish(uart_transport_t *, uint32_t absolute_count);
void uart_transport_rx_lost(uart_transport_t *);
uint32_t uart_transport_rx_restart(uart_transport_t *);
void uart_transport_poll(uart_transport_t *, uint32_t);
void uart_transport_tx_complete(uart_transport_t *);
void uart_transport_tx_error(uart_transport_t *);
void uart_transport_error(uart_transport_t *, bool rx_error, bool tx_error);
void uart_transport_recover(uart_transport_t *, uart_recovery_io_t);
bool uart_transport_reserve(uart_transport_t *);
bool uart_transport_record(uart_transport_t *, diag_record_t *, bool reserved, bool periodic);
uint8_t uart_transport_depth(const uart_transport_t *);
bool uart_transport_backpressure(const uart_transport_t *, uint32_t);
#endif
