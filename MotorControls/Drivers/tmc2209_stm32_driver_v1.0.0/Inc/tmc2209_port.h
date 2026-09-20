/**
 * STM32F446 half-duplex, polling UART port. Own the UART exclusively.
 * Configure 8N1, no flow control, half duplex, TX open drain with pull-up.
 * Calls are task-context only. Interrupts/debugger/scheduler must not delay
 * RX service beyond one byte time; hardware timing is not host-qualified.
 */
#ifndef TMC2209_PORT_H
#define TMC2209_PORT_H
#include "tmc2209_types.h"
#ifdef __cplusplus
extern "C" {
#endif

struct __UART_HandleTypeDef;
typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

typedef struct {
    UART_HandleTypeDef *huart;
    uint32_t baud;              /**< Must match HAL Init.BaudRate, 9000..500000. */
    uint32_t tx_timeout_ms;     /**< Entire frame incl. TC, 1..1000 ms. */
    uint32_t rx_timeout_ms;     /**< Entire reply incl. SENDDELAY, 1..1000 ms. */
    uint32_t bus_idle_us;       /**< Additional minimum idle; 0 uses 80 bit times. */
    bool initialized;
    bool awaiting_reply;
} tmc2209_port_t;

/* Initialize configuration before Port_Init; do not modify initialized state.
 * For runtime transport reconfiguration, quiesce/deinitialize first.
 */
tmc2209_status_t TMC2209_Port_Init(tmc2209_port_t *port);
tmc2209_status_t TMC2209_Port_Deinit(tmc2209_port_t *port);
/** Success means physically transmitted, not acknowledged by the slave. */
tmc2209_status_t TMC2209_Port_Transmit(tmc2209_port_t *port,
                                     const uint8_t *data, uint16_t len);
/** Receive immediately after a four-byte request; never discard live reply data.
 * A failed receive may have filled part of the buffer; ignore it on error.
 */
tmc2209_status_t TMC2209_Port_Receive(tmc2209_port_t *port,
                                    uint8_t *data, uint16_t len);
/** Bounded RX drain. Use only when discarding stale data is intentional. */
tmc2209_status_t TMC2209_Port_FlushRx(tmc2209_port_t *port);
/** DWT delay, 0..1,000,000 us. Reports stopped counter / invalid clock. */
tmc2209_status_t TMC2209_Port_DelayUs(uint32_t us);
#ifdef __cplusplus
}
#endif
#endif
