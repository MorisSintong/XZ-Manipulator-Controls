#include "uart_transport.h"
#include <string.h>

void uart_transport_init(uart_transport_t *t, uart_transport_io_t io)
{
    memset(t, 0, sizeof(*t)); t->io = io; vision_parser_init(&t->parser);
}
void uart_transport_rx_publish(uart_transport_t *t, uint32_t count)
{
    t->producer = count;
}
void uart_transport_error(uart_transport_t *t, bool rx_error, bool tx_error)
{
    if (rx_error) { t->rx_error_pending = true; }
    if (tx_error) { t->tx_error_pending = true; }
}
void uart_transport_recover(uart_transport_t *t, uart_recovery_io_t io)
{
    if (t->tx_error_pending) {
        t->tx_error_pending = false;
        if (io.abort_tx(io.context)) { uart_transport_tx_error(t); }
        else { t->tx_error_pending = true; }
    }
    if (t->rx_error_pending) {
        t->rx_error_pending = false;
        if (t->rx_running) {
            if (!io.abort_rx(io.context)) { t->rx_error_pending = true; return; }
            io.publish_rx(io.context);
            io.drain_rx(io.context);
        }
        t->rx_running = false;
        (void)uart_transport_rx_restart(t);
    }
    if (!t->rx_running) { t->rx_running = io.start_rx(io.context); }
    else { io.publish_rx(io.context); }
}
void uart_transport_rx_lost(uart_transport_t *t)
{
    ++t->rx_overflows; t->parser.used = 0U; t->consumer = t->producer;
}
uint32_t uart_transport_rx_restart(uart_transport_t *t)
{
    /* DMA restarts at rx[0]; the logical origin must have the same modulo.
     * Drain complete pre-error frames before this call; discard only parser residue. */
    const uint32_t p = t->io.lock(t->io.context);
    const uint32_t base = (t->producer + UART_RX_SIZE - 1U) & ~(UART_RX_SIZE - 1U);
    if (t->parser.used != 0U) { ++t->parser.resyncs; }
    ++t->rx_overflows;
    t->parser.used = 0U; t->producer = base; t->consumer = base;
    t->io.unlock(t->io.context, p);
    return base;
}
uint8_t uart_transport_depth(const uart_transport_t *t)
{
    return (uint8_t)(t->head - t->tail);
}
bool uart_transport_reserve(uart_transport_t *t)
{
    if (t->head - t->tail + t->reserved >= MOTION_TX_SLOTS) { return false; }
    ++t->reserved; return true;
}
bool uart_transport_record(uart_transport_t *t, diag_record_t *r,
                           bool reserved, bool periodic)
{
    uint32_t p = t->io.lock(t->io.context);
    uint32_t slot = t->head;
    bool coalesce = false;
    if (periodic && slot != t->tail) {
        const uint32_t last = slot - 1U;
        if (t->tx[last % MOTION_TX_SLOTS].periodic &&
            (!t->active || last != t->tail)) { slot = last; coalesce = true; }
    }
    if ((reserved && t->reserved == 0U) ||
        (!coalesce && slot - t->tail + (reserved ? 0U : t->reserved) >= MOTION_TX_SLOTS)) {
        ++t->tx_drops; t->io.unlock(t->io.context, p); return false;
    }
    t->io.unlock(t->io.context, p);
    uart_tx_slot_t *s = &t->tx[slot % MOTION_TX_SLOTS];
    r->record_seq = t->record_seq + 1U;
    const size_t size = diag_record_pack(r, s->bytes, sizeof(s->bytes));
    if (size == 0U) { return false; }
    s->length = (uint16_t)size; s->periodic = periodic;
    p = t->io.lock(t->io.context);
    ++t->record_seq;
    if (reserved) { --t->reserved; }
    if (!coalesce) { ++t->head; }
    t->io.unlock(t->io.context, p);
    return true;
}
void uart_transport_tx_complete(uart_transport_t *t)
{
    if (t->active) { ++t->tail; t->active = false; t->tx_failed = false; }
}
void uart_transport_tx_error(uart_transport_t *t)
{
    /* Keep the immutable frame. A receiver resynchronizes if a prefix escaped. */
    if (!t->tx_starting) { t->active = false; }
}
bool uart_transport_backpressure(const uart_transport_t *t, uint32_t now)
{
    return t->head - t->tail + t->reserved >= MOTION_TX_SLOTS ||
        (t->active && now - t->tx_start_us > 250000U) ||
        (t->tx_failed && now - t->tx_failure_us > 100000U);
}
void uart_transport_poll(uart_transport_t *t, uint32_t now)
{
    uint32_t available = t->producer - t->consumer;
    if (available >= UART_RX_SIZE) {
        uart_transport_rx_lost(t);
        available = 0U;
    }
    /* Bound each pump; callbacks only publish DMA producer progress. */
    while (available-- != 0U) {
        vision_cmd_t cmd;
        const uint8_t byte = t->rx[t->consumer % UART_RX_SIZE];
        ++t->consumer;
        if (vision_parser_feed(&t->parser, byte, &cmd) == VISION_PARSE_FRAME_OK) {
            t->io.command(t->io.context, &cmd);
        }
    }
    const uart_tx_slot_t *slot = NULL;
    uint32_t claimed_tail = 0U;
    uint32_t p = t->io.lock(t->io.context);
    if (!t->active && t->head != t->tail) {
        claimed_tail = t->tail;
        slot = &t->tx[claimed_tail % MOTION_TX_SLOTS];
        t->active = true; t->tx_starting = true; t->tx_start_us = now;
    }
    t->io.unlock(t->io.context, p);
    /* HAL timeouts and timer interrupts must remain live while starting DMA.
     * Completion may run before start_tx returns: never re-assert ownership. */
    if (slot != NULL) {
        const bool started = t->io.start_tx(t->io.context, slot->bytes, slot->length);
        p = t->io.lock(t->io.context);
        t->tx_starting = false;
        if (!started && t->active && t->tail == claimed_tail) {
            t->active = false;
            if (!t->tx_failed) { t->tx_failure_us = now; }
            t->tx_failed = true;
        }
        t->io.unlock(t->io.context, p);
    }
}
