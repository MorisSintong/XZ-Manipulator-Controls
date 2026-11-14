/**
 ******************************************************************************
 * @file    usb_cdc_port.h
 * @brief   Portable, non-blocking TinyUSB CDC (USB-C Virtual COM Port) module
 *          for BIGTREETECH SKR Octopus Pro V1.1 (STM32H723ZET6).
 *
 *          Drop-in transport designed to be reused directly by motion firmware
 *          profiles (motion_diagnostics, stallguard_cli, z_bounce_emulation)
 *          without altering PLL1 (400 MHz CPU / TIM2 / TIM5) or requiring PA9
 *          hardware VBUS sensing.
 ******************************************************************************
 */
#ifndef USB_CDC_PORT_H
#define USB_CDC_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Runtime counters & link state exposed for telemetry/debugging. */
typedef struct {
  uint32_t rx_bytes;
  uint32_t tx_bytes;
  uint32_t tx_dropped_bytes;
  uint32_t mount_count;
  uint32_t umount_count;
  uint32_t bit_rate;
  uint8_t stop_bits;
  uint8_t parity;
  uint8_t data_bits;
  bool mounted;
  bool dtr_connected;
} usb_cdc_stats_t;

/**
 * @brief Initialize 48 MHz HSI48 + CRS clock, USB voltage detector, PA11/PA12
 *        AF10_OTG1_HS pins, VBUS-override (PA9 bypass), OTG_HS NVIC priority 5,
 *        and TinyUSB CDC device stack on RHPort 0.
 * @return true if initialization succeeded, false on HAL clock config error or
 *         TinyUSB init failure.
 */
bool usb_cdc_port_init(void);

/**
 * @brief Pump TinyUSB device events non-blockingly. Call on every iteration of
 *        fw_app_poll(). Safe and fast even when USB is unplugged.
 */
void usb_cdc_port_poll(void);

/** @return true when USB host has enumerated & configured the device. */
bool usb_cdc_port_is_mounted(void);

/**
 * @return true when USB is mounted AND the host terminal has asserted DTR.
 *         Note: Windows .NET SerialPort / some raw tools default
 * DtrEnable=false; usb_cdc_port_write() only requires
 * usb_cdc_port_is_mounted().
 */
bool usb_cdc_port_is_connected(void);

/** @return Number of unread bytes currently waiting in the CDC RX FIFO. */
uint32_t usb_cdc_port_available(void);

/**
 * @brief Non-blocking read of up to max_len bytes from CDC RX FIFO.
 * @return Number of bytes copied into buf (0 if none available).
 */
uint32_t usb_cdc_port_read(uint8_t *buf, uint32_t max_len);

/**
 * @brief Non-blocking single-byte read (drop-in for console_getc() / RXNE).
 * @return Byte value [0..255], or -1 if RX FIFO is empty.
 */
int usb_cdc_port_read_byte(void);

/**
 * @brief Non-blocking write into CDC TX FIFO + flush trigger.
 *        Requires USB to be mounted (works even if host leaves DTR low).
 *        Never blocks; excess bytes beyond TX FIFO space are dropped and
 *        counted in tx_dropped_bytes.
 * @return Number of bytes queued into the TinyUSB TX FIFO.
 */
uint32_t usb_cdc_port_write(const uint8_t *data, uint32_t len);

/**
 * @brief Bounded-timeout write for complete binary frames (e.g., 356-byte
 *        diagnostic records) or bursts larger than 64 bytes. Pumps TinyUSB
 *        between chunks until all bytes are queued or timeout_ms elapses.
 *        Returns false immediately if USB is not mounted.
 */
bool usb_cdc_port_write_all(const uint8_t *data, uint32_t len,
                            uint32_t timeout_ms);

/**
 * @brief Non-blocking formatted print helper over CDC TX.
 * @return Number of bytes queued, or 0 if not mounted / dropped.
 */
int usb_cdc_port_printf(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));

/** @brief Copy current CDC statistics and line-coding state. */
void usb_cdc_port_get_stats(usb_cdc_stats_t *out);

/**
 * @brief Explicit IRQ forwarder (called automatically by OTG_HS_IRQHandler
 *        unless USB_CDC_PORT_EXTERNAL_IRQ_HANDLER is defined).
 */
void usb_cdc_port_irq_handler(void);

#ifdef __cplusplus
}
#endif

#endif /* USB_CDC_PORT_H */
