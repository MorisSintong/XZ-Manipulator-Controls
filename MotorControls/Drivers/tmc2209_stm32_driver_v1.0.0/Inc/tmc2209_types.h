/**
 * @file tmc2209_types.h
 * @brief TMC2209 driver status codes and common forward declarations.
 */

#ifndef TMC2209_TYPES_H
#define TMC2209_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#if !defined(TMC2209_NO_CONF)
#if defined(__has_include)
#if __has_include("tmc2209_conf.h")
#include "tmc2209_conf.h"
#endif
#else
#include "tmc2209_conf.h"
#endif
#endif

/* Fallback default definitions if tmc2209_conf.h is not included */
#ifndef TMC2209_OS_FREERTOS
#define TMC2209_OS_FREERTOS 0
#endif
#ifndef TMC2209_OS_NONE
#define TMC2209_OS_NONE (!TMC2209_OS_FREERTOS)
#endif
#ifndef TMC2209_DEFAULT_BAUD
#define TMC2209_DEFAULT_BAUD 115200u
#endif
#ifndef TMC2209_BUS_LOCK_TIMEOUT_MS
#define TMC2209_BUS_LOCK_TIMEOUT_MS 100u
#endif
#ifndef TMC2209_UART_TX_TIMEOUT_MS
#define TMC2209_UART_TX_TIMEOUT_MS 20u
#endif
#ifndef TMC2209_UART_RX_TIMEOUT_MS
#define TMC2209_UART_RX_TIMEOUT_MS 20u
#endif
#ifndef TMC2209_BUS_IDLE_US
#define TMC2209_BUS_IDLE_US 0u
#endif
#ifndef TMC2209_POLL_LIMIT
#define TMC2209_POLL_LIMIT 10000000u
#endif
#ifndef TMC2209_RX_DRAIN_LIMIT
#define TMC2209_RX_DRAIN_LIMIT 32u
#endif
#ifndef TMC2209_OS_CREATE_MUTEX
#define TMC2209_OS_CREATE_MUTEX 1
#endif

#if (TMC2209_OS_NONE != 0 && TMC2209_OS_NONE != 1) || \
    (TMC2209_OS_FREERTOS != 0 && TMC2209_OS_FREERTOS != 1) || \
    (TMC2209_OS_NONE + TMC2209_OS_FREERTOS != 1)
#error "Select exactly one TMC2209 OS backend"
#endif
#if TMC2209_OS_CREATE_MUTEX != 0 && TMC2209_OS_CREATE_MUTEX != 1
#error "TMC2209_OS_CREATE_MUTEX must be 0 or 1"
#endif
#if defined(TMC2209_REPLY_GAP_US) || defined(TMC2209_REPLY_POLL_US)
#error "Obsolete reply delays: receive immediately; use TMC2209_BUS_IDLE_US for interframe idle"
#endif
#if TMC2209_UART_TX_TIMEOUT_MS < 1 || TMC2209_UART_TX_TIMEOUT_MS > 1000 || \
    TMC2209_UART_RX_TIMEOUT_MS < 1 || TMC2209_UART_RX_TIMEOUT_MS > 1000 || \
    TMC2209_BUS_LOCK_TIMEOUT_MS > 60000
#error "TMC2209 timeouts must be finite (UART 1..1000 ms, lock 0..60000 ms)"
#endif
#if TMC2209_DEFAULT_BAUD < 9000 || TMC2209_DEFAULT_BAUD > 500000 || \
    TMC2209_BUS_IDLE_US > 1000000 || \
    TMC2209_POLL_LIMIT < 1 || TMC2209_POLL_LIMIT > 100000000 || \
    TMC2209_RX_DRAIN_LIMIT < 1 || TMC2209_RX_DRAIN_LIMIT > 256
#error "Invalid TMC2209 transport configuration"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Driver status / error codes.
 *
 * Fallible operations return this type. Check for TMC2209_OK before using
 * out-parameter values; CRC utilities and infallible delete hooks differ.
 */
typedef enum {
  TMC2209_OK = 0,      /**< Success                       */
  TMC2209_ERR_PARAM,   /**< Invalid parameter (NULL ptr, bad addr) */
  TMC2209_ERR_PORT,    /**< UART transport error          */
  TMC2209_ERR_TIMEOUT, /**< UART or mutex timeout         */
  TMC2209_ERR_CRC,     /**< Reply CRC mismatch            */
  TMC2209_ERR_BUSY,    /**< Bus mutex could not be acquired */
  TMC2209_ERR_NODEV,   /**< No reply / device not present */
  TMC2209_ERR_OS,      /**< OS primitive failure           */
  TMC2209_ERR_STATE,   /**< Uninitialised/in-use/unconfigured object */
  TMC2209_ERR_VERIFY,  /**< Write acceptance or readback mismatch */
  TMC2209_ERR_FAULT    /**< Unacknowledged reset or observed driver fault/warning */
} tmc2209_status_t;

/**
 * @brief Sentinel for "no data / error" on uint32_t read paths.
 *
 * Legacy constant only. Read outputs are unchanged on failure and every
 * 32-bit value is data; only the returned status determines success.
 */
#define TMC2209_UINT32_ERR 0xFFFFFFFFu

#ifdef __cplusplus
}
#endif

#endif /* TMC2209_TYPES_H */
