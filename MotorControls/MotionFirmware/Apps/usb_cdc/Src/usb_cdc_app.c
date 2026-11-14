/**
 ******************************************************************************
 * @file    usb_cdc_app.c
 * @brief   Standalone verification & test application profile (fw_app.h) for
 * the portable TinyUSB USB_OTG_HS CDC module on SKR Octopus Pro V1.1.
 *
 *          Verifies:
 *          - USB-C Virtual COM Port enumeration (PA11/PA12, HSI48 + CRS)
 *          - Bidirectional ASCII line commands (PING, INFO, STAT, ECHO, LED,
 * BINTEST)
 *          - Multi-packet (>64 B) binary transfer integrity (e.g., 356 B
 * frames)
 *          - Non-blocking operation alongside SEGGER RTT & LD2 (PD15) heartbeat
 ******************************************************************************
 */
#include "SEGGER_RTT.h"
#include "fw_app.h"
#include "main.h"
#include "usb_cdc_port.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef enum { LED_MODE_HEARTBEAT = 0, LED_MODE_ON, LED_MODE_OFF } led_mode_t;

static char s_cmd_buf[256];
static uint32_t s_cmd_len = 0U;
static bool s_usb_ok = false;
static bool s_prev_connected = false;
static uint32_t s_last_led_ms = 0U;
static led_mode_t s_led_mode = LED_MODE_HEARTBEAT;

static void rtt_log(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  const int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (len > 0) {
    (void)SEGGER_RTT_Write(0U, buf, (unsigned)len);
  }
}

static void send_banner_over_cdc(void) {
  (void)usb_cdc_port_printf(
      "\r\n[USB_CDC] Octopus Pro V1.1 (STM32H723ZET6 @ %lu MHz) ready.\r\n"
      "[USB_CDC] Commands: HELP | PING | INFO | STAT | ECHO <msg> | LED "
      "<on|off|blink> | BINTEST [len]\r\n",
      (unsigned long)(HAL_RCC_GetSysClockFreq() / 1000000UL));
}

static void handle_command(const char *line) {
  while ((*line == ' ') || (*line == '\t')) {
    ++line;
  }
  if (*line == '\0') {
    return;
  }

  if (strcmp(line, "HELP") == 0 || strcmp(line, "help") == 0 ||
      strcmp(line, "?") == 0) {
    (void)usb_cdc_port_printf(
        "Commands:\r\n"
        "  PING               -> PONG <uptime_ms>\r\n"
        "  INFO               -> MCU clock, UID, USB status\r\n"
        "  STAT               -> RX/TX/drop counters & line coding\r\n"
        "  ECHO <text>        -> Echoes payload back\r\n"
        "  LED <on|off|blink> -> Controls PD15 (D27)\r\n"
        "  BINTEST [len]      -> Sends <len> (1..512, default 356) byte ramp "
        "frame\r\n");
  } else if (strcmp(line, "PING") == 0 || strcmp(line, "ping") == 0) {
    (void)usb_cdc_port_printf("PONG %lu\r\n", (unsigned long)HAL_GetTick());
  } else if (strcmp(line, "INFO") == 0 || strcmp(line, "info") == 0) {
    const volatile uint32_t *uid = (const volatile uint32_t *)UID_BASE;
    (void)usb_cdc_port_printf(
        "MCU=STM32H723ZET6 SYSCLK=%luHz HCLK=%luHz UID=%08lX%08lX%08lX "
        "USB_MOUNTED=%u DTR=%u UPTIME_MS=%lu\r\n",
        (unsigned long)HAL_RCC_GetSysClockFreq(),
        (unsigned long)HAL_RCC_GetHCLKFreq(), (unsigned long)uid[0],
        (unsigned long)uid[1], (unsigned long)uid[2],
        usb_cdc_port_is_mounted() ? 1U : 0U,
        usb_cdc_port_is_connected() ? 1U : 0U, (unsigned long)HAL_GetTick());
  } else if (strcmp(line, "STAT") == 0 || strcmp(line, "stat") == 0) {
    usb_cdc_stats_t st;
    usb_cdc_port_get_stats(&st);
    (void)usb_cdc_port_printf(
        "RX=%lu TX=%lu DROPPED=%lu MOUNTS=%lu UMOUNTS=%lu BAUD=%lu (%uN%u)\r\n",
        (unsigned long)st.rx_bytes, (unsigned long)st.tx_bytes,
        (unsigned long)st.tx_dropped_bytes, (unsigned long)st.mount_count,
        (unsigned long)st.umount_count, (unsigned long)st.bit_rate,
        (unsigned)st.data_bits, (unsigned)(st.stop_bits == 0U ? 1U : 2U));
  } else if (strncmp(line, "ECHO ", 5U) == 0 ||
             strncmp(line, "echo ", 5U) == 0) {
    (void)usb_cdc_port_printf("ECHO %s\r\n", line + 5U);
  } else if (strcmp(line, "LED on") == 0) {
    s_led_mode = LED_MODE_ON;
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    (void)usb_cdc_port_printf("OK LED=ON\r\n");
  } else if (strcmp(line, "LED off") == 0) {
    s_led_mode = LED_MODE_OFF;
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    (void)usb_cdc_port_printf("OK LED=OFF\r\n");
  } else if (strcmp(line, "LED blink") == 0) {
    s_led_mode = LED_MODE_HEARTBEAT;
    (void)usb_cdc_port_printf("OK LED=BLINK\r\n");
  } else if (strncmp(line, "BINTEST", 7U) == 0 ||
             strncmp(line, "bintest", 7U) == 0) {
    uint32_t len = 356U;
    if (line[7] == ' ') {
      const long parsed = strtol(line + 8, NULL, 10);
      if ((parsed >= 1L) && (parsed <= 512L)) {
        len = (uint32_t)parsed;
      }
    }
    uint8_t frame[512];
    frame[0] = 0xAAU;
    frame[1] = 0x55U;
    frame[2] = (uint8_t)(len & 0xFFU);
    frame[3] = (uint8_t)((len >> 8U) & 0xFFU);
    for (uint32_t i = 4U; i < len; ++i) {
      frame[i] = (uint8_t)(i & 0xFFU);
    }
    const bool ok = usb_cdc_port_write_all(frame, len, 100U);
    rtt_log("[USB_CDC] BINTEST len=%lu ok=%u\r\n", (unsigned long)len,
            ok ? 1U : 0U);
  } else {
    (void)usb_cdc_port_printf("ERR unknown_cmd '%s' (try HELP)\r\n", line);
  }
}

void fw_app_init(void) {
  SEGGER_RTT_Init();
  rtt_log(
      "\r\n[USB_CDC] Initializing TinyUSB USB_OTG_HS CDC on PA11/PA12...\r\n");

  s_usb_ok = usb_cdc_port_init();
  if (!s_usb_ok) {
    rtt_log("[USB_CDC] ERROR: usb_cdc_port_init() failed!\r\n");
    Error_Handler();
  }

  rtt_log(
      "[USB_CDC] TinyUSB initialized (SYSCLK=%lu MHz, HSI48+CRS active).\r\n",
      (unsigned long)(HAL_RCC_GetSysClockFreq() / 1000000UL));
  s_last_led_ms = HAL_GetTick();
}

void fw_app_poll(void) {
  usb_cdc_port_poll();

  const bool connected = usb_cdc_port_is_connected();
  if (connected && !s_prev_connected) {
    rtt_log("[USB_CDC] Host DTR connected.\r\n");
    send_banner_over_cdc();
  } else if (!connected && s_prev_connected) {
    rtt_log("[USB_CDC] Host DTR disconnected.\r\n");
  }
  s_prev_connected = connected;

  int ch = 0;
  while ((ch = usb_cdc_port_read_byte()) >= 0) {
    const char c = (char)ch;
    if ((c == '\r') || (c == '\n')) {
      if (s_cmd_len > 0U) {
        s_cmd_buf[s_cmd_len] = '\0';
        rtt_log("[USB_CDC] RX: %s\r\n", s_cmd_buf);
        handle_command(s_cmd_buf);
        s_cmd_len = 0U;
      }
    } else if (s_cmd_len < (sizeof(s_cmd_buf) - 1U)) {
      s_cmd_buf[s_cmd_len++] = c;
    } else {
      s_cmd_len = 0U;
    }
  }

  if (s_led_mode == LED_MODE_HEARTBEAT) {
    const uint32_t period_ms =
        connected ? 150U : (usb_cdc_port_is_mounted() ? 350U : 800U);
    const uint32_t now_ms = HAL_GetTick();
    if ((now_ms - s_last_led_ms) >= period_ms) {
      s_last_led_ms = now_ms;
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    }
  }
}

void fw_app_tim_irq(uint8_t motor) { (void)motor; }

bool fw_app_usart2_irq(void) { return false; }

void fw_app_error_handler(void) {
  rtt_log("[USB_CDC] FATAL: entered Error_Handler()\r\n");
}
