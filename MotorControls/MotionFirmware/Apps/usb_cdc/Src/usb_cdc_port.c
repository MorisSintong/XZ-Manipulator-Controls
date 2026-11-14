/**
 ******************************************************************************
 * @file    usb_cdc_port.c
 * @brief   Portable TinyUSB USB_OTG_HS CDC implementation for STM32H723ZET6
 *          (BIGTREETECH SKR Octopus Pro V1.1).
 ******************************************************************************
 */
#include "usb_cdc_port.h"

#include "stm32h7xx_hal.h"
#include "tusb.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define USB_CDC_RHPORT 0U
#define USB_CDC_IRQ_PRIORITY                                                   \
  5U /* Lower urgency than TIM2/TIM5 step IRQs (priority 0) */

static usb_cdc_stats_t s_stats;
static bool s_initialized = false;

static bool usb_cdc_enable_hsi48_and_crs(void) {
  /* Turn on HSI48 without altering HSE or PLL1 (400 MHz SYSCLK / TIM2 / TIM5).
   */
  RCC_OscInitTypeDef osc = {0};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
  osc.HSI48State = RCC_HSI48_ON;
  osc.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    return false;
  }

  RCC_PeriphCLKInitTypeDef periph_clk = {0};
  periph_clk.PeriphClockSelection = RCC_PERIPHCLK_USB;
  periph_clk.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&periph_clk) != HAL_OK) {
    return false;
  }

  /* Trim HSI48 continuously to USB 1 kHz SOF frames via CRS. */
  __HAL_RCC_CRS_CLK_ENABLE();
  RCC_CRSInitTypeDef crs = {0};
  crs.Prescaler = RCC_CRS_SYNC_DIV1;
  crs.Source = RCC_CRS_SYNC_SOURCE_USB1;
  crs.Polarity = RCC_CRS_SYNC_POLARITY_RISING;
  crs.ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT;
  crs.ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT;
  crs.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
  HAL_RCCEx_CRSConfig(&crs);

  return true;
}

static void usb_cdc_configure_gpio_and_vbus_bypass(void) {
  HAL_PWREx_EnableUSBVoltageDetector();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef gpio = {0};
  gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF10_OTG1_HS;
  HAL_GPIO_Init(GPIOA, &gpio);

  __HAL_RCC_USB1_OTG_HS_CLK_ENABLE();

  /*
   * SKR Octopus Pro V1.1 routes PA9 to J70 USART1_TX, not 5V VBUS.
   * Disable hardware VBUS sensing and force B-peripheral session valid so USB
   * attaches reliably under both USB-C bench power (J68) and 24V MAIN POWER.
   */
  USB_OTG_HS->GCCFG &= ~USB_OTG_GCCFG_VBDEN;
  USB_OTG_HS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN | USB_OTG_GOTGCTL_BVALOVAL;
}

bool usb_cdc_port_init(void) {
  memset(&s_stats, 0, sizeof(s_stats));
  s_stats.bit_rate = 115200U;
  s_stats.data_bits = 8U;

  if (!usb_cdc_enable_hsi48_and_crs()) {
    return false;
  }

  usb_cdc_configure_gpio_and_vbus_bypass();

  HAL_NVIC_SetPriority(OTG_HS_IRQn, USB_CDC_IRQ_PRIORITY, 0U);

  const tusb_rhport_init_t dev_init = {
      .role = TUSB_ROLE_DEVICE,
      .speed = TUSB_SPEED_FULL,
  };
  if (!tusb_rhport_init(USB_CDC_RHPORT, &dev_init)) {
    return false;
  }

  /* Keep VBUS sensing disabled after core reset inside dcd_init(). */
  USB_OTG_HS->GCCFG &= ~USB_OTG_GCCFG_VBDEN;
  USB_OTG_HS->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN | USB_OTG_GOTGCTL_BVALOVAL;

  s_initialized = true;
  return true;
}

void usb_cdc_port_poll(void) {
  if (!s_initialized) {
    return;
  }
  tud_task_ext(0U, false);
}

bool usb_cdc_port_is_mounted(void) { return s_initialized && tud_mounted(); }

bool usb_cdc_port_is_connected(void) {
  return s_initialized && tud_mounted() && tud_cdc_connected();
}

uint32_t usb_cdc_port_available(void) {
  if (!s_initialized) {
    return 0U;
  }
  return tud_cdc_available();
}

uint32_t usb_cdc_port_read(uint8_t *buf, uint32_t max_len) {
  if (!s_initialized || (buf == NULL) || (max_len == 0U) ||
      !tud_cdc_available()) {
    return 0U;
  }
  const uint32_t n = tud_cdc_read(buf, max_len);
  s_stats.rx_bytes += n;
  return n;
}

int usb_cdc_port_read_byte(void) {
  uint8_t ch = 0U;
  if (usb_cdc_port_read(&ch, 1U) == 1U) {
    return (int)ch;
  }
  return -1;
}

uint32_t usb_cdc_port_write(const uint8_t *data, uint32_t len) {
  if ((data == NULL) || (len == 0U)) {
    return 0U;
  }
  if (!usb_cdc_port_is_mounted()) {
    s_stats.tx_dropped_bytes += len;
    return 0U;
  }

  const uint32_t avail = tud_cdc_write_available();
  const uint32_t to_write = (len <= avail) ? len : avail;
  uint32_t written = 0U;
  if (to_write > 0U) {
    written = tud_cdc_write(data, to_write);
    (void)tud_cdc_write_flush();
    s_stats.tx_bytes += written;
  }
  if (written < len) {
    s_stats.tx_dropped_bytes += (len - written);
  }
  return written;
}

bool usb_cdc_port_write_all(const uint8_t *data, uint32_t len,
                            uint32_t timeout_ms) {
  if ((data == NULL) || (len == 0U)) {
    return true;
  }
  if (!usb_cdc_port_is_mounted()) {
    s_stats.tx_dropped_bytes += len;
    return false;
  }

  const uint32_t start_ms = HAL_GetTick();
  uint32_t offset = 0U;

  while (offset < len) {
    usb_cdc_port_poll();
    if (!usb_cdc_port_is_mounted()) {
      s_stats.tx_dropped_bytes += (len - offset);
      return false;
    }

    const uint32_t avail = tud_cdc_write_available();
    if (avail > 0U) {
      const uint32_t chunk = ((len - offset) < avail) ? (len - offset) : avail;
      const uint32_t written = tud_cdc_write(data + offset, chunk);
      (void)tud_cdc_write_flush();
      offset += written;
      s_stats.tx_bytes += written;
    }

    if ((offset < len) && ((HAL_GetTick() - start_ms) >= timeout_ms)) {
      s_stats.tx_dropped_bytes += (len - offset);
      return false;
    }
  }

  return true;
}

int usb_cdc_port_printf(const char *fmt, ...) {
  char buf[256];
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  if (len <= 0) {
    return 0;
  }
  if ((size_t)len >= sizeof(buf)) {
    len = (int)sizeof(buf) - 1;
  }
  return (int)usb_cdc_port_write((const uint8_t *)buf, (uint32_t)len);
}

void usb_cdc_port_get_stats(usb_cdc_stats_t *out) {
  if (out == NULL) {
    return;
  }
  s_stats.mounted = usb_cdc_port_is_mounted();
  s_stats.dtr_connected = usb_cdc_port_is_connected();

  cdc_line_coding_t coding;
  if (s_initialized) {
    tud_cdc_get_line_coding(&coding);
    s_stats.bit_rate = coding.bit_rate;
    s_stats.stop_bits = coding.stop_bits;
    s_stats.parity = coding.parity;
    s_stats.data_bits = coding.data_bits;
  }

  *out = s_stats;
}

void usb_cdc_port_irq_handler(void) { tusb_int_handler(USB_CDC_RHPORT, true); }

#ifndef USB_CDC_PORT_EXTERNAL_IRQ_HANDLER
void OTG_HS_IRQHandler(void) { usb_cdc_port_irq_handler(); }
#endif

/*---------------------------------------------------------------------------*
 * TinyUSB Device Callbacks
 *---------------------------------------------------------------------------*/
void tud_mount_cb(void) { ++s_stats.mount_count; }

void tud_umount_cb(void) { ++s_stats.umount_count; }

void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
  (void)itf;
  (void)dtr;
  (void)rts;
}
