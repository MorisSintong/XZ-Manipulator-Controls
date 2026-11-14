/**
 ******************************************************************************
 * @file    usb_descriptors.c
 * @brief   TinyUSB CDC ACM descriptors for SKR Octopus Pro V1.1 (STM32H723).
 ******************************************************************************
 */
#include "stm32h7xx.h"
#include "tusb.h"

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* TinyUSB standard CDC ACM VID/PID so Windows 10/11 loads usbser.sys
 * automatically */
#define USB_VID 0xCafeU
#define USB_PID 0x4001U
#define USB_BCD 0x0200U

static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,

    /* IAD required for CDC composite compatibility on Windows/Linux/macOS */
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x0100U,

    .iManufacturer = 0x01U,
    .iProduct = 0x02U,
    .iSerialNumber = 0x03U,

    .bNumConfigurations = 0x01U,
};

uint8_t const *tud_descriptor_device_cb(void) {
  return (uint8_t const *)&desc_device;
}

enum { ITF_NUM_CDC = 0, ITF_NUM_CDC_DATA, ITF_NUM_TOTAL };

#define EPNUM_CDC_NOTIF 0x81U
#define EPNUM_CDC_OUT 0x02U
#define EPNUM_CDC_IN 0x82U

#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

static const uint8_t desc_fs_configuration[] = {
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EPNUM_CDC_NOTIF, 8, EPNUM_CDC_OUT,
                       EPNUM_CDC_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
  (void)index;
  return desc_fs_configuration;
}

enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_CDC_INTERFACE
};

static const char *const string_desc_arr[] = {
    (const char[]){0x09, 0x04}, /* 0: English (0x0409) */
    "XZ-Manipulator",           /* 1: Manufacturer */
    "XZ-MotionControls",        /* 2: Product */
    NULL,                /* 3: Serial (populated from STM32 96-bit UID) */
    "XZ-MotionControls", /* 4: CDC Interface */
};

static uint16_t s_desc_str[32 + 1];

static size_t format_uid_utf16(uint16_t *dst, size_t max_chars) {
  static const char hex[] = "0123456789ABCDEF";
  const volatile uint32_t *uid = (const volatile uint32_t *)UID_BASE;
  size_t pos = 0U;

  for (size_t w = 0U; (w < 3U) && (pos + 8U <= max_chars); ++w) {
    const uint32_t word = uid[w];
    for (int shift = 28; shift >= 0; shift -= 4) {
      dst[pos++] = (uint16_t)hex[(word >> (uint32_t)shift) & 0x0FU];
    }
  }
  return pos;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
  (void)langid;
  size_t chr_count = 0U;

  switch (index) {
  case STRID_LANGID:
    memcpy(&s_desc_str[1], string_desc_arr[0], 2U);
    chr_count = 1U;
    break;

  case STRID_SERIAL:
    chr_count = format_uid_utf16(&s_desc_str[1], 32U);
    break;

  default: {
    if (index >=
        (uint8_t)(sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))) {
      return NULL;
    }
    const char *str = string_desc_arr[index];
    if (str == NULL) {
      return NULL;
    }
    chr_count = strlen(str);
    if (chr_count > 32U) {
      chr_count = 32U;
    }
    for (size_t i = 0U; i < chr_count; ++i) {
      s_desc_str[1U + i] = (uint16_t)(uint8_t)str[i];
    }
    break;
  }
  }

  s_desc_str[0] = (uint16_t)(((uint16_t)TUSB_DESC_STRING << 8U) |
                             (uint16_t)(2U * chr_count + 2U));
  return s_desc_str;
}
