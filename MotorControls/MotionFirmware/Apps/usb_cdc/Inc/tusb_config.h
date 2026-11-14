/**
 ******************************************************************************
 * @file    tusb_config.h
 * @brief   TinyUSB 0.18.0 configuration for BIGTREETECH SKR Octopus Pro V1.1
 *          (STM32H723ZET6, USB1_OTG_HS @ 0x40040000 in internal Full-Speed
 * PHY).
 ******************************************************************************
 */
#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*---------------------------------------------------------------------------*
 * Board / MCU Configuration
 *---------------------------------------------------------------------------*/
#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU OPT_MCU_STM32H7
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_NONE
#endif

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

/*
 * STM32H723 has only one USB controller (USB1_OTG_HS @ 0x40040000, internal
 * Full-Speed PHY on PA11/PA12). TinyUSB's dwc2_stm32.h aliases RHPort 0 to
 * USB1_OTG_HS / OTG_HS_IRQn when USB2_OTG_FS is absent.
 */
#ifndef CFG_TUSB_RHPORT0_MODE
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#endif

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

/*---------------------------------------------------------------------------*
 * Device Stack Configuration
 *---------------------------------------------------------------------------*/
#define CFG_TUD_ENABLED 1
#define CFG_TUD_MAX_SPEED OPT_MODE_FULL_SPEED
#define CFG_TUD_ENDPOINT0_SIZE 64

/* Use DWC2 dedicated packet FIFO RAM directly (avoids AHB DMA cache/DTCM
 * issues). */
#define CFG_TUD_DWC2_SLAVE_ENABLE 1
#define CFG_TUD_DWC2_DMA_ENABLE 0

/* Octopus Pro PA9 is wired to USART1_TX (J70), not USB VBUS. */
#define CFG_TUD_VBUS_SENSING 0

/*---------------------------------------------------------------------------*
 * USB Device Classes
 *---------------------------------------------------------------------------*/
#define CFG_TUD_CDC 1
#define CFG_TUD_MSC 0
#define CFG_TUD_HID 0
#define CFG_TUD_MIDI 0
#define CFG_TUD_VENDOR 0
#define CFG_TUD_AUDIO 0
#define CFG_TUD_VIDEO 0
#define CFG_TUD_BTH 0
#define CFG_TUD_ECM_RNDIS 0
#define CFG_TUD_NCM 0
#define CFG_TUD_DFU 0
#define CFG_TUD_DFU_RUNTIME 0
#define CFG_TUD_USBTMC 0

/* CDC FIFO & Endpoint Buffer Sizes (Full-Speed Bulk max packet = 64 bytes) */
#define CFG_TUD_CDC_RX_BUFSIZE 512
#define CFG_TUD_CDC_TX_BUFSIZE 4096
#define CFG_TUD_CDC_EP_BUFSIZE 64

#ifdef __cplusplus
}
#endif

#endif /* TUSB_CONFIG_H */
