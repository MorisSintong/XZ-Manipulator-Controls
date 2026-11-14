# Reusable TinyUSB USB_OTG_HS CDC module helper for MotionFirmware targets.
# Include this file from MotionFirmware/CMakeLists.txt, then call:
#   add_usb_cdc_module(<target_name>)
# This adds only the portable TinyUSB CDC transport (usb_cdc_port.c,
# usb_descriptors.c, tusb_config.h, and TinyUSB 0.18.0 DWC2/CDC sources),
# NOT the standalone usb_cdc_app.c demo entry point.

set(USB_CDC_MODULE_DIR ${CMAKE_CURRENT_LIST_DIR})
set(TINYUSB_DIR ${CMAKE_CURRENT_LIST_DIR}/../../../Drivers/tinyusb)

set(USB_CDC_PORTABLE_SOURCES
    ${USB_CDC_MODULE_DIR}/Src/usb_cdc_port.c
    ${USB_CDC_MODULE_DIR}/Src/usb_descriptors.c
    ${TINYUSB_DIR}/src/tusb.c
    ${TINYUSB_DIR}/src/common/tusb_fifo.c
    ${TINYUSB_DIR}/src/device/usbd.c
    ${TINYUSB_DIR}/src/device/usbd_control.c
    ${TINYUSB_DIR}/src/class/cdc/cdc_device.c
    ${TINYUSB_DIR}/src/portable/synopsys/dwc2/dcd_dwc2.c
    ${TINYUSB_DIR}/src/portable/synopsys/dwc2/dwc2_common.c
)

set(USB_CDC_PORTABLE_INCLUDES
    ${USB_CDC_MODULE_DIR}/Inc
    ${TINYUSB_DIR}/src
)

function(add_usb_cdc_module target_name)
    target_sources(${target_name} PRIVATE ${USB_CDC_PORTABLE_SOURCES})
    target_include_directories(${target_name} PRIVATE ${USB_CDC_PORTABLE_INCLUDES})
endfunction()
