cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_OBJECT_PATH_MAX 210)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Select which firmware profile is built as the primary "MotionFirmware" (.elf/.hex/.bin)
# Options: "motion_diagnostics" | "stallguard_cli" | "z_bounce_emulation" | "usb_cdc" | "motion_characterization" | "as5600_tuner"
set(FIRMWARE_APP "motion_diagnostics" CACHE STRING
    "Default firmware app profile for MotionFirmware.{elf,hex,bin}")
set_property(CACHE FIRMWARE_APP PROPERTY STRINGS
    "motion_diagnostics" "stallguard_cli" "z_bounce_emulation" "usb_cdc" "motion_characterization" "as5600_tuner")

# Target-side configuration parameters for motion_diagnostics profile
option(MOTION_DIAG_BOUNDS_CONFIRMED
    "Confirm physical X/Z bounds on target so VisionSystem PICK commands are accepted" ON)
option(MOTION_DIAG_CLAMP_X
    "Clamp out-of-range X pick targets to [0, usable_max_01mm] instead of rejecting" OFF)
option(MOTION_DIAG_MSCNT_QUALIFIED
    "Enforce strict TMC2240 MSCNT phase verification on every move phase" OFF)
set(MOTION_DIAG_SAFE_Z_01MM "0" CACHE STRING
    "Safe Z travel height in 0.1 mm units (0 = top home clearance)")
set(MOTION_DIAG_PICK_DEPTH_01MM "0" CACHE STRING
    "Z pick plunge depth in 0.1 mm units (0 = safe Z dry-run; e.g. 500 = 50.0 mm plunge)")
set(MOTION_DIAG_X_DIR_SIGN "1" CACHE STRING
    "X-axis STEP/DIR polarity (+1 or -1)")
set(MOTION_DIAG_X_ENCODER_SIGN "1" CACHE STRING
    "X-axis AS5600 encoder count polarity (+1 or -1)")
set(MOTION_DIAG_Z_DIR_SIGN "-1" CACHE STRING
    "Z-axis STEP/DIR polarity (-1 = DIR HIGH moves UP toward home, +Z moves DOWN)")
set(MOTION_DIAG_Z_ENCODER_SIGN "1" CACHE STRING
    "Z-axis AS5600 encoder count polarity (+1 or -1)")

set(CMAKE_PROJECT_NAME MotionFirmware)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message(STATUS "Build type: ${CMAKE_BUILD_TYPE} | Default FIRMWARE_APP: ${FIRMWARE_APP}")

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
enable_language(C CXX ASM)

# Default executable (used by flash scripts and debuggers)
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated Core + HAL sources
add_subdirectory(cmake/stm32cubemx)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
list(REMOVE_ITEM CMAKE_CXX_IMPLICIT_LINK_LIBRARIES ob)

# ---------------------------------------------------------------------------
# Scrutiny Debugger embedded instrumentation library (used by as5600_tuner)
# ---------------------------------------------------------------------------
option(ENABLE_SCRUTINY
    "Fetch and link scrutiny-embedded over RTT in MotionFirmware_AS5600Tuner (set OFF for offline builds)"
    ON)
if(ENABLE_SCRUTINY)
    include(FetchContent)
    set(SCRUTINY_ENABLE_DATALOGGING 1 CACHE BOOL "" FORCE)
    set(SCRUTINY_SUPPORT_64BITS 0 CACHE BOOL "" FORCE)
    set(SCRUTINY_DATALOGGING_MAX_SIGNAL 16 CACHE STRING "" FORCE)
    set(SCRUTINY_DATALOGGING_BUFFER_32BITS 0 CACHE BOOL "" FORCE)
    set(SCRUTINY_SUPPORT_PROTECTED_REGIONS 1 CACHE BOOL "" FORCE)
    set(SCRUTINY_BUILD_CWRAPPER 0 CACHE BOOL "" FORCE)
    FetchContent_Declare(
        scrutiny-embedded
        GIT_REPOSITORY https://github.com/scrutinydebugger/scrutiny-embedded.git
        GIT_TAG        cc7e7705c037caaa24d1c5be122e931200e79003 # v0.7.2
        GIT_SHALLOW    FALSE
    )
    FetchContent_MakeAvailable(scrutiny-embedded)
    find_program(SCRUTINY_CLI scrutiny)
    find_program(ARM_CPPFILT arm-none-eabi-c++filt)
    if(NOT SCRUTINY_CLI OR NOT ARM_CPPFILT)
        message(WARNING "ENABLE_SCRUTINY is ON but scrutiny CLI (${SCRUTINY_CLI}) or arm-none-eabi-c++filt (${ARM_CPPFILT}) not found; SFD generation will be skipped")
    endif()
endif()

set(DRIVERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers)
set(SHARED_DEVICE_DRIVERS_SRC
    ${DRIVERS_DIR}/TMC2240-Driver/driver/src/TMC2240.c
    ${DRIVERS_DIR}/TMC2240-Driver/driver/src/tmc2240_hal.c
    ${DRIVERS_DIR}/TMC2240-Driver/driver/src/CRC.c
    ${DRIVERS_DIR}/TMC2240-Driver/driver/src/Functions.c
    ${DRIVERS_DIR}/AS5600/src/as5600.c
    ${DRIVERS_DIR}/AS5600/ports/stm32_hal/as5600_stm32_hal.c
    ${DRIVERS_DIR}/SEGGER_RTT/SEGGER_RTT.c
    ${DRIVERS_DIR}/SEGGER_RTT/SEGGER_RTT_printf.c
)
set(SHARED_DEVICE_DRIVERS_INC
    ${DRIVERS_DIR}/TMC2240-Driver/driver/inc
    ${DRIVERS_DIR}/AS5600/include
    ${DRIVERS_DIR}/AS5600/ports/stm32_hal
    ${DRIVERS_DIR}/SEGGER_RTT
)

# ---------------------------------------------------------------------------
# App Profile 1: motion_diagnostics (Dual-Axis Binary Diagnostics & Vision Executor)
# ---------------------------------------------------------------------------
set(DIAG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Apps/motion_diagnostics)
set(DIAG_SOURCES
    ${DIAG_DIR}/Src/crc16_ccitt.c
    ${DIAG_DIR}/Src/vision_frame.c
    ${DIAG_DIR}/Src/encoder_unwrap.c
    ${DIAG_DIR}/Src/motion_units.c
    ${DIAG_DIR}/Src/motion_config.c
    ${DIAG_DIR}/Src/diag_record.c
    ${DIAG_DIR}/Src/motion_profile.c
    ${DIAG_DIR}/Src/step_engine.c
    ${DIAG_DIR}/Src/encoder_sampler.c
    ${DIAG_DIR}/Src/driver_evidence.c
    ${DIAG_DIR}/Src/uart_transport.c
    ${DIAG_DIR}/Src/homing.c
    ${DIAG_DIR}/Src/motion_executor.c
    ${DIAG_DIR}/Src/command_dispatch.c
    ${DIAG_DIR}/Src/motion_board.c
    ${DIAG_DIR}/Src/console.c
)

# ---------------------------------------------------------------------------
# App Profile 2: stallguard_cli (Interactive StallGuard4 Homing, Soft/Bounce CLI)
# ---------------------------------------------------------------------------
set(SG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Apps/stallguard_cli)
set(SG_SOURCES
    ${SG_DIR}/Src/app.c
    ${SG_DIR}/Src/app_config.c
    ${SG_DIR}/Src/axis_ctrl.c
    ${SG_DIR}/Src/axis_hw.c
    ${SG_DIR}/Src/cmd_parse.c
    ${SG_DIR}/Src/console.c
    ${SG_DIR}/Src/enc_track.c
    ${SG_DIR}/Src/encoder.c
    ${SG_DIR}/Src/motion_profile.c
    ${SG_DIR}/Src/stall_detect.c
    ${SG_DIR}/Src/stepgen.c
    ${SG_DIR}/Src/tmc_axis.c
    ${SG_DIR}/Src/tmc_calc.c
)
set_property(SOURCE ${SG_SOURCES} APPEND PROPERTY COMPILE_OPTIONS
    -Wextra -Wshadow -Wdouble-promotion -Wundef -fno-math-errno)
set_property(SOURCE ${SG_DIR}/Src/stepgen.c ${SG_DIR}/Src/motion_profile.c
    APPEND PROPERTY COMPILE_OPTIONS -O2)

# ---------------------------------------------------------------------------
# App Profile 3: z_bounce_emulation (Z-Axis Bounce Demo + VisionUart Protocol)
# ---------------------------------------------------------------------------
set(ZB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Apps/z_bounce_emulation)
set(ZB_SOURCES
    ${ZB_DIR}/Src/z_bounce_app.c
    ${ZB_DIR}/Src/vision_uart_protocol.c
)

# ---------------------------------------------------------------------------
# App Profile 4 & Portable Module: usb_cdc (TinyUSB USB_OTG_HS CDC Transport)
# ---------------------------------------------------------------------------
include(${CMAKE_CURRENT_SOURCE_DIR}/Apps/usb_cdc/usb_cdc_module.cmake)
set(USB_CDC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Apps/usb_cdc)
set(USB_CDC_DEMO_SOURCES
    ${USB_CDC_DIR}/Src/usb_cdc_app.c
)

# ---------------------------------------------------------------------------
# App Profile 5: motion_characterization (Passive-by-Default Bench Metrology)
# ---------------------------------------------------------------------------
set(MC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Apps/motion_characterization)
set(MC_SOURCES
    ${MC_DIR}/Src/mc_core.c
    ${MC_DIR}/Src/mc_bench.c
    ${SG_DIR}/Src/app_config.c
    ${SG_DIR}/Src/tmc_calc.c
    ${SG_DIR}/Src/tmc_axis.c
    ${SG_DIR}/Src/motion_profile.c
    ${SG_DIR}/Src/stepgen.c
    ${SG_DIR}/Src/cmd_parse.c
    ${DIAG_DIR}/Src/encoder_sampler.c
    ${DIAG_DIR}/Src/encoder_unwrap.c
)

# ---------------------------------------------------------------------------
# App Profile 6: as5600_tuner (Dedicated Passive AS5600 Alignment & Filter Tuner)
# ---------------------------------------------------------------------------
set(AS5600_TUNER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Apps/as5600_tuner)
set(AS5600_TUNER_SOURCES
    ${AS5600_TUNER_DIR}/Src/as5600_tuner_app.c
    ${SG_DIR}/Src/cmd_parse.c
    ${DIAG_DIR}/Src/encoder_unwrap.c
)
if(ENABLE_SCRUTINY)
    list(APPEND AS5600_TUNER_SOURCES ${AS5600_TUNER_DIR}/Src/as5600_scrutiny_bridge.cpp)
endif()

# Helper function to configure any firmware executable target
function(configure_firmware_target target_name app_profile)
    if(NOT TARGET ${target_name})
        add_executable(${target_name} ${MX_Application_Src})
        target_link_libraries(${target_name} ${MX_LINK_LIBS})
        set_target_properties(${target_name} PROPERTIES
            ADDITIONAL_CLEAN_FILES ${target_name}.map)
    endif()

    if(app_profile STREQUAL "motion_diagnostics")
        set(app_srcs ${DIAG_SOURCES})
        set(app_inc  ${DIAG_DIR}/Inc)
        target_compile_definitions(${target_name} PRIVATE
            MOTION_BOUNDS_CONFIRMED=$<BOOL:${MOTION_DIAG_BOUNDS_CONFIRMED}>
            MOTION_CLAMP_X=$<BOOL:${MOTION_DIAG_CLAMP_X}>
            MOTION_MSCNT_QUALIFIED=$<BOOL:${MOTION_DIAG_MSCNT_QUALIFIED}>
            MOTION_SAFE_Z_01MM=${MOTION_DIAG_SAFE_Z_01MM}
            MOTION_PICK_DEPTH_01MM=${MOTION_DIAG_PICK_DEPTH_01MM}
            MOTION_X_DIR_SIGN=${MOTION_DIAG_X_DIR_SIGN}
            MOTION_X_ENCODER_SIGN=${MOTION_DIAG_X_ENCODER_SIGN}
            MOTION_Z_DIR_SIGN=${MOTION_DIAG_Z_DIR_SIGN}
            MOTION_Z_ENCODER_SIGN=${MOTION_DIAG_Z_ENCODER_SIGN}
        )
    elseif(app_profile STREQUAL "stallguard_cli")
        set(app_srcs ${SG_SOURCES})
        set(app_inc  ${SG_DIR}/Inc)
        add_usb_cdc_module(${target_name})
        target_compile_definitions(${target_name} PRIVATE USE_USB_CDC=1)
        target_link_options(${target_name} PRIVATE -u _printf_float)
    elseif(app_profile STREQUAL "z_bounce_emulation")
        set(app_srcs ${ZB_SOURCES})
        set(app_inc  ${ZB_DIR}/Inc)
        target_link_options(${target_name} PRIVATE -u _printf_float)
    elseif(app_profile STREQUAL "usb_cdc")
        set(app_srcs ${USB_CDC_DEMO_SOURCES})
        set(app_inc  ${USB_CDC_DIR}/Inc)
        add_usb_cdc_module(${target_name})
    elseif(app_profile STREQUAL "motion_characterization")
        set(app_srcs ${MC_SOURCES})
        set(app_inc
            ${MC_DIR}/Inc
            ${SG_DIR}/Inc
            ${DIAG_DIR}/Inc
        )
        add_usb_cdc_module(${target_name})
        target_compile_definitions(${target_name} PRIVATE
            USE_USB_CDC=1
            STEPGEN_CHARACTERIZATION=1
        )
    elseif(app_profile STREQUAL "as5600_tuner")
        set(app_srcs ${AS5600_TUNER_SOURCES})
        set(app_inc
            ${AS5600_TUNER_DIR}/Inc
            ${SG_DIR}/Inc
            ${DIAG_DIR}/Inc
        )
        add_usb_cdc_module(${target_name})
        target_compile_definitions(${target_name} PRIVATE USE_USB_CDC=1)
        if(ENABLE_SCRUTINY)
            target_compile_definitions(${target_name} PRIVATE ENABLE_SCRUTINY=1)
            # Keep DWARF-4 debug symbols even in Release so scrutiny elf2varmap can extract variables.
            target_compile_options(${target_name} PRIVATE -gdwarf-4)
            target_link_libraries(${target_name} scrutiny-embedded)
        endif()
    else()
        message(FATAL_ERROR "Unknown FIRMWARE_APP profile: ${app_profile}")
    endif()

    target_sources(${target_name} PRIVATE
        ${app_srcs}
        ${SHARED_DEVICE_DRIVERS_SRC}
    )
    target_include_directories(${target_name} PRIVATE
        ${app_inc}
        ${SHARED_DEVICE_DRIVERS_INC}
    )
    target_compile_options(${target_name} PRIVATE -Wall -Wextra)
    target_link_libraries(${target_name} stm32cubemx m)

    if(app_profile STREQUAL "as5600_tuner" AND ENABLE_SCRUTINY AND SCRUTINY_CLI AND ARM_CPPFILT)
        set(_sfd_work ${CMAKE_CURRENT_BINARY_DIR}/${target_name}_sfd_work)
        set(_sfd_out  ${CMAKE_CURRENT_BINARY_DIR}/${target_name}.sfd)
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E rm -rf ${_sfd_work}
            COMMAND ${CMAKE_COMMAND} -E make_directory ${_sfd_work}
            COMMAND ${SCRUTINY_CLI} elf2varmap $<TARGET_FILE:${target_name}> --cppfilt ${ARM_CPPFILT} --output ${_sfd_work}
            COMMAND ${SCRUTINY_CLI} get-firmware-id $<TARGET_FILE:${target_name}> --output ${_sfd_work}
            COMMAND ${SCRUTINY_CLI} tag-firmware-id $<TARGET_FILE:${target_name}> --inplace
            COMMAND ${SCRUTINY_CLI} make-metadata --output ${_sfd_work} --project-name "XZ-Manipulator-AS5600Tuner" --version "1.0.0" --author "XZ-Manipulator-Controls"
            COMMAND ${SCRUTINY_CLI} add-alias ${_sfd_work} --file ${AS5600_TUNER_DIR}/as5600_aliases.json
            COMMAND ${SCRUTINY_CLI} make-sfd ${_sfd_work} ${_sfd_out} --install
            COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target_name}> ${CMAKE_CURRENT_BINARY_DIR}/${target_name}.bin
            COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target_name}> ${CMAKE_CURRENT_BINARY_DIR}/${target_name}.hex
            COMMENT "Tagging Scrutiny firmware ID, generating/installing ${_sfd_out}, and building .bin/.hex for ${target_name}"
        )
    else()
        add_custom_command(TARGET ${target_name} POST_BUILD
            COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target_name}> ${CMAKE_CURRENT_BINARY_DIR}/${target_name}.bin
            COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target_name}> ${CMAKE_CURRENT_BINARY_DIR}/${target_name}.hex
            COMMENT "Building binary and intel hex artifacts for ${target_name} (${app_profile})"
        )
    endif()
endfunction()

# 1. Configure default target MotionFirmware according to FIRMWARE_APP
configure_firmware_target(${CMAKE_PROJECT_NAME} ${FIRMWARE_APP})

# 2. Always build dedicated named targets for all profiles side-by-side
configure_firmware_target(MotionFirmware_Diagnostics "motion_diagnostics")
configure_firmware_target(MotionFirmware_StallGuardCLI "stallguard_cli")
configure_firmware_target(MotionFirmware_ZBounceEmulation "z_bounce_emulation")
configure_firmware_target(MotionFirmware_UsbCdc "usb_cdc")
configure_firmware_target(MotionFirmware_Characterization "motion_characterization")
configure_firmware_target(MotionFirmware_AS5600Tuner "as5600_tuner")
