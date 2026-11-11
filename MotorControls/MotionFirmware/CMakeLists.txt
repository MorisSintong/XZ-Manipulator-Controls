cmake_minimum_required(VERSION 3.22)

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Select which firmware profile is built as the primary "MotionFirmware" (.elf/.hex/.bin)
# Options: "motion_diagnostics" | "stallguard_cli" | "z_bounce_emulation"
set(FIRMWARE_APP "motion_diagnostics" CACHE STRING
    "Default firmware app profile for MotionFirmware.{elf,hex,bin}")
set_property(CACHE FIRMWARE_APP PROPERTY STRINGS
    "motion_diagnostics" "stallguard_cli" "z_bounce_emulation")

set(CMAKE_PROJECT_NAME MotionFirmware)
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

project(${CMAKE_PROJECT_NAME})
message(STATUS "Build type: ${CMAKE_BUILD_TYPE} | Default FIRMWARE_APP: ${FIRMWARE_APP}")

enable_language(C ASM)

# Default executable (used by flash scripts, debuggers, and Renode runner)
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated Core + HAL sources
add_subdirectory(cmake/stm32cubemx)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

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
# App Profile 3: z_bounce_emulation (Z-Axis Bounce Demo + Renode UART Emulation)
# ---------------------------------------------------------------------------
set(ZB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Apps/z_bounce_emulation)
set(ZB_SOURCES
    ${ZB_DIR}/Src/z_bounce_app.c
    ${ZB_DIR}/Src/vision_uart_protocol.c
)

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
    elseif(app_profile STREQUAL "stallguard_cli")
        set(app_srcs ${SG_SOURCES})
        set(app_inc  ${SG_DIR}/Inc)
        target_link_options(${target_name} PRIVATE -u _printf_float)
    elseif(app_profile STREQUAL "z_bounce_emulation")
        set(app_srcs ${ZB_SOURCES})
        set(app_inc  ${ZB_DIR}/Inc)
        target_link_options(${target_name} PRIVATE -u _printf_float)
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

    add_custom_command(TARGET ${target_name} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target_name}> ${CMAKE_CURRENT_BINARY_DIR}/${target_name}.bin
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target_name}> ${CMAKE_CURRENT_BINARY_DIR}/${target_name}.hex
        COMMENT "Building binary and intel hex artifacts for ${target_name} (${app_profile})"
    )
endfunction()

# 1. Configure default target MotionFirmware according to FIRMWARE_APP
configure_firmware_target(${CMAKE_PROJECT_NAME} ${FIRMWARE_APP})

# 2. Always build dedicated named targets for all three profiles side-by-side
configure_firmware_target(MotionFirmware_Diagnostics "motion_diagnostics")
configure_firmware_target(MotionFirmware_StallGuardCLI "stallguard_cli")
configure_firmware_target(MotionFirmware_ZBounceEmulation "z_bounce_emulation")
