cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME MotionFirmware)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

set(APP_DIR     ${CMAKE_CURRENT_SOURCE_DIR}/App)
set(DRIVERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../Drivers)

# Application: homing / cycling logic (host tested in tests/) + target glue
set(APP_SOURCES
    ${APP_DIR}/Src/app.c
    ${APP_DIR}/Src/app_config.c
    ${APP_DIR}/Src/axis_ctrl.c
    ${APP_DIR}/Src/axis_hw.c
    ${APP_DIR}/Src/cmd_parse.c
    ${APP_DIR}/Src/console.c
    ${APP_DIR}/Src/enc_track.c
    ${APP_DIR}/Src/encoder.c
    ${APP_DIR}/Src/motion_profile.c
    ${APP_DIR}/Src/stall_detect.c
    ${APP_DIR}/Src/stepgen.c
    ${APP_DIR}/Src/tmc_axis.c
    ${APP_DIR}/Src/tmc_calc.c
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${APP_SOURCES}
    ${DRIVERS_DIR}/TMC2240-Driver/driver/src/TMC2240.c
    ${DRIVERS_DIR}/TMC2240-Driver/driver/src/tmc2240_hal.c
    ${DRIVERS_DIR}/TMC2240-Driver/driver/src/CRC.c
    ${DRIVERS_DIR}/TMC2240-Driver/driver/src/Functions.c
    ${DRIVERS_DIR}/AS5600/src/as5600.c
    ${DRIVERS_DIR}/AS5600/ports/stm32_hal/as5600_stm32_hal.c
    ${DRIVERS_DIR}/SEGGER_RTT/SEGGER_RTT.c
    ${DRIVERS_DIR}/SEGGER_RTT/SEGGER_RTT_printf.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${APP_DIR}/Inc
    ${DRIVERS_DIR}/TMC2240-Driver/driver/inc
    ${DRIVERS_DIR}/AS5600/include
    ${DRIVERS_DIR}/AS5600/ports/stm32_hal
    ${DRIVERS_DIR}/SEGGER_RTT
)

# Stricter warnings for the application code only (HAL/CMSIS stay as generated)
set_property(SOURCE ${APP_SOURCES} APPEND PROPERTY COMPILE_OPTIONS
    -Wextra -Wshadow -Wdouble-promotion -Wundef -fno-math-errno)
# The step ISR (stepgen + planner) stays fast even in Debug (-O0) builds
set_property(SOURCE ${APP_DIR}/Src/stepgen.c ${APP_DIR}/Src/motion_profile.c
    APPEND PROPERTY COMPILE_OPTIONS -O2)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    m
)

# newlib-nano: enable %f in printf (telemetry prints mm and mm/s)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

# Post-build: generate .bin and .hex
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex
    COMMENT "Building binary and intel hex artifacts"
)
