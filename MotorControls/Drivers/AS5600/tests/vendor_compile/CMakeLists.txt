cmake_minimum_required(VERSION 3.24)
project(as5600_vendor_compile LANGUAGES C)

if(NOT CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR "Use the Cortex-M4 toolchain, not a host compiler")
endif()

include(FetchContent)

FetchContent_Declare(stm32_hal
    URL https://github.com/STMicroelectronics/stm32f4xx-hal-driver/archive/b6f0ed3829f3829eb358a2e7417d80bba1a42db7.zip
    URL_HASH SHA256=db48e624fd80cc87d1ac4c2797c3e8dbba87710297c39e99e9b6db6005d4d181
    DOWNLOAD_EXTRACT_TIMESTAMP FALSE
    SOURCE_SUBDIR validation-download-only
)
FetchContent_Declare(stm32_device
    URL https://github.com/STMicroelectronics/cmsis-device-f4/archive/0fa0e489e053fa1ca7790bb40b4d76458f64c55d.zip
    URL_HASH SHA256=c52b7e008d10c28d3a5edb664f88b466b07a6ac8a7a0b741504933a7a0bfb571
    DOWNLOAD_EXTRACT_TIMESTAMP FALSE
    SOURCE_SUBDIR validation-download-only
)
FetchContent_Declare(arm_cmsis
    URL https://github.com/ARM-software/CMSIS_6/archive/b0bbb0423b278ca632cfe1474eb227961d835fd2.zip
    URL_HASH SHA256=7187cce527139f4a3d62c249852b889cc74e5fad421d05d273fa5fe86a59de37
    DOWNLOAD_EXTRACT_TIMESTAMP FALSE
    SOURCE_SUBDIR validation-download-only
)
FetchContent_Declare(cmsis_freertos
    URL https://github.com/ARM-software/CMSIS-FreeRTOS/archive/c7e7294e78a80be862340d8d50edd515ac3c41fd.zip
    URL_HASH SHA256=5bf431cbe32a0a4591943e8b46829a238b32db28a20d2701a55700a2a5f9575c
    DOWNLOAD_EXTRACT_TIMESTAMP FALSE
    SOURCE_SUBDIR validation-download-only
)
FetchContent_MakeAvailable(stm32_hal stm32_device arm_cmsis cmsis_freertos)

get_filename_component(driver_root "../.." ABSOLUTE)

add_library(as5600_target_objects OBJECT
    "${driver_root}/src/as5600.c"
    "${driver_root}/ports/stm32_hal/as5600_stm32_hal.c"
    "${driver_root}/ports/cmsis_rtos2/as5600_cmsis_rtos2.c"
    "${driver_root}/examples/nucleo_f446re/as5600_example_baremetal.c"
    "${driver_root}/examples/nucleo_f446re/as5600_example_rtos.c"
)
set_target_properties(as5600_target_objects PROPERTIES
    C_STANDARD 99 C_STANDARD_REQUIRED ON C_EXTENSIONS OFF
)
target_compile_definitions(as5600_target_objects PRIVATE STM32F446xx USE_HAL_DRIVER)
target_include_directories(as5600_target_objects PRIVATE
    "${driver_root}/include"
    "${driver_root}/ports/stm32_hal"
    "${driver_root}/ports/cmsis_rtos2"
    "${driver_root}/examples/nucleo_f446re"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_include_directories(as5600_target_objects SYSTEM PRIVATE
    "${stm32_hal_SOURCE_DIR}/Inc"
    "${stm32_device_SOURCE_DIR}/Include"
    "${arm_cmsis_SOURCE_DIR}/CMSIS/Core/Include"
    "${arm_cmsis_SOURCE_DIR}/CMSIS/RTOS2/Include"
    "${cmsis_freertos_SOURCE_DIR}/Source/include"
    "${cmsis_freertos_SOURCE_DIR}/Source/portable/GCC/ARM_CM4F"
)
target_compile_options(as5600_target_objects PRIVATE
    -Wall -Wextra -Wpedantic -Werror -Wconversion -Wsign-conversion
    -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wundef
    -Wdouble-promotion -Wformat=2 -fno-common -fstack-usage -Os
)
