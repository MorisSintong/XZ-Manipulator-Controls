cmake_minimum_required(VERSION 3.24)
project(as5600_actual_freertos_test LANGUAGES C ASM)

if(NOT CMAKE_C_COMPILER_ID MATCHES "Clang" OR
   NOT CMAKE_C_COMPILER_TARGET STREQUAL "arm-none-eabi")
    message(FATAL_ERROR "Use -DCMAKE_TOOLCHAIN_FILE=clang-arm.cmake (bare-metal Arm Clang).")
endif()

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

include(FetchContent)
# Upstream license files stay with the unmodified sources in build\_deps.
FetchContent_Declare(cmsis6
    URL "https://codeload.github.com/ARM-software/CMSIS_6/tar.gz/b0bbb0423b278ca632cfe1474eb227961d835fd2"
    URL_HASH "SHA256=a6b7e1dc74427025ba0b64c86530fbca4da6509b8d6ff4a74a7354d17b1775c4"
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR "not-a-cmake-project")
FetchContent_Declare(cmsis_freertos
    URL "https://codeload.github.com/ARM-software/CMSIS-FreeRTOS/tar.gz/c7e7294e78a80be862340d8d50edd515ac3c41fd"
    URL_HASH "SHA256=536916cd40946c7f2e1e85277789ae5bd8d5bcb323a88451ad5218948dafabbd"
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
    SOURCE_SUBDIR "not-a-cmake-project")
FetchContent_MakeAvailable(cmsis6 cmsis_freertos)

get_filename_component(AS5600_ROOT "${CMAKE_CURRENT_SOURCE_DIR}\\..\\.." ABSOLUTE)
set(kernel "${cmsis_freertos_SOURCE_DIR}\\Source")
set(wrapper "${cmsis_freertos_SOURCE_DIR}\\CMSIS\\RTOS2\\FreeRTOS")
set(cmsis "${cmsis6_SOURCE_DIR}\\CMSIS")

add_library(rtos_headers INTERFACE)
target_include_directories(rtos_headers INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}\\freestanding"
    "${AS5600_ROOT}\\include"
    "${AS5600_ROOT}\\ports\\cmsis_rtos2")
target_include_directories(rtos_headers SYSTEM INTERFACE
    "${kernel}\\include"
    "${kernel}\\portable\\GCC\\ARM_CM4F"
    "${wrapper}\\Include"
    "${cmsis}\\Core\\Include"
    "${cmsis}\\RTOS2\\Include")

add_library(official_freertos STATIC
    "${kernel}\\tasks.c"
    "${kernel}\\queue.c"
    "${kernel}\\list.c"
    "${kernel}\\event_groups.c"
    "${kernel}\\portable\\GCC\\ARM_CM4F\\port.c"
    "${wrapper}\\Source\\cmsis_os2.c"
    "${cmsis}\\RTOS2\\Source\\os_systick.c")
target_link_libraries(official_freertos PUBLIC rtos_headers)

add_executable(as5600_rtos
    integration.c startup.c runtime.c aeabi_div.S
    "${AS5600_ROOT}\\src\\as5600.c"
    "${AS5600_ROOT}\\ports\\cmsis_rtos2\\as5600_cmsis_rtos2.c")
set_target_properties(as5600_rtos PROPERTIES SUFFIX ".elf")
target_link_libraries(as5600_rtos PRIVATE official_freertos)
target_compile_options(as5600_rtos PRIVATE
    "$<$<COMPILE_LANGUAGE:C>:-Wall;-Wextra;-Werror>")
target_link_options(as5600_rtos PRIVATE
    "-Wl,-T,${CMAKE_CURRENT_SOURCE_DIR}\\mps2.ld"
    "-Wl,-Map,${CMAKE_CURRENT_BINARY_DIR}\\as5600_rtos.map"
    "-Wl,--gc-sections")
set_property(TARGET as5600_rtos APPEND PROPERTY LINK_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}\\mps2.ld")

set(AS5600_QEMU "" CACHE FILEPATH "Path to qemu-system-arm.exe")
if(NOT AS5600_QEMU)
    find_program(AS5600_QEMU_FOUND NAMES qemu-system-arm)
    if(AS5600_QEMU_FOUND)
        set(AS5600_QEMU "${AS5600_QEMU_FOUND}")
    endif()
endif()
find_program(AS5600_PWSH NAMES pwsh)
if(AS5600_QEMU AND AS5600_PWSH)
    enable_testing()
    add_test(NAME actual_cmsis_freertos
        COMMAND "${AS5600_PWSH}" -NoProfile -File
            "${CMAKE_CURRENT_SOURCE_DIR}\\run.ps1"
            -Qemu "${AS5600_QEMU}"
            -Image "$<TARGET_FILE:as5600_rtos>"
            -LogPath "${CMAKE_CURRENT_BINARY_DIR}\\qemu.log"
            -TimeoutSeconds 30)
    set_tests_properties(actual_cmsis_freertos PROPERTIES TIMEOUT 40)
else()
    message(STATUS "Build only: set AS5600_QEMU and install PowerShell 7 to enable CTest.")
endif()
