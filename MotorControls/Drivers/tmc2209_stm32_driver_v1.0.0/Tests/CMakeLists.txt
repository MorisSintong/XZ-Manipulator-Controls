cmake_minimum_required(VERSION 3.20)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(tmc2209_tests LANGUAGES C)
    enable_testing()
endif()

set(TMC2209_SRC "${CMAKE_CURRENT_SOURCE_DIR}/../Src")
set(TMC2209_CORE "${TMC2209_SRC}/tmc2209.c" "${TMC2209_SRC}/tmc2209_ll.c")

function(tmc2209_test name)
    add_executable(${name} ${ARGN})
    set_target_properties(${name} PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
    target_include_directories(${name} PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/../Inc" "${CMAKE_CURRENT_SOURCE_DIR}"
        "${CMAKE_CURRENT_SOURCE_DIR}/fakes")
    target_compile_definitions(${name} PRIVATE TMC2209_NO_CONF=1)
    target_compile_options(${name} PRIVATE -Wall -Wextra -Wpedantic -Werror
        -Wconversion -Wsign-conversion -Wshadow -Wformat=2 -Wundef -Wstrict-prototypes)
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES TIMEOUT 120)
endfunction()

tmc2209_test(tmc2209_protocol test_crc_datagram.c fakes/fake_core.c ${TMC2209_CORE})
foreach(variant IN ITEMS rtos none external none_external)
    tmc2209_test(tmc2209_core_${variant} test_driver_full.c fakes/fake_core.c ${TMC2209_CORE})
    if(variant MATCHES "^none")
        target_compile_definitions(tmc2209_core_${variant} PRIVATE TMC2209_OS_NONE=1 TMC2209_OS_FREERTOS=0)
    else()
        target_compile_definitions(tmc2209_core_${variant} PRIVATE TMC2209_OS_NONE=0 TMC2209_OS_FREERTOS=1)
    endif()
    if(variant STREQUAL "external" OR variant STREQUAL "none_external")
        target_compile_definitions(tmc2209_core_${variant} PRIVATE TMC2209_OS_CREATE_MUTEX=0)
    endif()
endforeach()

add_test(NAME tmc2209_config_contracts COMMAND "${CMAKE_COMMAND}"
    "-DCOMPILER=${CMAKE_C_COMPILER}" "-DTEST_DIR=${CMAKE_CURRENT_SOURCE_DIR}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/check_config.cmake")
set_tests_properties(tmc2209_config_contracts PROPERTIES TIMEOUT 60)

if(TMC_ENABLE_SANITIZERS)
    add_executable(tmc2209_instrumentation instrumentation_probe.c)
    set_target_properties(tmc2209_instrumentation PROPERTIES C_STANDARD 11 C_STANDARD_REQUIRED YES C_EXTENSIONS NO)
    target_compile_options(tmc2209_instrumentation PRIVATE -Wall -Wextra -Wpedantic -Werror -Wundef)
    foreach(mode IN ITEMS asan ubsan)
        add_test(NAME tmc2209_instrumentation_${mode} COMMAND "${CMAKE_COMMAND}"
            "-DEXE=$<TARGET_FILE:tmc2209_instrumentation>" "-DMODE=${mode}"
            "-DLOG=${CMAKE_CURRENT_BINARY_DIR}/instrumentation-${mode}.log"
            -P "${CMAKE_CURRENT_SOURCE_DIR}/check_instrumentation.cmake")
        set_tests_properties(tmc2209_instrumentation_${mode} PROPERTIES TIMEOUT 30
            ENVIRONMENT "ASAN_OPTIONS=detect_leaks=0:halt_on_error=1;UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1")
    endforeach()
endif()

tmc2209_test(tmc2209_hal test_port_hal.c fakes/fake_hal.c "${TMC2209_SRC}/tmc2209_port_hal.c")
target_compile_definitions(tmc2209_hal PRIVATE TMC2209_TEST_MMIO=1 TMC2209_POLL_LIMIT=512)
tmc2209_test(tmc2209_os_none test_os_none.c fakes/fake_hal.c "${TMC2209_SRC}/tmc2209_os_none.c")
foreach(bits IN ITEMS 16 32)
    foreach(rate IN ITEMS 100 128 1000 1024 2000)
        set(name tmc2209_os_${bits}_${rate})
        tmc2209_test(${name} test_os_freertos.c fakes/fake_rtos.c "${TMC2209_SRC}/tmc2209_os_freertos.c")
        target_compile_definitions(${name} PRIVATE TMC2209_OS_NONE=0 TMC2209_OS_FREERTOS=1
            FAKE_TICK_BITS=${bits} FAKE_TICK_RATE=${rate})
    endforeach()
endforeach()
tmc2209_test(tmc2209_os_static test_os_freertos.c fakes/fake_rtos.c "${TMC2209_SRC}/tmc2209_os_freertos.c")
target_compile_definitions(tmc2209_os_static PRIVATE TMC2209_OS_NONE=0 TMC2209_OS_FREERTOS=1
    FAKE_TICK_BITS=16 FAKE_TICK_RATE=128 configSUPPORT_DYNAMIC_ALLOCATION=0)
foreach(variant IN ITEMS none rtos rtos_static)
    if(variant STREQUAL "none")
        set(backend "${TMC2209_SRC}/tmc2209_os_none.c")
    else()
        set(backend "${TMC2209_SRC}/tmc2209_os_freertos.c" fakes/fake_rtos.c)
    endif()
    tmc2209_test(tmc2209_integration_${variant} test_integration.c fakes/fake_hal.c
        ${TMC2209_CORE} "${TMC2209_SRC}/tmc2209_port_hal.c" ${backend})
    target_compile_definitions(tmc2209_integration_${variant} PRIVATE
        TMC2209_TEST_MMIO=1 TMC2209_POLL_LIMIT=512)
    if(variant MATCHES "^rtos")
        target_compile_definitions(tmc2209_integration_${variant} PRIVATE
            TMC2209_OS_NONE=0 TMC2209_OS_FREERTOS=1 FAKE_TICK_BITS=32 FAKE_TICK_RATE=1024)
    endif()
    if(variant STREQUAL "rtos_static")
        target_compile_definitions(tmc2209_integration_${variant} PRIVATE
            TMC2209_OS_CREATE_MUTEX=0 configSUPPORT_DYNAMIC_ALLOCATION=0)
    endif()
endforeach()
