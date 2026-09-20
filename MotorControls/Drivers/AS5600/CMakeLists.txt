cmake_minimum_required(VERSION 3.22)
project(as5600 VERSION 1.0.0 LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(as5600_test_default OFF)
if(PROJECT_IS_TOP_LEVEL AND NOT CMAKE_CROSSCOMPILING)
    set(as5600_test_default ON)
endif()

option(AS5600_BUILD_TESTS "Build host regression tests" ${as5600_test_default})
option(AS5600_BUILD_STM32_HAL "Build the STM32F4 HAL adapter" OFF)
option(AS5600_BUILD_CMSIS_RTOS2 "Build the CMSIS-RTOS2 adapter" OFF)
option(AS5600_ENABLE_SANITIZERS "Enable host address/undefined sanitizers" OFF)
option(AS5600_ENABLE_COVERAGE "Enable Clang source-based coverage" OFF)
set(AS5600_VENDOR_INCLUDE_DIRS "" CACHE STRING "HAL/CMSIS/application include directories")
set(AS5600_STM32_DEVICE "STM32F446xx" CACHE STRING "STM32F4 device preprocessor symbol")

function(as5600_strict_c target)
    if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(${target} PRIVATE
            -Wall -Wextra -Wpedantic -Werror -Wconversion -Wsign-conversion
            -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wundef
            -Wdouble-promotion -Wformat=2 -fno-common
        )
    elseif(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX)
    endif()
endfunction()

if(AS5600_ENABLE_SANITIZERS)
    if(CMAKE_CROSSCOMPILING OR NOT CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
        message(FATAL_ERROR "Sanitizers require a supported host Clang/GCC toolchain")
    endif()
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer)
    add_link_options(-fsanitize=address,undefined)
endif()

if(AS5600_ENABLE_COVERAGE)
    if(CMAKE_CROSSCOMPILING OR NOT CMAKE_C_COMPILER_ID MATCHES "Clang")
        message(FATAL_ERROR "Coverage requires host Clang")
    endif()
    add_compile_options(-fprofile-instr-generate -fcoverage-mapping)
    add_link_options(-fprofile-instr-generate)
endif()

add_library(as5600 STATIC src/as5600.c)
add_library(as5600::core ALIAS as5600)
target_include_directories(as5600 PUBLIC include)
as5600_strict_c(as5600)

if(AS5600_BUILD_STM32_HAL)
    add_library(as5600_stm32_hal STATIC ports/stm32_hal/as5600_stm32_hal.c)
    target_include_directories(as5600_stm32_hal PUBLIC ports/stm32_hal)
    target_include_directories(as5600_stm32_hal SYSTEM PUBLIC ${AS5600_VENDOR_INCLUDE_DIRS})
    target_compile_definitions(as5600_stm32_hal PUBLIC ${AS5600_STM32_DEVICE} USE_HAL_DRIVER)
    target_link_libraries(as5600_stm32_hal PUBLIC as5600)
    as5600_strict_c(as5600_stm32_hal)
endif()

if(AS5600_BUILD_CMSIS_RTOS2)
    add_library(as5600_cmsis_rtos2 STATIC ports/cmsis_rtos2/as5600_cmsis_rtos2.c)
    target_include_directories(as5600_cmsis_rtos2 PUBLIC ports/cmsis_rtos2)
    target_include_directories(as5600_cmsis_rtos2 SYSTEM PUBLIC ${AS5600_VENDOR_INCLUDE_DIRS})
    target_link_libraries(as5600_cmsis_rtos2 PUBLIC as5600)
    as5600_strict_c(as5600_cmsis_rtos2)
endif()

if(AS5600_BUILD_TESTS)
    enable_language(CXX)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
    set(CMAKE_CXX_EXTENSIONS OFF)
    enable_testing()
    find_package(Threads REQUIRED)

    add_executable(as5600_test_core tests/test_core.c)
    target_link_libraries(as5600_test_core PRIVATE as5600)

    add_executable(as5600_test_ports
        tests/test_ports.c
        ports/stm32_hal/as5600_stm32_hal.c
        ports/cmsis_rtos2/as5600_cmsis_rtos2.c
    )
    target_include_directories(as5600_test_ports PRIVATE
        tests/fakes ports/stm32_hal ports/cmsis_rtos2
    )
    target_link_libraries(as5600_test_ports PRIVATE as5600)
    as5600_strict_c(as5600_test_ports)

    add_executable(as5600_test_concurrency tests/test_concurrency.cpp)
    target_link_libraries(as5600_test_concurrency PRIVATE as5600 Threads::Threads)

    foreach(target IN ITEMS as5600_test_core as5600_test_concurrency)
        if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
            target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
        endif()
    endforeach()

    add_test(NAME core COMMAND as5600_test_core)
    add_test(NAME ports COMMAND as5600_test_ports)
    add_test(NAME concurrency COMMAND as5600_test_concurrency)
    set_tests_properties(core ports concurrency PROPERTIES TIMEOUT 90)

    if(AS5600_ENABLE_COVERAGE)
        file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/profiles")
        foreach(test_name IN ITEMS core ports concurrency)
            set_tests_properties(${test_name} PROPERTIES
                ENVIRONMENT "LLVM_PROFILE_FILE=${CMAKE_CURRENT_BINARY_DIR}/profiles/${test_name}.profraw"
            )
        endforeach()
    endif()
endif()
