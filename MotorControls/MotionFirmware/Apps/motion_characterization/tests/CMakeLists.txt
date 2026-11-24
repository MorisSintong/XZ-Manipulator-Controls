cmake_minimum_required(VERSION 3.22)
project(MotionCharacterizationHostTests LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

set(APP_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/..)
set(SG_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/../../stallguard_cli)
set(DIAG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../motion_diagnostics)

set(STRICT_WARNINGS -Wall -Wextra -Wshadow -Wundef -Wstrict-prototypes -Werror)

add_library(mc_host_core STATIC
    ${APP_DIR}/Src/mc_core.c
    ${SG_DIR}/Src/cmd_parse.c
    ${SG_DIR}/Src/motion_profile.c
    ${DIAG_DIR}/Src/encoder_sampler.c
    ${DIAG_DIR}/Src/encoder_unwrap.c
)
target_include_directories(mc_host_core PUBLIC
    ${APP_DIR}/Inc
    ${SG_DIR}/Inc
    ${DIAG_DIR}/Inc
)
if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(mc_host_core PRIVATE ${STRICT_WARNINGS})
    target_link_libraries(mc_host_core PUBLIC m)
endif()

add_executable(test_mc_core test_mc_core.c)
target_link_libraries(test_mc_core PRIVATE mc_host_core)
if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(test_mc_core PRIVATE ${STRICT_WARNINGS})
endif()
add_test(NAME test_mc_core COMMAND test_mc_core)
set_tests_properties(test_mc_core PROPERTIES TIMEOUT 60)
