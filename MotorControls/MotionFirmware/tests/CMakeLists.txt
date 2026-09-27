# Host unit tests for the hardware-independent MotionFirmware modules.
#
#   cmake -S tests -B build-host -G Ninja -DCMAKE_C_COMPILER=clang
#   cmake --build build-host
#   ctest --test-dir build-host --output-on-failure
#
# The tests compile the real App/ sources (planner, StallGuard4 detector,
# encoder unwrap, TMC2240 math, console parser, axis controller and the
# default configuration) and run the homing/cycle state machine against a
# simulated axis with hard stops, a StallGuard4 load model and an AS5600.
cmake_minimum_required(VERSION 3.22)
project(MotionFirmwareHostTests LANGUAGES C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

set(APP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../App)

set(STRICT_WARNINGS -Wall -Wextra -Wshadow -Wundef -Wstrict-prototypes -Werror)

add_library(motion_core STATIC
    ${APP_DIR}/Src/app_config.c
    ${APP_DIR}/Src/axis_ctrl.c
    ${APP_DIR}/Src/cmd_parse.c
    ${APP_DIR}/Src/enc_track.c
    ${APP_DIR}/Src/motion_profile.c
    ${APP_DIR}/Src/stall_detect.c
    ${APP_DIR}/Src/tmc_calc.c
)
target_include_directories(motion_core PUBLIC ${APP_DIR}/Inc)
if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
    target_compile_options(motion_core PRIVATE ${STRICT_WARNINGS} -Wdouble-promotion)
    target_link_libraries(motion_core PUBLIC m)
endif()

function(add_host_test name)
    add_executable(${name} ${ARGN})
    target_link_libraries(${name} PRIVATE motion_core)
    if(CMAKE_C_COMPILER_ID MATCHES "Clang|GNU")
        target_compile_options(${name} PRIVATE ${STRICT_WARNINGS})
    endif()
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES TIMEOUT 300)
endfunction()

add_host_test(test_motion_profile test_motion_profile.c)
add_host_test(test_units          test_units.c)
add_host_test(test_axis_ctrl      test_axis_ctrl.c sim_axis.c)
