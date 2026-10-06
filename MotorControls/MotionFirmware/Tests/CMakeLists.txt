cmake_minimum_required(VERSION 3.20)
project(motion_app_host_tests LANGUAGES C)
enable_testing()
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

include(CheckCSourceCompiles)
set(CMAKE_REQUIRED_FLAGS "-fsanitize=address,undefined")
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
check_c_source_compiles("int main(void) { return 0; }" MOTION_SANITIZERS_SUPPORTED)
unset(CMAKE_TRY_COMPILE_TARGET_TYPE)
unset(CMAKE_REQUIRED_FLAGS)

add_library(motion_app STATIC
    ../App/Src/crc16_ccitt.c
    ../App/Src/vision_frame.c
    ../App/Src/encoder_unwrap.c
    ../App/Src/motion_units.c
    ../App/Src/motion_config.c
    ../App/Src/diag_record.c
    ../App/Src/motion_profile.c
    ../App/Src/step_engine.c
    ../App/Src/encoder_sampler.c
    ../App/Src/driver_evidence.c
    ../App/Src/uart_transport.c
    ../App/Src/homing.c
    ../App/Src/motion_executor.c
    ../App/Src/command_dispatch.c)
target_link_libraries(motion_app PUBLIC m)
target_include_directories(motion_app PUBLIC ../App/Inc)
target_compile_options(motion_app PUBLIC
    -Wall -Wextra -Werror -Wconversion -Wsign-conversion -UNDEBUG)
if(MOTION_SANITIZERS_SUPPORTED)
    target_compile_options(motion_app PUBLIC
        -fsanitize=address,undefined -fno-omit-frame-pointer
        -fno-sanitize-recover=all)
    target_link_options(motion_app PUBLIC -fsanitize=address,undefined)
endif()

add_library(motion_test_support OBJECT test_support.c)
target_link_libraries(motion_test_support PRIVATE motion_app)

foreach(module IN ITEMS crc parser unwrap units motion_profile diag_record
                        step_engine encoder_sampler motion_config uart_transport
                        motion_executor homing crash_suppression)
    add_executable(test_${module} test_${module}.c $<TARGET_OBJECTS:motion_test_support>)
    target_link_libraries(test_${module} PRIVATE motion_app)
    add_test(NAME motion_${module} COMMAND test_${module})
    set_tests_properties(motion_${module} PROPERTIES TIMEOUT 30)
endforeach()

add_executable(fixture_writer fixture_writer.c $<TARGET_OBJECTS:motion_test_support>)
add_executable(test_qa_fixes test_qa_fixes.c $<TARGET_OBJECTS:motion_test_support>)
target_link_libraries(test_qa_fixes PRIVATE motion_app)
add_executable(test_uart_recovery test_uart_recovery.c $<TARGET_OBJECTS:motion_test_support>)
target_link_libraries(test_uart_recovery PRIVATE motion_app)
add_test(NAME motion_uart_recovery_order COMMAND test_uart_recovery 1)
add_test(NAME motion_uart_recovery_start COMMAND test_uart_recovery 2)
foreach(item IN ITEMS 1 3 6 7 9)
    add_test(NAME motion_qa_${item} COMMAND test_qa_fixes ${item})
    set_tests_properties(motion_qa_${item} PROPERTIES TIMEOUT 30)
endforeach()
target_link_libraries(fixture_writer PRIVATE motion_app)
add_custom_target(generate_diag_fixtures
    COMMAND fixture_writer "${CMAKE_CURRENT_SOURCE_DIR}/fixtures/diag_records_v1.json"
    DEPENDS fixture_writer)
add_test(NAME motion_golden_fixtures
    COMMAND ${CMAKE_COMMAND}
        "-DWRITER=$<TARGET_FILE:fixture_writer>"
        "-DFIXTURE=${CMAKE_CURRENT_SOURCE_DIR}/fixtures/diag_records_v1.json"
        -P "${CMAKE_CURRENT_SOURCE_DIR}/check_fixtures.cmake")

get_property(motion_registered_tests DIRECTORY PROPERTY TESTS)
set_tests_properties(${motion_registered_tests} PROPERTIES TIMEOUT 30)
