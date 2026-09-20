if(NOT DEFINED COMPILER OR NOT DEFINED TEST_DIR)
    message(FATAL_ERROR "Pass COMPILER and TEST_DIR")
endif()
set(inc "${TEST_DIR}/../Inc")
set(probe "${TEST_DIR}/header_probe.c")
set(cases
    "both_on|Select exactly one|-DTMC2209_OS_NONE=1|-DTMC2209_OS_FREERTOS=1"
    "both_off|Select exactly one|-DTMC2209_OS_NONE=0|-DTMC2209_OS_FREERTOS=0"
    "nonboolean|Select exactly one|-DTMC2209_OS_NONE=2"
    "mutex|must be 0 or 1|-DTMC2209_OS_CREATE_MUTEX=2"
    "tx_zero|timeouts must be finite|-DTMC2209_UART_TX_TIMEOUT_MS=0"
    "rx_infinite|timeouts must be finite|-DTMC2209_UART_RX_TIMEOUT_MS=4294967295u"
    "lock_infinite|timeouts must be finite|-DTMC2209_BUS_LOCK_TIMEOUT_MS=4294967295u"
    "bad_baud|Invalid TMC2209 transport|-DTMC2209_DEFAULT_BAUD=0"
    "bad_idle|Invalid TMC2209 transport|-DTMC2209_BUS_IDLE_US=1000001"
    "poll_zero|Invalid TMC2209 transport|-DTMC2209_POLL_LIMIT=0"
    "drain_zero|Invalid TMC2209 transport|-DTMC2209_RX_DRAIN_LIMIT=0"
    "old_gap|Obsolete reply delays|-DTMC2209_REPLY_GAP_US=70"
    "old_poll|Obsolete reply delays|-DTMC2209_REPLY_POLL_US=500"
    "optional_conf|Select exactly one|-I${TEST_DIR}/fakes/conf_probe")
set(completed 0)
foreach(case IN LISTS cases)
    string(REPLACE "|" ";" fields "${case}")
    list(POP_FRONT fields name expected)
    execute_process(COMMAND "${COMPILER}" -std=c11 -fsyntax-only "-I${inc}" ${fields} "${probe}"
        RESULT_VARIABLE result OUTPUT_VARIABLE out ERROR_VARIABLE err)
    if(result EQUAL 0 OR NOT "${err}" MATCHES "${expected}")
        message(FATAL_ERROR "${name}: expected deliberate configuration rejection, got ${result}\n${out}${err}")
    endif()
    math(EXPR completed "${completed}+1")
endforeach()
execute_process(COMMAND "${COMPILER}" -std=c11 -Wall -Wextra -Wpedantic -Werror -Wundef
    -fsyntax-only -DTMC2209_NO_CONF=1 "-I${TEST_DIR}/fakes/conf_probe" "-I${inc}" "${probe}"
    RESULT_VARIABLE result ERROR_VARIABLE err)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "NO_CONF override failed: ${err}")
endif()
execute_process(COMMAND "${COMPILER}" -x c++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -Wundef
    -fsyntax-only -DTMC2209_NO_CONF=1 "-I${inc}" "${probe}"
    RESULT_VARIABLE result ERROR_VARIABLE err)
if(NOT result EQUAL 0)
    message(FATAL_ERROR "C++ consumer header failed: ${err}")
endif()
message(STATUS "${completed} invalid configurations rejected; NO_CONF and C/C++ consumers passed")
