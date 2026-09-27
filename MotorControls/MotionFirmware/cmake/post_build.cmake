#
# Post-build commands for STM32 embedded projects
# Automatically produces .hex and .bin binaries and displays memory footprint
#
if(CMAKE_OBJCOPY AND CMAKE_SIZE)
    add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
        COMMENT "Generating .hex and .bin binaries & checking memory usage"
    )
endif()
