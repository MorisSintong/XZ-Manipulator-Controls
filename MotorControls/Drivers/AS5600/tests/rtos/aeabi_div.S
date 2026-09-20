.syntax unified
.thumb
.section .text.__aeabi_uldivmod, "ax", %progbits
.global __aeabi_uldivmod
.type __aeabi_uldivmod, %function
.thumb_func
__aeabi_uldivmod:
    push {r4, lr}
    sub sp, sp, #16
    add r4, sp, #8
    str r4, [sp]
    bl test_unsigned_divide
    ldr r2, [sp, #8]
    ldr r3, [sp, #12]
    add sp, sp, #16
    pop {r4, pc}
.size __aeabi_uldivmod, . - __aeabi_uldivmod
