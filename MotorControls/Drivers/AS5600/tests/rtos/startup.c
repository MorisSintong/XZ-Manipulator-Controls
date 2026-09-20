#include "mps2_device.h"
#include "test_support.h"

extern uint32_t __data_load;
extern uint32_t __data_start;
extern uint32_t __data_end;
extern uint32_t __bss_start;
extern uint32_t __bss_end;
extern uint32_t __stack_top;
extern int main(void);
extern void SVC_Handler(void);
extern void PendSV_Handler(void);
extern void SysTick_Handler(void);

uint32_t SystemCoreClock = 25000000U;

static void fault_handler(void)
{
    __disable_irq();
    test_puts("FAIL: Cortex-M exception ");
    test_put_u32(__get_IPSR());
    test_puts("\n");
    test_exit(1U);
}

void Reset_Handler(void)
{
    uint32_t *source = &__data_load;
    uint32_t *destination;

    for (destination = &__data_start; destination < &__data_end; ++destination)
    {
        *destination = *source++;
    }
    for (destination = &__bss_start; destination < &__bss_end; ++destination)
    {
        *destination = 0U;
    }
    SCB->VTOR = 0U;
    SCB->CPACR |= 0x00F00000U;
    __DSB();
    __ISB();
    (void)main();
    test_exit(1U);
}

__attribute__((used, section(".vectors")))
void (*const vectors[16])(void) = {
    (void (*)(void))&__stack_top,
    Reset_Handler,
    fault_handler,
    fault_handler,
    fault_handler,
    fault_handler,
    fault_handler,
    0, 0, 0, 0,
    SVC_Handler,
    fault_handler,
    0,
    PendSV_Handler,
    SysTick_Handler
};
