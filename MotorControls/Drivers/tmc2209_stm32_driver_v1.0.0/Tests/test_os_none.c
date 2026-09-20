#include "tmc2209_os.h"
#include "fake_hal.h"
#include "test_support.h"
static test_counts_t counts;

int main(void)
{
    ++counts.tests;
    hal_reset();
    tmc2209_mutex_t mutex = &counts;
    CHECK(TMC2209_Os_MutexCreate(NULL) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Os_MutexCreate(&mutex) == TMC2209_OK && mutex == NULL);
    CHECK(TMC2209_Os_MutexLock(NULL, 0u) == TMC2209_OK);
    CHECK(TMC2209_Os_MutexLock(NULL, UINT32_MAX) == TMC2209_OK);
    CHECK(TMC2209_Os_MutexLock(&counts, 10u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Os_MutexUnlock(NULL) == TMC2209_OK);
    CHECK(TMC2209_Os_MutexUnlock(&counts) == TMC2209_ERR_PARAM);
    TMC2209_Os_MutexDelete(NULL); TMC2209_Os_MutexDelete(&counts);
    tmc2209_sem_t sem = &counts;
    CHECK(TMC2209_Os_SemCreate(NULL) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Os_SemCreate(&sem) == TMC2209_ERR_OS && sem == NULL);
    CHECK(TMC2209_Os_SemTake(NULL, 1u) == TMC2209_ERR_OS);
    CHECK(TMC2209_Os_SemGive(NULL) == TMC2209_ERR_OS);
    CHECK(TMC2209_Os_SemGiveFromISR(NULL) == TMC2209_ERR_OS);
    TMC2209_Os_SemDelete(NULL); TMC2209_Os_SemDelete(&counts);
    CHECK(TMC2209_Os_DelayMs(0u) == TMC2209_OK && hal_fake.delayed_ms == 0u);
    CHECK(TMC2209_Os_DelayMs(1u) == TMC2209_OK && hal_fake.delayed_ms == 1u);
    CHECK(TMC2209_Os_DelayMs(UINT32_MAX) == TMC2209_ERR_PARAM);
    hal_fake.tick = UINT32_MAX;
    CHECK(TMC2209_Os_GetTickMs() == UINT32_MAX);
    CHECK(TMC2209_Os_DelayMs(1u) == TMC2209_OK);
    CHECK(TMC2209_Os_GetTickMs() == 0u);
    SUMMARY("Production bare-metal OS");
    return EXIT_SUCCESS;
}
