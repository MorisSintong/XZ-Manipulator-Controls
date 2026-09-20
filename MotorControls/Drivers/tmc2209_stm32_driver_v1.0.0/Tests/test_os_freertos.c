#include "tmc2209_os.h"
#include "fake_rtos.h"
#include "test_support.h"
static test_counts_t counts;

static TickType_t expected_ticks(uint32_t ms)
{
    uint64_t whole = ((uint64_t)ms / 1000u) * FAKE_TICK_RATE;
    uint64_t fraction = ((uint64_t)ms % 1000u) * FAKE_TICK_RATE;
    uint64_t result = whole + fraction / 1000u + (fraction % 1000u != 0u ? 1u : 0u);
    uint64_t limit = ((UINT64_C(1) << FAKE_TICK_BITS) - 2u);
    return (TickType_t)(result > limit ? limit : result);
}

static void test_create_delete(void)
{
    rtos_reset();
    tmc2209_mutex_t mutex = &counts;
    tmc2209_sem_t sem = &counts;
    CHECK(TMC2209_Os_MutexCreate(NULL) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Os_SemCreate(NULL) == TMC2209_ERR_PARAM);
    rtos_fake.allocation_fails = pdTRUE;
    CHECK(TMC2209_Os_MutexCreate(&mutex) == TMC2209_ERR_OS && mutex == NULL);
    CHECK(TMC2209_Os_SemCreate(&sem) == TMC2209_ERR_OS && sem == NULL);
    rtos_fake.allocation_fails = pdFALSE;
#if configSUPPORT_DYNAMIC_ALLOCATION
    CHECK(TMC2209_Os_MutexCreate(&mutex) == TMC2209_OK);
    CHECK(TMC2209_Os_SemCreate(&sem) == TMC2209_OK);
    CHECK(((SemaphoreHandle_t)mutex)->available == pdTRUE);
    CHECK(((SemaphoreHandle_t)sem)->available == pdFALSE);
    TMC2209_Os_MutexDelete(mutex); TMC2209_Os_SemDelete(sem);
    CHECK(rtos_fake.allocations == 2u && rtos_fake.deletions == 2u);
#else
    CHECK(TMC2209_Os_MutexCreate(&mutex) == TMC2209_ERR_OS);
    CHECK(TMC2209_Os_SemCreate(&sem) == TMC2209_ERR_OS);
    CHECK(rtos_fake.allocations == 0u);
    fake_semaphore_t external = {pdTRUE, pdTRUE, pdTRUE};
    TMC2209_Os_MutexDelete(&external);
    external.allocated = pdTRUE;
    TMC2209_Os_SemDelete(&external);
#endif
    unsigned deletes = rtos_fake.deletions;
    TMC2209_Os_MutexDelete(NULL); TMC2209_Os_SemDelete(NULL);
    CHECK(rtos_fake.deletions == deletes);
}

static void test_waits_and_failures(void)
{
    rtos_reset();
    fake_semaphore_t mutex = {pdTRUE, pdTRUE, pdTRUE};
    fake_semaphore_t sem = {pdTRUE, pdFALSE, pdFALSE};
    CHECK(TMC2209_Os_MutexLock(NULL, 0u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Os_MutexUnlock(NULL) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Os_SemTake(NULL, 0u) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Os_SemGive(NULL) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Os_SemGiveFromISR(NULL) == TMC2209_ERR_PARAM);
    CHECK(TMC2209_Os_MutexLock(&mutex, 1u) == TMC2209_OK);
    CHECK(rtos_fake.last_wait == expected_ticks(1u));
    CHECK(TMC2209_Os_MutexLock(&mutex, 1u) == TMC2209_ERR_TIMEOUT);
    rtos_fake.give_fails = pdTRUE;
    CHECK(TMC2209_Os_MutexUnlock(&mutex) == TMC2209_ERR_OS);
    rtos_fake.give_fails = pdFALSE;
    CHECK(TMC2209_Os_MutexUnlock(&mutex) == TMC2209_OK);
    CHECK(TMC2209_Os_SemTake(&sem, 1u) == TMC2209_ERR_TIMEOUT);
    CHECK(TMC2209_Os_SemGive(&sem) == TMC2209_OK);
    CHECK(TMC2209_Os_SemGive(&sem) == TMC2209_ERR_OS);
    CHECK(TMC2209_Os_SemTake(&sem, 1u) == TMC2209_OK);
    rtos_fake.isr_fails = pdTRUE;
    CHECK(TMC2209_Os_SemGiveFromISR(&sem) == TMC2209_ERR_OS);
    rtos_fake.isr_fails = pdFALSE;
    CHECK(TMC2209_Os_SemGiveFromISR(&sem) == TMC2209_OK);
    CHECK(rtos_fake.yielded == pdFALSE);
    CHECK(TMC2209_Os_SemTake(&sem, 0u) == TMC2209_OK);
    rtos_fake.isr_woken = pdTRUE;
    CHECK(TMC2209_Os_SemGiveFromISR(&sem) == TMC2209_OK);
    CHECK(rtos_fake.yielded == pdTRUE && rtos_fake.yields == 3u);
    const uint32_t times[] = {0u, 1u, 2u, 9u, 10u, 11u, 999u, 1000u, 1001u,
                              65534u, 65535u, 65536u, 65535000u, UINT32_MAX - 1u, UINT32_MAX};
    for (size_t i = 0u; i < sizeof(times) / sizeof(times[0]); ++i) {
        CHECK(TMC2209_Os_MutexLock(&mutex, times[i]) == TMC2209_OK);
        CHECK(rtos_fake.last_wait == expected_ticks(times[i]));
        CHECK(rtos_fake.last_wait != portMAX_DELAY);
        CHECK(TMC2209_Os_MutexUnlock(&mutex) == TMC2209_OK);
        sem.available = pdTRUE;
        CHECK(TMC2209_Os_SemTake(&sem, times[i]) == TMC2209_OK);
        CHECK(rtos_fake.last_wait == expected_ticks(times[i]));
        unsigned before = rtos_fake.delays;
        CHECK(TMC2209_Os_DelayMs(times[i]) == TMC2209_OK);
        if (times[i] == 0u) CHECK(rtos_fake.delays == before);
        else CHECK(rtos_fake.last_delay == expected_ticks(times[i]));
    }
    uint32_t seed = 0x2209F00Du, completed = 0u;
    for (counts.case_index = 0u; counts.case_index < 10000u; ++counts.case_index) {
        uint32_t ms = seeded_random(&seed);
        CHECK(TMC2209_Os_MutexLock(&mutex, ms) == TMC2209_OK);
        CHECK(rtos_fake.last_wait == expected_ticks(ms));
        CHECK(rtos_fake.last_wait != portMAX_DELAY);
        CHECK(TMC2209_Os_MutexUnlock(&mutex) == TMC2209_OK);
        ++completed;
    }
    printf("RTOS timeout properties seed=0x2209F00D completed=%" PRIu32 "\n", completed);
}

static void test_tick_wraps(void)
{
    rtos_reset();
    const uint64_t wrap = UINT64_C(1) << FAKE_TICK_BITS;
    const uint64_t samples[] = {0u, 1u, 17u, wrap - 1u, wrap, wrap + 1u,
                                3u * wrap + 999u, UINT64_C(1234567890123)};
    for (size_t i = 0u; i < sizeof(samples) / sizeof(samples[0]); ++i) {
        rtos_fake.total_ticks = samples[i];
        CHECK(TMC2209_Os_GetTickMs() ==
              (uint32_t)((samples[i] * 1000u) / FAKE_TICK_RATE));
    }
    rtos_fake.total_ticks = wrap - 1u;
    uint32_t before = TMC2209_Os_GetTickMs();
    rtos_fake.total_ticks += FAKE_TICK_RATE;
    CHECK((uint32_t)(TMC2209_Os_GetTickMs() - before) == 1000u);
}

int main(void)
{
    RUN(test_create_delete);
    RUN(test_waits_and_failures);
    RUN(test_tick_wraps);
    printf("TickType_t=%u, configTICK_RATE_HZ=%u, dynamic=%u\n",
           (unsigned)(sizeof(TickType_t) * 8u), (unsigned)FAKE_TICK_RATE,
           (unsigned)configSUPPORT_DYNAMIC_ALLOCATION);
    SUMMARY("Production FreeRTOS OS");
    return EXIT_SUCCESS;
}
