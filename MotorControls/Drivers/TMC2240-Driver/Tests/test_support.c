#include "test_support.h"
#include <inttypes.h>

uint64_t test_assertions;
uint32_t test_seed = UINT32_C(0x2240C0DE);

void test_failure(const char *expression, const char *file, int line)
{
    fprintf(stderr, "%s:%d: FAIL %s (seed state 0x%08" PRIX32 ")\n",
            file, line, expression, test_seed);
    exit(EXIT_FAILURE);
}

uint32_t test_random(void)
{
    test_seed ^= test_seed << 13U;
    test_seed ^= test_seed >> 17U;
    test_seed ^= test_seed << 5U;
    return test_seed;
}

void test_summary(const char *name, uint32_t properties, uint32_t recovery)
{
    printf("%s: %" PRIu64 " assertions, %" PRIu32 " seeded properties, %"
           PRIu32 " recovery iterations; seed=0x2240C0DE\n",
           name, test_assertions, properties, recovery);
}
